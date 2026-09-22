package drona.deadreckoning

import java.io.File
import java.util.Random
import drona.deadreckoning.fusion.HeadingPolicy
import drona.deadreckoning.fusion.MapConstraintConfig
import drona.deadreckoning.fusion.NonHolonomicConfig
import drona.deadreckoning.fusion.VehicleFusionEkf
import drona.deadreckoning.matching.HiddenMarkovRoadMatcher
import drona.deadreckoning.support.BlackoutFixture
import drona.deadreckoning.support.IdrOnnxModel
import drona.deadreckoning.support.ModelContract
import drona.deadreckoning.support.MotionModel
import drona.deadreckoning.support.PersistenceModel
import drona.deadreckoning.support.PolylineRoadNetwork
import drona.deadreckoning.support.V8OnnxModel
import drona.deadreckoning.support.median
import drona.deadreckoning.support.percentile
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.osmdroid.util.GeoPoint

/**
 * IDR-V1 & V8 Baseline End-to-End GNSS Blackout Measurement Suite.
 *
 * Evaluates the primary IDR-V1 heteroscedastic neural motion engine against the V8 baseline
 * across simulated open-loop GNSS outages. Paired with [PinoBlackoutAblationTest] (PINO-DR v3)
 * and [V9AdaptiveProjectionEngineTest] (v9 Adaptive Projection DR).
 *
 * ## What is actually being measured
 *
 * Per-window regression error is not the quantity of interest for dead reckoning. Small
 * biases compound through integration, which is how the shipped V8 model reached a
 * respectable-looking 1.85 m per-window RMSE and still drifted more than a kilometre in a
 * minute. So each configuration is driven open loop through a simulated outage and judged
 * on final position error and on error as a percentage of distance travelled.
 *
 * ## Protocol
 *
 * For every blackout start point:
 *  1. Seed the estimator from ground truth, as a real outage would from the last trusted fix.
 *  2. Propagate using only model output and, where the policy calls for it, gyro. No truth
 *     enters after the seed. The model's own speed estimate, filtered through the estimator,
 *     becomes the next window's input, which is what happens on the phone.
 *  3. Record error at every window boundary, so the 10, 20, 30 and 60 second figures are
 *     readings from one continuous run rather than four independent runs.
 *
 * Windows abut exactly (stride = window - 1). Overlapping them would count the same motion
 * repeatedly; `LEGACY_V8` keeping a non-overlapping stride is the same constraint.
 *
 * Blackout start points are drawn once from a fixed seed and reused across every
 * configuration, so differences between rows are attributable to the configuration.
 *
 * ## Honest limits
 *
 * The road polyline is derived from this session's own ground truth, so the map-matching row
 * is an upper bound rather than an expectation. The gyro scale is fitted on this session.
 * There is one test session. All three are printed with the results.
 */
class BlackoutAblationTest {

    private companion object {
        /** Longest blackout to simulate. Shorter horizons are read from the same run. */
        const val MAX_BLACKOUT_SECONDS = 60.0

        /** Horizons to report, seconds. */
        val HORIZONS = listOf(10.0, 20.0, 30.0, 60.0)

        const val BLACKOUT_COUNT = 90
        const val BLACKOUT_SEED = 20260831L

        /** Only start where the vehicle is moving; a blackout while parked flatters everyone. */
        const val MIN_START_SPEED_MPS = 3.0

        /** Accuracy attributed to the last trusted fix before the outage. */
        const val SEED_ACCURACY_METERS = 5.0

        /** Speed uncertainty assumed for a model with no trained variance head, i.e. V8. */
        const val UNTRAINED_SPEED_SIGMA_MPS = 2.0

        const val SIH_DRIFT_TARGET_PERCENT = 10.0
    }

    /** One configuration of the pipeline. */
    private data class Configuration(
        val id: String,
        val description: String,
        val model: MotionModel,
        val headingPolicy: HeadingPolicy,
        val nonHolonomic: NonHolonomicConfig,
        val mapConstraint: MapConstraintConfig,
        val useGyro: Boolean
    )

    /** Error at one window boundary during one blackout. */
    private data class Reading(
        val elapsedSeconds: Double,
        val errorMeters: Double,
        val travelledMeters: Double,
        val latitude: Double,
        val longitude: Double
    )

    private data class Summary(
        val medianErrorMeters: Double,
        val p90ErrorMeters: Double,
        val medianDriftPercent: Double,
        val samples: Int
    )

    /**
     * Estimator counters at the end of one blackout.
     *
     * These turn an outcome into an explanation. A map constraint that appears not to help
     * could be applying corrections that do not help, or could be rejecting almost every
     * match at the innovation gate; those are different problems with different fixes, and
     * the outcome column alone cannot tell them apart.
     */
    private data class Diagnostics(
        val nonHolonomicUpdates: Int,
        val mapUpdates: Int,
        val mapRejections: Int,
        val headingRejections: Int,
        val finalCrossTrackSigma: Double,
        val finalHeadingSigma: Double
    )

    private data class Outcome(val readings: List<Reading>, val diagnostics: Diagnostics)

    @Test
    fun `blackout ablation on held-out session`() {
        val fixture = BlackoutFixture.load()
        val payload = fixture.payload

        val idr = IdrOnnxModel()
        assertTrue(
            "The shipped IDR-V1 artifact is missing required outputs",
            idr.outputsPresent()
        )
        assertEquals(
            "Fixture and shipped artifact disagree on the preprocessing contract",
            payload.contract.preprocessing_version,
            idr.preprocessingVersion
        )

        val v8Available = runCatching { V8OnnxModel() }.getOrNull()
        val road = PolylineRoadNetwork(payload.road.ways)

        printHeader(payload, road)
        assertInferenceParity(fixture, idr, v8Available)

        val maxWindows = (MAX_BLACKOUT_SECONDS / fixture.spanSeconds).toInt()
        val starts = chooseStarts(fixture, maxWindows)
        assertTrue("Not enough moving start points to measure anything", starts.size >= 20)

        val configurations = buildList {
            add(
                Configuration(
                    "P", "persistence + EKF (no network, no model)",
                    PersistenceModel(fixture.spanSeconds),
                    HeadingPolicy.MODEL_ONLY,
                    NonHolonomicConfig.DISABLED, MapConstraintConfig.DISABLED, useGyro = false
                )
            )
            v8Available?.let {
                add(
                    Configuration(
                        "A", "V8 + EKF",
                        it, HeadingPolicy.MODEL_ONLY,
                        NonHolonomicConfig.DISABLED, MapConstraintConfig.DISABLED, useGyro = false
                    )
                )
            }
            add(
                Configuration(
                    "B", "IDR-V1 + EKF",
                    idr, HeadingPolicy.MODEL_ONLY,
                    NonHolonomicConfig.DISABLED, MapConstraintConfig.DISABLED, useGyro = false
                )
            )
            add(
                Configuration(
                    "C", "B + non-holonomic constraint (closed form, no gyro)",
                    idr, HeadingPolicy.MODEL_ONLY,
                    NonHolonomicConfig(), MapConstraintConfig.DISABLED, useGyro = false
                )
            )
            // Gyro is enabled here purely to feed the constraint's heading shape integrals.
            // Under MODEL_ONLY the gyro cannot touch heading, so this isolates the effect of
            // an arbitrary measured yaw profile against the constant-rate closed form. Turning
            // gyro on together with the map, as an earlier version of this test did, conflated
            // the two and credited the map with whatever the shape integrals contributed.
            add(
                Configuration(
                    "C2", "C but constraint uses measured gyro shape integrals",
                    idr, HeadingPolicy.MODEL_ONLY,
                    NonHolonomicConfig(), MapConstraintConfig.DISABLED, useGyro = true
                )
            )
            add(
                Configuration(
                    "D", "C2 + map constraint (optimistic road proxy)",
                    idr, HeadingPolicy.MODEL_ONLY,
                    NonHolonomicConfig(), MapConstraintConfig(), useGyro = true
                )
            )
            // Dm and Dn differ only in whether the non-holonomic constraint is on, with gyro
            // off in both, so the pair isolates the constraint's contribution under the map
            // constraint without the shape integrals confounding it.
            add(
                Configuration(
                    "Dn", "C + map constraint, no gyro",
                    idr, HeadingPolicy.MODEL_ONLY,
                    NonHolonomicConfig(), MapConstraintConfig(), useGyro = false
                )
            )
            add(
                Configuration(
                    "Dm", "B + map constraint only, no non-holonomic constraint, no gyro",
                    idr, HeadingPolicy.MODEL_ONLY,
                    NonHolonomicConfig.DISABLED, MapConstraintConfig(), useGyro = false
                )
            )
            add(
                Configuration(
                    "G1", "D but heading = GYRO_WITH_MODEL_UPDATE (production default)",
                    idr, HeadingPolicy.GYRO_WITH_MODEL_UPDATE,
                    NonHolonomicConfig(), MapConstraintConfig(), useGyro = true
                )
            )
            add(
                Configuration(
                    "G2", "D but heading = GYRO_ONLY",
                    idr, HeadingPolicy.GYRO_ONLY,
                    NonHolonomicConfig(), MapConstraintConfig(), useGyro = true
                )
            )
        }

        val results = LinkedHashMap<String, Map<Double, Summary>>()
        val descriptions = LinkedHashMap<String, String>()
        val runsByConfiguration = LinkedHashMap<String, List<List<Reading>>>()
        val diagnosticsByConfiguration = LinkedHashMap<String, List<Diagnostics>>()
        for (configuration in configurations) {
            val outcomes = starts.map { start ->
                runBlackout(fixture, configuration, road, start, maxWindows)
            }
            runsByConfiguration[configuration.id] = outcomes.map { it.readings }
            diagnosticsByConfiguration[configuration.id] = outcomes.map { it.diagnostics }
            results[configuration.id] = summarise(outcomes.map { it.readings })
            descriptions[configuration.id] = configuration.description
        }

        printResults(results, descriptions, starts.size, fixture.spanSeconds)
        printDiagnostics(diagnosticsByConfiguration, results, maxWindows)
        printVerdict(results, payload)
        writeArtifacts(fixture, starts, runsByConfiguration, results, descriptions)

        assertRegressions(results, v8Available != null)

        idr.close()
        v8Available?.close()
    }

    /**
     * Confirm JVM inference reproduces the Python reference.
     *
     * Without this, a normalisation or channel-order mistake on either side would silently
     * shift every number in the table. The reference was produced with ground-truth initial
     * speed, so the same seed is used here and only here.
     */
    private fun assertInferenceParity(
        fixture: BlackoutFixture.Data,
        idr: IdrOnnxModel,
        v8: V8OnnxModel?
    ) {
        val checked = fixture.windows.indices.step(fixture.windows.size / 25 + 1).take(25)
        var worstIdr = 0.0
        var worstV8 = 0.0

        for (index in checked) {
            val window = fixture.windows[index]
            window.idrRef?.let { reference ->
                val actual = idr.predict(
                    fixture.window(window.start, ModelContract.IDR_V1), window.startSpeedMps
                )
                worstIdr = maxOf(
                    worstIdr,
                    kotlin.math.abs(actual.forward - reference.forward),
                    kotlin.math.abs(actual.lateral - reference.lateral),
                    kotlin.math.abs(actual.speed - reference.speed),
                    kotlin.math.abs(actual.headingDelta - reference.headingDelta)
                )
            }
            if (v8 != null) {
                window.v8Ref?.let { reference ->
                    val actual = v8.predict(
                        fixture.window(window.start, ModelContract.LEGACY_V8), window.startSpeedMps
                    )
                    worstV8 = maxOf(
                        worstV8,
                        kotlin.math.abs(actual.forward - reference.forward),
                        kotlin.math.abs(actual.lateral - reference.lateral),
                        kotlin.math.abs(actual.speed - reference.speed)
                    )
                }
            }
        }

        println("inference parity against the Python export, worst absolute difference")
        println("  IDR-V1 %.5f".format(worstIdr))
        if (v8 != null) println("  V8     %.5f".format(worstV8))
        println()

        // The reference is rounded to 4 decimals on export, so the floor is 1e-4.
        assertTrue(
            "JVM IDR-V1 inference diverges from the Python export by $worstIdr; " +
                "normalisation or channel order disagree",
            worstIdr < 0.02
        )
        if (v8 != null) {
            assertTrue(
                "JVM V8 inference diverges from the Python export by $worstV8",
                worstV8 < 0.05
            )
        }
    }

    /**
     * Pick blackout start indices once, then reuse them for every configuration.
     *
     * Drawing separately per configuration would let sampling noise masquerade as a
     * difference between configurations.
     */
    private fun chooseStarts(fixture: BlackoutFixture.Data, maxWindows: Int): List<Int> {
        val limit = fixture.windows.size - maxWindows - 1
        val candidates = (0 until limit).filter {
            fixture.windows[it].startSpeedMps > MIN_START_SPEED_MPS
        }
        if (candidates.size <= BLACKOUT_COUNT) return candidates
        val shuffled = candidates.toMutableList()
        val random = Random(BLACKOUT_SEED)
        for (index in shuffled.indices.reversed()) {
            val swap = random.nextInt(index + 1)
            val held = shuffled[index]
            shuffled[index] = shuffled[swap]
            shuffled[swap] = held
        }
        return shuffled.take(BLACKOUT_COUNT).sorted()
    }

    /**
     * Roll one blackout forward through the real estimator.
     *
     * Call order matters and mirrors the runtime: gyro samples advance heading across the
     * window first, then the window displacement is consumed, at which point the estimator
     * rotates it by the heading as it was when the window began and re-anchors.
     */
    private fun runBlackout(
        fixture: BlackoutFixture.Data,
        configuration: Configuration,
        road: PolylineRoadNetwork,
        startIndex: Int,
        windowCount: Int
    ): Outcome {
        val first = fixture.windows[startIndex]
        val ekf = VehicleFusionEkf(
            headingPolicy = configuration.headingPolicy,
            nonHolonomic = configuration.nonHolonomic,
            mapConstraint = configuration.mapConstraint
        )
        ekf.reset(
            GeoPoint(first.startLat, first.startLon),
            first.startSpeedMps,
            first.startHeadingDeg,
            SEED_ACCURACY_METERS
        )
        val matcher = if (configuration.mapConstraint.enabled) HiddenMarkovRoadMatcher() else null

        var speedInput = first.startSpeedMps
        var travelled = 0.0
        var elapsed = 0.0
        val readings = ArrayList<Reading>(windowCount)

        for (offset in 0 until windowCount) {
            val window = fixture.windows[startIndex + offset]

            if (configuration.useGyro) {
                window.gyroRates.forEach { rate ->
                    ekf.predictGyro(rate, fixture.sampleIntervalSeconds)
                }
            }

            val prediction = configuration.model.predict(
                fixture.window(window.start, configuration.model.contract),
                speedInput
            )
            ekf.predict(
                forwardMeters = prediction.forward,
                lateralMeters = prediction.lateral,
                headingDeltaRadians = prediction.headingDelta,
                intervalSeconds = fixture.spanSeconds
            )
            ekf.updateSpeed(prediction.speed, prediction.speedSigma ?: UNTRAINED_SPEED_SIGMA_MPS)

            if (matcher != null) {
                val current = ekf.state()
                val candidates = road.candidates(current.position)
                matcher.update(current.position, candidates)?.let { matched ->
                    ekf.updateMapConstraint(
                        matchedPosition = matched.candidate.point,
                        roadBearingDegrees = matched.candidate.bearingDegrees,
                        confidence = matched.confidence
                    )
                }
            }

            val state = ekf.state()
            // Close the speed loop through the estimator, as the runtime does. Re-seeding
            // from truth here would leak ground truth into every window of the outage.
            speedInput = state.speedMps

            travelled += window.pathMeters
            elapsed += fixture.spanSeconds
            readings += Reading(
                elapsedSeconds = elapsed,
                errorMeters = BlackoutFixture.haversineMeters(
                    state.position, window.endLat, window.endLon
                ),
                travelledMeters = travelled,
                latitude = state.position.latitude,
                longitude = state.position.longitude
            )
        }

        val finalState = ekf.state()
        return Outcome(
            readings = readings,
            diagnostics = Diagnostics(
                nonHolonomicUpdates = ekf.nonHolonomicUpdates,
                mapUpdates = ekf.mapConstraintUpdates,
                mapRejections = ekf.rejectedMapConstraints,
                headingRejections = ekf.rejectedHeadingUpdates,
                finalCrossTrackSigma = finalState.crossTrackUncertaintyMeters,
                finalHeadingSigma = finalState.headingUncertaintyDegrees
            )
        )
    }

    /** Reduce per-blackout readings to a summary at each reporting horizon. */
    private fun summarise(runs: List<List<Reading>>): Map<Double, Summary> {
        val summary = LinkedHashMap<Double, Summary>()
        for (horizon in HORIZONS) {
            val errors = ArrayList<Double>()
            val drifts = ArrayList<Double>()
            for (run in runs) {
                val reading = run.lastOrNull { it.elapsedSeconds <= horizon + 1e-9 } ?: continue
                errors += reading.errorMeters
                if (reading.travelledMeters > 1.0) {
                    drifts += reading.errorMeters / reading.travelledMeters * 100.0
                }
            }
            if (errors.isEmpty()) continue
            summary[horizon] = Summary(
                medianErrorMeters = errors.median(),
                p90ErrorMeters = errors.percentile(0.90),
                medianDriftPercent = drifts.median(),
                samples = errors.size
            )
        }
        return summary
    }

    private fun printHeader(payload: BlackoutFixture.Payload, road: PolylineRoadNetwork) {
        println()
        println("=".repeat(96))
        println("FULL-PIPELINE GNSS BLACKOUT ABLATION")
        println("=".repeat(96))
        println("session            ${payload.session} (held out: unseen driver, unseen vehicle)")
        println("contract           ${payload.contract.preprocessing_version}, " +
            "window ${payload.contract.window_samples} @ ${payload.contract.sample_rate_hz} Hz, " +
            "span ${"%.1f".format(payload.contract.window_span_seconds)} s, " +
            "chain stride ${payload.contract.chain_stride}")
        println("windows            ${payload.windows.size}")
        println("correspondence     ${"%.3f".format(payload.contract.correspondence)} " +
            "(gyro vs vehicle yaw rate, lag ${payload.contract.lag_samples} samples)")
        println("gyro calibration   channel ${payload.gyroCalibration.channel} " +
            "(${payload.gyroCalibration.channelName}), " +
            "scale ${"%+.4f".format(payload.gyroCalibration.scale)}, " +
            "corr ${"%+.3f".format(payload.gyroCalibration.correlation)}")
        println("gyro heading error ${"%.2f".format(payload.gyroCalibration.medianHeadingError10sDeg)} " +
            "deg median over 10 s")
        println("road proxy         ${road.wayCount} ways, ${road.nodeCount} nodes, " +
            "${"%.0f".format(payload.road.spacingMeters)} m spacing, " +
            "${"%.0f".format(payload.road.noiseSigmaMeters)} m noise")
        println("estimator          real VehicleFusionEkf, real HiddenMarkovRoadMatcher")
        println("model artifacts    ${File("src/main/assets/ml").absolutePath}")
        println()
    }

    private fun printResults(
        results: Map<String, Map<Double, Summary>>,
        descriptions: Map<String, String>,
        blackouts: Int,
        spanSeconds: Double
    ) {
        println("-".repeat(96))
        println("MEDIAN FINAL POSITION ERROR AND DRIFT, $blackouts blackouts per configuration")
        println("horizons are reached in whole windows of ${"%.1f".format(spanSeconds)} s")
        println("-".repeat(96))

        val header = StringBuilder("%-4s".format("cfg"))
        HORIZONS.forEach { header.append("%12s%9s".format("${it.toInt()}s err", "drift")) }
        println(header)
        println("-".repeat(96))

        for ((id, perHorizon) in results) {
            val row = StringBuilder("%-4s".format(id))
            for (horizon in HORIZONS) {
                val entry = perHorizon[horizon]
                if (entry == null) {
                    row.append("%12s%9s".format("-", "-"))
                } else {
                    row.append("%11.1fm%8.1f%%".format(entry.medianErrorMeters, entry.medianDriftPercent))
                }
            }
            println(row)
        }

        println()
        println("90th percentile error, metres")
        val p90Header = StringBuilder("%-4s".format("cfg"))
        HORIZONS.forEach { p90Header.append("%12s".format("${it.toInt()}s")) }
        println(p90Header)
        for ((id, perHorizon) in results) {
            val row = StringBuilder("%-4s".format(id))
            for (horizon in HORIZONS) {
                val entry = perHorizon[horizon]
                row.append(if (entry == null) "%12s".format("-") else "%11.1fm".format(entry.p90ErrorMeters))
            }
            println(row)
        }

        println()
        println("configurations")
        descriptions.forEach { (id, description) -> println("  %-4s %s".format(id, description)) }
        println("  E    NOT IMPLEMENTED. No AI INS error-correction model exists in this repository,")
        println("       so the fifth ablation stage cannot be measured and is not claimed.")
        println()
    }

    /**
     * Why each stage behaved the way it did, per 60 s blackout.
     *
     * Counts are means across blackouts. The map columns separate corrections that were
     * applied from matches thrown out at the confidence or innovation gate, because a stage
     * that looks inert may in fact be rejecting nearly everything.
     */
    private fun printDiagnostics(
        diagnostics: Map<String, List<Diagnostics>>,
        results: Map<String, Map<Double, Summary>>,
        windowCount: Int
    ) {
        println("-".repeat(96))
        println("ESTIMATOR BEHAVIOUR PER BLACKOUT, mean across blackouts of $windowCount windows")
        println("-".repeat(96))
        println("%-4s%10s%10s%12s%12s%14s%13s".format(
            "cfg", "NHC", "map ok", "map reject", "hdg reject", "cross sigma", "hdg sigma"
        ))
        println("-".repeat(96))
        for ((id, samples) in diagnostics) {
            if (samples.isEmpty()) continue
            println("%-4s%10.1f%10.1f%12.1f%12.1f%13.1fm%9.1f deg".format(
                id,
                samples.map { it.nonHolonomicUpdates.toDouble() }.average(),
                samples.map { it.mapUpdates.toDouble() }.average(),
                samples.map { it.mapRejections.toDouble() }.average(),
                samples.map { it.headingRejections.toDouble() }.average(),
                samples.map { it.finalCrossTrackSigma }.average(),
                samples.map { it.finalHeadingSigma }.average()
            ))
        }
        println()
        println("COVARIANCE CALIBRATION at the 60 s horizon")
        println("A well-calibrated filter would report a 1-sigma of roughly the median error,")
        println("so a ratio near 1 is the target. Larger means the filter is overconfident and")
        println("the uncertainty it shows the user understates how lost it actually is.")
        println("%-4s%16s%16s%12s".format("cfg", "median error", "reported sigma", "ratio"))
        for ((id, samples) in diagnostics) {
            val error = results[id]?.get(60.0)?.medianErrorMeters ?: continue
            if (samples.isEmpty()) continue
            val sigma = samples.map { it.finalCrossTrackSigma }.average()
            println("%-4s%15.1fm%15.1fm%11.1fx".format(id, error, sigma, error / sigma))
        }
        println()
    }

    private fun printVerdict(
        results: Map<String, Map<Double, Summary>>,
        payload: BlackoutFixture.Payload
    ) {
        println("-".repeat(96))
        println("AGAINST THE SIH TARGET OF UNDER ${SIH_DRIFT_TARGET_PERCENT.toInt()} PERCENT DRIFT")
        println("-".repeat(96))
        for ((id, perHorizon) in results) {
            val verdicts = HORIZONS.mapNotNull { horizon ->
                perHorizon[horizon]?.let { entry ->
                    val pass = entry.medianDriftPercent < SIH_DRIFT_TARGET_PERCENT
                    "${horizon.toInt()}s ${if (pass) "PASS" else "FAIL"}"
                }
            }
            println("  %-4s %s".format(id, verdicts.joinToString("  ")))
        }
        println()
        println("LIMITATIONS, which apply to every number above")
        payload.limitations.forEachIndexed { index, limitation ->
            println("  ${index + 1}. $limitation")
        }
        println("  ${payload.limitations.size + 1}. Ablation stage E is absent, not zero.")
        println("=".repeat(96))
        println()
    }

    /**
     * Persist the summary and a few full trajectories for plotting.
     *
     * The plotter consumes this rather than recomputing anything, so the figures cannot
     * disagree with the table. Two blackouts are exported for the reference configuration:
     * the one closest to its own median 60 s error, and the one closest to its 90th
     * percentile, so the illustration shows a typical case and a bad case rather than a
     * flattering hand-picked one.
     */
    private fun writeArtifacts(
        fixture: BlackoutFixture.Data,
        starts: List<Int>,
        runs: Map<String, List<List<Reading>>>,
        results: Map<String, Map<Double, Summary>>,
        descriptions: Map<String, String>
    ) {
        val reference = "Dn"
        val referenceRuns = runs[reference] ?: return
        val finalErrors = referenceRuns.map { it.lastOrNull()?.errorMeters ?: Double.NaN }
        val typical = closestIndex(finalErrors, finalErrors.median())
        val bad = closestIndex(finalErrors, finalErrors.percentile(0.90))
        val selected = listOf(typical, bad).distinct().filter { it >= 0 }

        val gson = com.google.gson.GsonBuilder().setPrettyPrinting().create()
        val payload = linkedMapOf<String, Any>(
            "session" to fixture.payload.session,
            "spanSeconds" to fixture.spanSeconds,
            "blackouts" to starts.size,
            "horizons" to HORIZONS,
            "limitations" to fixture.payload.limitations,
            "gyroCalibration" to fixture.payload.gyroCalibration,
            "descriptions" to descriptions,
            "summary" to results.mapValues { (_, perHorizon) ->
                perHorizon.mapKeys { it.key.toInt().toString() }.mapValues { (_, entry) ->
                    linkedMapOf(
                        "medianErrorMeters" to entry.medianErrorMeters,
                        "p90ErrorMeters" to entry.p90ErrorMeters,
                        "medianDriftPercent" to entry.medianDriftPercent,
                        "samples" to entry.samples
                    )
                }
            },
            "errorCurves" to runs.mapValues { (_, perStart) ->
                // Median error across blackouts at each window boundary, for error-vs-time.
                val depth = perStart.minOfOrNull { it.size } ?: 0
                (0 until depth).map { step ->
                    linkedMapOf(
                        "elapsedSeconds" to perStart.first()[step].elapsedSeconds,
                        "medianErrorMeters" to perStart.map { it[step].errorMeters }.median(),
                        "medianDriftPercent" to perStart
                            .filter { it[step].travelledMeters > 1.0 }
                            .map { it[step].errorMeters / it[step].travelledMeters * 100.0 }
                            .median()
                    )
                }
            },
            "referenceConfiguration" to reference,
            "trajectories" to selected.map { index ->
                val start = starts[index]
                val windowCount = referenceRuns[index].size
                linkedMapOf(
                    "label" to if (index == typical) "median 60 s error" else "90th percentile 60 s error",
                    "startWindow" to start,
                    "finalErrorMeters" to finalErrors[index],
                    "seedLat" to fixture.windows[start].startLat,
                    "seedLon" to fixture.windows[start].startLon,
                    "truth" to (0 until windowCount).map {
                        val window = fixture.windows[start + it]
                        listOf(window.endLat, window.endLon)
                    },
                    "estimates" to runs.mapValues { (_, perStart) ->
                        perStart[index].map { listOf(it.latitude, it.longitude) }
                    }
                )
            }
        )

        val output = File("../.codex-ml-codes-review/outputs/blackout_ablation.json")
        output.parentFile?.mkdirs()
        output.writeText(gson.toJson(payload))
        println("wrote ${output.absolutePath}")
        println()
    }

    private fun closestIndex(values: List<Double>, target: Double): Int {
        var best = -1
        var bestDistance = Double.MAX_VALUE
        values.forEachIndexed { index, value ->
            if (!value.isFinite()) return@forEachIndexed
            val distance = kotlin.math.abs(value - target)
            if (distance < bestDistance) {
                bestDistance = distance
                best = index
            }
        }
        return best
    }

    /**
     * Guard the deployment decision.
     *
     * These are the only claims the measurement is asked to defend: the replacement model
     * beats the artifact it replaces, and it beats doing nothing. The absolute drift figure
     * is reported rather than asserted, because asserting a target would turn a measurement
     * into a thing that must be made to pass.
     */
    private fun assertRegressions(
        results: Map<String, Map<Double, Summary>>,
        v8Present: Boolean
    ) {
        val idr = requireNotNull(results["B"])
        val persistence = requireNotNull(results["P"])

        for (horizon in HORIZONS) {
            val model = idr[horizon] ?: continue
            val baseline = persistence[horizon] ?: continue
            assertTrue(
                "At ${horizon.toInt()} s IDR-V1 median error ${model.medianErrorMeters} m is not " +
                    "better than persistence ${baseline.medianErrorMeters} m, so the model is " +
                    "not worth deploying",
                model.medianErrorMeters < baseline.medianErrorMeters
            )
            if (v8Present) {
                val v8 = results["A"]?.get(horizon) ?: continue
                assertTrue(
                    "At ${horizon.toInt()} s IDR-V1 median error ${model.medianErrorMeters} m is " +
                        "not better than V8 ${v8.medianErrorMeters} m",
                    model.medianErrorMeters < v8.medianErrorMeters
                )
            }
        }
    }
}
