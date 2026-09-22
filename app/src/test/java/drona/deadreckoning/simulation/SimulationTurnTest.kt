package drona.deadreckoning.simulation

import drona.deadreckoning.util.OSRMRouteFetcher
import drona.deadreckoning.util.RouteMapMatcher
import org.junit.Test
import org.osmdroid.util.GeoPoint
import kotlin.math.abs

class SimulationTurnTest {

    @Test
    fun testTurnBehaviorDuringBlackout() {
        val option = SimulationController.CANONICAL_ROUTES.first()
        val fetched = OSRMRouteFetcher.generateStreetGridRoute(option.start, option.end, option.name)
        val routePoints = fetched.routePoints
        println("Route points count: ${routePoints.size}")
        routePoints.forEachIndexed { idx, pt ->
            println("Waypoint $idx: (${pt.latitude}, ${pt.longitude})")
        }

        val config = SimulationConfig(
            blackoutStartPct = 0.10f,
            blackoutEndPct = 0.20f,
            randomSeed = 26168L
        )
        val trajectory = GroundTruthTrajectoryGenerator.generate(
            routePoints = routePoints,
            targetSpeedMps = config.vehicleSpeedKmh / 3.6,
            dtSeconds = config.dtSeconds
        )

        println("Trajectory steps count: ${trajectory.size}")
        val blackoutStart = (trajectory.size * 0.10).toInt()
        val blackoutEnd = (trajectory.size * 0.20).toInt()
        println("Blackout from index $blackoutStart to $blackoutEnd")

        val hybridEstimator = drona.deadreckoning.fusion.VehicleHierarchicalHybridEstimator()
        val sensorGenerator = SyntheticSensorGenerator(config)
        val firstGt = trajectory.first()
        val modelFile = drona.deadreckoning.support.locateAsset("src/main/assets/ml/v9_adaptive_projection.onnx")
        val manifestFile = drona.deadreckoning.support.locateAsset("src/main/assets/ml/v9_manifest.json")
        val normalizationFile = drona.deadreckoning.support.locateAsset("src/main/assets/ml/v9_normalization.json")

        val gson = com.google.gson.Gson()
        val v9Manifest = gson.fromJson(manifestFile.readText(), drona.deadreckoning.ml.V9Manifest::class.java)
        val v9Norm = gson.fromJson(normalizationFile.readText(), drona.deadreckoning.ml.V9Normalization::class.java)
        val v9Engine = drona.deadreckoning.ml.V9AdaptiveProjectionEngine(
            modelBytes = modelFile.readBytes(),
            manifest = v9Manifest,
            normalization = v9Norm
        )
        v9Engine.reset(firstGt.speedMps.toFloat(), Math.toRadians(firstGt.headingDegrees).toFloat())
        hybridEstimator.reset(firstGt.position, firstGt.speedMps, firstGt.headingDegrees, 2.0)

        var maxHybridError = 0.0
        var maxCrossTrack = 0.0
        var currentMatchedSegmentIndex = 0
        var maxFgoDist = 0.0
        var v9ProjectionsCount = 0

        // Precompute cumulative distance along route waypoints
        val waypointDistances = mutableListOf<Double>()
        var totalDist = 0.0
        waypointDistances.add(0.0)
        for (i in 0 until routePoints.size - 1) {
            totalDist += routePoints[i].distanceToAsDouble(routePoints[i + 1])
            waypointDistances.add(totalDist)
        }

        var odometerMeters = 0.0
        for (idx in 0 until blackoutEnd + 50) {
            val gt = trajectory[idx]
            val imuStep = sensorGenerator.generateImuStep(gt, config.dtSeconds)
            val aiSpeed = sensorGenerator.generateAiSpeed(gt)
            val gnssObs = sensorGenerator.generateGnss(gt, idx * config.dtSeconds)
            val inBlackout = gnssObs.isBlackout

            odometerMeters += imuStep.forwardMeters

            // Physical vehicle acceleration (centripetal during turns)
            val trueYawRateRad = Math.toRadians(gt.yawRateDps)
            val centripetalAccel = (gt.speedMps * trueYawRateRad).toFloat()
            val aFwd = gt.accelMps2.toFloat()
            val aLat = centripetalAccel

            // 1. Ingest into V9 ML Engine continuously on every step
            val v9Result = v9Engine.addSample(
                aFwd = aFwd,
                aLat = aLat,
                wYaw = (imuStep.headingDeltaRadians / imuStep.intervalSeconds).toFloat(),
                stepDisplacementMeters = imuStep.forwardMeters.toFloat(),
                stepHeadingDeltaRad = imuStep.headingDeltaRadians.toFloat(),
                isStationary = gt.speedMps < 0.3
            )

            // 2. Step Hybrid Estimator
            hybridEstimator.predict(
                forwardMeters = imuStep.forwardMeters,
                lateralMeters = imuStep.lateralMeters,
                headingDeltaRadians = imuStep.headingDeltaRadians,
                intervalSeconds = imuStep.intervalSeconds
            )
            hybridEstimator.updateSpeed(aiSpeed.speedMps, aiSpeed.uncertaintyMps)

            val gnssPos = gnssObs.position
            if (!inBlackout && gnssPos != null) {
                hybridEstimator.updateGnss(
                    position = gnssPos,
                    speedMps = gt.speedMps,
                    headingDegrees = gt.headingDegrees,
                    accuracyMeters = gnssObs.accuracyMeters
                )
            } else if (inBlackout && v9Result != null && v9Result.projectionTriggered) {
                v9ProjectionsCount++
                val corrE = v9Result.errorStateCorrection[0] * v9Result.confidence
                val corrN = v9Result.errorStateCorrection[1] * v9Result.confidence
                val corrV = v9Result.errorStateCorrection[2] * v9Result.confidence
                val corrPsi = v9Result.errorStateCorrection[3] * v9Result.confidence
                hybridEstimator.applyErrorStateCorrection(
                    deltaEastMeters = corrE.toDouble(),
                    deltaNorthMeters = corrN.toDouble(),
                    deltaSpeedMps = corrV.toDouble(),
                    deltaHeadingRad = corrPsi.toDouble(),
                    confidence = v9Result.confidence
                )
            }

            val hybridState = hybridEstimator.state()!!
            val hybridPos = hybridState.position
            val hybridHeading = hybridState.headingDegrees

            // Use RouteMapMatcher exactly as SimulationController does
            val routeMatch = RouteMapMatcher.match(
                point = hybridPos,
                route = routePoints,
                headingDegrees = hybridHeading,
                preferredSegmentIndex = currentMatchedSegmentIndex
            )
            if (routeMatch != null) {
                currentMatchedSegmentIndex = routeMatch.segmentIndex
            }

            val isTurning = abs(imuStep.rawGyroYawRateDps) > 2.0 ||
                hybridEstimator.dominantMotionMode == drona.deadreckoning.fusion.VehicleFusionImmUkf.MotionMode.CONSTANT_TURN_RATE

            if (inBlackout && routeMatch != null) {
                hybridEstimator.updateMapConstraint(
                    matchedPosition = routeMatch.point,
                    roadBearingDegrees = if (isTurning) null else routeMatch.bearingDegrees,
                    confidence = routeMatch.confidence
                )
                val roadCand = drona.deadreckoning.data.RoadCandidate(
                    roadName = config.routeName,
                    point = routeMatch.point,
                    distanceMeters = routeMatch.distanceMeters,
                    wayId = 1001L + currentMatchedSegmentIndex,
                    bearingDegrees = routeMatch.bearingDegrees ?: hybridHeading,
                    oneWay = false
                )
                hybridEstimator.updateMapCandidates(listOf(roadCand))
            }

            val err = hybridPos.distanceToAsDouble(gt.position)
            val crossTrackErr = routeMatch?.distanceMeters ?: 0.0
            if (inBlackout && crossTrackErr > maxCrossTrack) {
                maxCrossTrack = crossTrackErr
            }
            if (inBlackout && err > maxHybridError) {
                maxHybridError = err
            }

            if (idx % 100 == 0 || (idx in 660..700 && idx % 10 == 0)) {
                println("STEP $idx (inBlackout=$inBlackout):")
                println("  gt: pos=(${gt.position.latitude}, ${gt.position.longitude}), spd=${gt.speedMps}, head=${gt.headingDegrees}, yawRate=${gt.yawRateDps}")
                println("  hybrid: pos=(${hybridPos.latitude}, ${hybridPos.longitude}), spd=${hybridState.speedMps}, head=${hybridHeading}")
                println("  err=$err m, crossTrack=$crossTrackErr m, segIdx=$currentMatchedSegmentIndex")
            }
        }

        println("MAX TOTAL ERROR DURING BLACKOUT: ${maxHybridError} m")
        println("MAX CROSS-TRACK ERROR DURING BLACKOUT: ${maxCrossTrack} m")
        println("V9 PATP PROJECTIONS FIRED: ${v9ProjectionsCount}")
    }

    private fun normalizeAngleDelta(delta: Double): Double {
        var d = delta
        while (d > 180.0) d -= 360.0
        while (d < -180.0) d += 360.0
        return d
    }

    private fun project(point: GeoPoint, start: GeoPoint, end: GeoPoint): GeoPoint {
        val latitudeScale = 111_111.0
        val longitudeScale = latitudeScale * kotlin.math.cos(Math.toRadians(point.latitude))
        val bx = (end.longitude - start.longitude) * longitudeScale
        val by = (end.latitude - start.latitude) * latitudeScale
        val px = (point.longitude - start.longitude) * longitudeScale
        val py = (point.latitude - start.latitude) * latitudeScale
        val lengthSquared = bx * bx + by * by
        if (lengthSquared == 0.0) return start
        val fraction = ((px * bx + py * by) / lengthSquared).coerceIn(0.0, 1.0)
        return GeoPoint(start.latitude + by * fraction / latitudeScale, start.longitude + bx * fraction / longitudeScale)
    }
}
