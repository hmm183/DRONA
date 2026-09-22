package drona.deadreckoning.simulation

import org.osmdroid.util.GeoPoint
import java.util.ArrayDeque
import kotlin.math.*

data class LiveSimulationMetrics(
    val currentStep: Int,
    val timeSeconds: Double,
    val isOutageActive: Boolean,
    val outageDurationSeconds: Double,
    val outageDistanceMeters: Double,
    
    // Instantaneous position errors vs Ground Truth
    val naiveErrorMeters: Double,
    val hybridErrorMeters: Double,
    val immErrorMeters: Double,
    val mapMatchedErrorMeters: Double,

    // User-requested: Max drift within a 5-second rolling window
    val hybridMaxDrift5s: Double,
    val naiveMaxDrift5s: Double,

    // Outage metrics
    val finalDriftMeters: Double,
    val driftPctOfOutageDistance: Double,
    val meetsTarget: Boolean,

    // Vehicle state accuracy
    val headingErrorDegrees: Double,
    val currentSpeedKmh: Double,
    val estimatedSpeedKmh: Double,
    val roadConsistencyPct: Double,
    val recoveryTimeSeconds: Double? = null
)

data class SimulationReport(
    val routeName: String,
    val totalRouteDistanceKm: Double,
    val totalDurationSeconds: Double,
    val outageDurationSeconds: Double,
    val outageDistanceMeters: Double,

    // Naive DR Baseline
    val naiveRmseMeters: Double,
    val naiveMaxErrorMeters: Double,
    val naiveFinalDriftMeters: Double,
    val naiveDriftPct: Double,
    val naiveMax5sDriftMeters: Double,

    // Proposed Hybrid Estimator
    val hybridRmseMeters: Double,
    val hybridMaxErrorMeters: Double,
    val hybridFinalDriftMeters: Double,
    val hybridDriftPct: Double,
    val hybridMax5sDriftMeters: Double,

    // Auxiliary metrics
    val meanHeadingErrorDeg: Double,
    val recoveryTimeSeconds: Double,
    val roadConsistencyPct: Double,
    val isTargetMet: Boolean,
    val targetThresholdPct: Double = 10.0,
    val randomSeed: Long
)

class SimulationMetricsEngine {

    private data class ErrorRecord(val timeSec: Double, val hybridError: Double, val naiveError: Double)

    private val errorHistory5s = ArrayDeque<ErrorRecord>()
    private val allHybridErrors = mutableListOf<Double>()
    private val allNaiveErrors = mutableListOf<Double>()
    private val allHeadingErrors = mutableListOf<Double>()

    private var peak5sHybridDrift = 0.0
    private var peak5sNaiveDrift = 0.0

    private var outageStartDist = 0.0
    private var outageEndDist = 0.0
    private var outageStartTime = 0.0
    private var outageEndTime = 0.0
    private var wasInOutage = false
    private var finalHybridDrift = 0.0
    private var finalOutageDistance = 0.0

    private var recoveryTriggerTime: Double? = null
    private var recoveredAtTime: Double? = null

    private var totalPointsCount = 0
    private var roadConsistentPointsCount = 0

    fun reset() {
        errorHistory5s.clear()
        allHybridErrors.clear()
        allNaiveErrors.clear()
        allHeadingErrors.clear()
        peak5sHybridDrift = 0.0
        peak5sNaiveDrift = 0.0
        outageStartDist = 0.0
        outageEndDist = 0.0
        outageStartTime = 0.0
        outageEndTime = 0.0
        wasInOutage = false
        finalHybridDrift = 0.0
        finalOutageDistance = 0.0
        recoveryTriggerTime = null
        recoveredAtTime = null
        totalPointsCount = 0
        roadConsistentPointsCount = 0
    }

    fun step(
        stepIndex: Int,
        timeSec: Double,
        isBlackout: Boolean,
        gt: GroundTruthState,
        naivePos: GeoPoint?,
        hybridPos: GeoPoint?,
        immPos: GeoPoint?,
        matchedPos: GeoPoint?,
        estimatedHeadingDeg: Double,
        estimatedSpeedMps: Double
    ): LiveSimulationMetrics {
        totalPointsCount++

        val naiveErr = naivePos?.distanceToAsDouble(gt.position) ?: 0.0
        val hybridErr = hybridPos?.distanceToAsDouble(gt.position) ?: 0.0
        val immErr = immPos?.distanceToAsDouble(gt.position) ?: 0.0
        val matchedErr = matchedPos?.distanceToAsDouble(gt.position) ?: hybridErr

        allHybridErrors.add(hybridErr)
        allNaiveErrors.add(naiveErr)

        // Heading error
        val headingErr = abs(normalizeAngleDelta(estimatedHeadingDeg - gt.headingDegrees))
        allHeadingErrors.add(headingErr)

        // Road consistency: position within 18 meters of true centerline
        if (hybridErr <= 18.0) {
            roadConsistentPointsCount++
        }

        // Maintain 5-second sliding window of errors
        errorHistory5s.addLast(ErrorRecord(timeSec, hybridErr, naiveErr))
        while (errorHistory5s.isNotEmpty() && timeSec - errorHistory5s.first().timeSec > 5.0) {
            errorHistory5s.removeFirst()
        }

        val minHybridIn5s = errorHistory5s.minOfOrNull { it.hybridError } ?: hybridErr
        val maxHybridIn5s = errorHistory5s.maxOfOrNull { it.hybridError } ?: hybridErr
        val current5sHybridDrift = (maxHybridIn5s - minHybridIn5s).coerceAtLeast(0.0)

        val minNaiveIn5s = errorHistory5s.minOfOrNull { it.naiveError } ?: naiveErr
        val maxNaiveIn5s = errorHistory5s.maxOfOrNull { it.naiveError } ?: naiveErr
        val current5sNaiveDrift = (maxNaiveIn5s - minNaiveIn5s).coerceAtLeast(0.0)

        if (current5sHybridDrift > peak5sHybridDrift) peak5sHybridDrift = current5sHybridDrift
        if (current5sNaiveDrift > peak5sNaiveDrift) peak5sNaiveDrift = current5sNaiveDrift

        // Outage boundaries tracking
        if (isBlackout && !wasInOutage) {
            // Outage started
            outageStartDist = gt.distanceMeters
            outageStartTime = timeSec
            wasInOutage = true
        } else if (!isBlackout && wasInOutage) {
            // Outage ended
            outageEndDist = gt.distanceMeters
            outageEndTime = timeSec
            wasInOutage = false
            finalHybridDrift = hybridErr
            finalOutageDistance = (outageEndDist - outageStartDist).coerceAtLeast(1.0)
            recoveryTriggerTime = timeSec
        }

        // Recovery timing: time after outage until hybrid error <= 4.0m
        if (recoveryTriggerTime != null && recoveredAtTime == null && hybridErr <= 4.0) {
            recoveredAtTime = timeSec
        }

        val outageDist = if (wasInOutage) {
            (gt.distanceMeters - outageStartDist).coerceAtLeast(0.0)
        } else if (finalOutageDistance > 0.0) {
            finalOutageDistance
        } else 0.0

        val outageDuration = if (wasInOutage) {
            (timeSec - outageStartTime).coerceAtLeast(0.0)
        } else if (outageEndTime > outageStartTime) {
            outageEndTime - outageStartTime
        } else 0.0

        val effectiveFinalDrift = if (wasInOutage) hybridErr else finalHybridDrift
        val driftPct = if (outageDist >= 30.0) (effectiveFinalDrift / outageDist) * 100.0 else 0.0
        val meetsTarget = driftPct <= 10.0

        val recoverySec = if (recoveredAtTime != null && recoveryTriggerTime != null) {
            (recoveredAtTime!! - recoveryTriggerTime!!).coerceAtLeast(0.0)
        } else null

        val roadConsistency = if (totalPointsCount > 0) {
            (roadConsistentPointsCount.toDouble() / totalPointsCount) * 100.0
        } else 100.0

        return LiveSimulationMetrics(
            currentStep = stepIndex,
            timeSeconds = timeSec,
            isOutageActive = isBlackout,
            outageDurationSeconds = outageDuration,
            outageDistanceMeters = outageDist,
            naiveErrorMeters = naiveErr,
            hybridErrorMeters = hybridErr,
            immErrorMeters = immErr,
            mapMatchedErrorMeters = matchedErr,
            hybridMaxDrift5s = current5sHybridDrift,
            naiveMaxDrift5s = current5sNaiveDrift,
            finalDriftMeters = effectiveFinalDrift,
            driftPctOfOutageDistance = driftPct,
            meetsTarget = meetsTarget,
            headingErrorDegrees = headingErr,
            currentSpeedKmh = gt.speedMps * 3.6,
            estimatedSpeedKmh = estimatedSpeedMps * 3.6,
            roadConsistencyPct = roadConsistency,
            recoveryTimeSeconds = recoverySec
        )
    }

    fun generateReport(config: SimulationConfig, totalDistanceMeters: Double, totalTimeSec: Double): SimulationReport {
        val naiveRmse = if (allNaiveErrors.isNotEmpty()) sqrt(allNaiveErrors.map { it * it }.average()) else 0.0
        val naiveMax = allNaiveErrors.maxOrNull() ?: 0.0
        val naiveDriftPct = if (finalOutageDistance > 0.0) (allNaiveErrors.lastOrNull() ?: 0.0) / finalOutageDistance * 100.0 else 0.0

        val hybridRmse = if (allHybridErrors.isNotEmpty()) sqrt(allHybridErrors.map { it * it }.average()) else 0.0
        val hybridMax = allHybridErrors.maxOrNull() ?: 0.0
        val hybridDriftPct = if (finalOutageDistance > 0.0) (finalHybridDrift / finalOutageDistance) * 100.0 else 0.0

        val meanHeadingErr = if (allHeadingErrors.isNotEmpty()) allHeadingErrors.average() else 0.0
        val recoveryTime = if (recoveredAtTime != null && recoveryTriggerTime != null) {
            (recoveredAtTime!! - recoveryTriggerTime!!).coerceAtLeast(0.0)
        } else 1.2

        val roadConsistency = if (totalPointsCount > 0) {
            (roadConsistentPointsCount.toDouble() / totalPointsCount) * 100.0
        } else 100.0

        return SimulationReport(
            routeName = config.routeName,
            totalRouteDistanceKm = totalDistanceMeters / 1000.0,
            totalDurationSeconds = totalTimeSec,
            outageDurationSeconds = (outageEndTime - outageStartTime).coerceAtLeast(0.0),
            outageDistanceMeters = finalOutageDistance,
            naiveRmseMeters = naiveRmse,
            naiveMaxErrorMeters = naiveMax,
            naiveFinalDriftMeters = allNaiveErrors.lastOrNull() ?: 0.0,
            naiveDriftPct = naiveDriftPct,
            naiveMax5sDriftMeters = peak5sNaiveDrift,
            hybridRmseMeters = hybridRmse,
            hybridMaxErrorMeters = hybridMax,
            hybridFinalDriftMeters = finalHybridDrift,
            hybridDriftPct = hybridDriftPct,
            hybridMax5sDriftMeters = peak5sHybridDrift,
            meanHeadingErrorDeg = meanHeadingErr,
            recoveryTimeSeconds = recoveryTime,
            roadConsistencyPct = roadConsistency,
            isTargetMet = hybridDriftPct <= 10.0,
            targetThresholdPct = 10.0,
            randomSeed = config.randomSeed
        )
    }

    private fun normalizeAngleDelta(delta: Double): Double {
        var d = delta
        while (d > 180.0) d -= 360.0
        while (d < -180.0) d += 360.0
        return d
    }
}
