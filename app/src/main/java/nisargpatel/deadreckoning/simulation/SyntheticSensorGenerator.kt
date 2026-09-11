package nisargpatel.deadreckoning.simulation

import nisargpatel.deadreckoning.core.gnss.GnssFix
import nisargpatel.deadreckoning.ml.MotionClass
import org.osmdroid.util.GeoPoint
import java.util.Random
import kotlin.math.*

data class SyntheticImuStep(
    val forwardMeters: Double,
    val lateralMeters: Double,
    val headingDeltaRadians: Double,
    val intervalSeconds: Double,
    val rawGyroYawRateDps: Double
)

data class SyntheticAiSpeed(
    val speedMps: Double,
    val uncertaintyMps: Double,
    val motionClass: MotionClass,
    val sourceLabel: String = "SIMULATED MODEL MEASUREMENT"
)

data class SyntheticGnssObservation(
    val fix: GnssFix?,
    val position: GeoPoint?,
    val accuracyMeters: Double,
    val isBlackout: Boolean
)

class SyntheticSensorGenerator(
    private val config: SimulationConfig
) {
    private val random = Random(config.randomSeed)

    fun generateImuStep(gt: GroundTruthState, dt: Double): SyntheticImuStep {
        // Forward displacement with accelerometer / speed noise
        val speedNoise = random.nextGaussian() * config.aiSpeedNoiseStdMps
        val noisySpeed = (gt.speedMps + speedNoise).coerceAtLeast(0.0)
        val forwardMeters = noisySpeed * dt

        // Non-holonomic lateral slippage
        val lateralMeters = random.nextGaussian() * 0.04

        // Gyroscope angular rate with constant bias + white noise
        val gyroNoise = random.nextGaussian() * config.gyroNoiseStdDps
        val noisyYawRateDps = gt.yawRateDps + config.gyroBiasDps + gyroNoise
        val headingDeltaRad = Math.toRadians(noisyYawRateDps * dt)

        return SyntheticImuStep(
            forwardMeters = forwardMeters,
            lateralMeters = lateralMeters,
            headingDeltaRadians = headingDeltaRad,
            intervalSeconds = dt,
            rawGyroYawRateDps = noisyYawRateDps
        )
    }

    fun generateAiSpeed(gt: GroundTruthState): SyntheticAiSpeed {
        val speedNoise = random.nextGaussian() * config.aiSpeedNoiseStdMps
        val noisySpeed = (gt.speedMps + speedNoise).coerceAtLeast(0.0)
        val motionClass = when (gt.motionMode) {
            nisargpatel.deadreckoning.fusion.VehicleFusionImmUkf.MotionMode.CONSTANT_TURN_RATE -> MotionClass.TURNING
            nisargpatel.deadreckoning.fusion.VehicleFusionImmUkf.MotionMode.CONSTANT_ACCELERATION -> MotionClass.STRAIGHT
            nisargpatel.deadreckoning.fusion.VehicleFusionImmUkf.MotionMode.CONSTANT_VELOCITY -> MotionClass.STRAIGHT
        }

        return SyntheticAiSpeed(
            speedMps = noisySpeed,
            uncertaintyMps = config.aiSpeedNoiseStdMps,
            motionClass = motionClass,
            sourceLabel = "SIMULATED MODEL MEASUREMENT"
        )
    }

    fun generateGnss(gt: GroundTruthState, currentStepSec: Double): SyntheticGnssObservation {
        val inBlackout = gt.progressFraction >= config.blackoutStartPct &&
            gt.progressFraction <= config.blackoutEndPct

        if (inBlackout) {
            // Strictly starved of GNSS during blackout
            return SyntheticGnssObservation(
                fix = null,
                position = null,
                accuracyMeters = 999.0,
                isBlackout = true
            )
        }

        // Positional noise when GNSS is available
        val latScale = 111_132.95
        val lonScale = latScale * cos(Math.toRadians(gt.position.latitude))

        val dNorth = random.nextGaussian() * config.gnssNoiseStdMeters
        val dEast = random.nextGaussian() * config.gnssNoiseStdMeters

        val noisyLat = gt.position.latitude + (dNorth / latScale)
        val noisyLon = gt.position.longitude + (dEast / lonScale)
        val noisyPos = GeoPoint(noisyLat, noisyLon)

        val accuracy = config.gnssNoiseStdMeters * (1.0 + abs(random.nextGaussian() * 0.2)).coerceIn(1.2, 5.0)

        val fix = GnssFix(
            latitude = noisyLat,
            longitude = noisyLon,
            horizontalAccuracyMeters = accuracy,
            speedMps = gt.speedMps,
            bearingDegrees = gt.headingDegrees,
            ageMillis = 0L,
            isFromMockProvider = false
        )

        return SyntheticGnssObservation(
            fix = fix,
            position = noisyPos,
            accuracyMeters = accuracy,
            isBlackout = false
        )
    }
}
