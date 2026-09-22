package drona.deadreckoning.fusion

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

data class VehicleAlignment(
    val yawOffsetDegrees: Double = 0.0,
    val pitchDegrees: Double = 0.0,
    val rollDegrees: Double = 0.0,
    val confidencePercentage: Int = 0,
    val sampleCount: Int = 0
)

/** Learns the phone-to-vehicle yaw offset from reliable GNSS course while moving. */
class VehicleAlignmentCalibrator {
    private var sumSin = 0.0
    private var sumCos = 0.0
    private var sampleCount = 0
    private var pitch = 0.0
    private var roll = 0.0

    fun addObservation(
        phoneYawDegrees: Float,
        phonePitchDegrees: Float,
        phoneRollDegrees: Float,
        gnssBearingDegrees: Double,
        speedKmh: Double,
        accuracyMeters: Double
    ): VehicleAlignment {
        if (speedKmh < 8.0 || accuracyMeters > 18.0) return alignment()
        val yawOffset = Math.toRadians(gnssBearingDegrees - phoneYawDegrees)
        sumSin += sin(yawOffset)
        sumCos += cos(yawOffset)
        sampleCount++
        pitch += (phonePitchDegrees - pitch) * 0.08
        roll += (phoneRollDegrees - roll) * 0.08
        return alignment()
    }

    fun adjustedHeading(phoneYawDegrees: Float): Double =
        (phoneYawDegrees + alignment().yawOffsetDegrees + 360.0) % 360.0

    fun restore(saved: VehicleAlignment) {
        if (saved.sampleCount <= 0) return
        val radians = Math.toRadians(saved.yawOffsetDegrees)
        sampleCount = saved.sampleCount
        sumSin = sin(radians) * sampleCount
        sumCos = cos(radians) * sampleCount
        pitch = saved.pitchDegrees
        roll = saved.rollDegrees
    }

    fun alignment(): VehicleAlignment {
        val offset = if (sampleCount == 0) 0.0 else Math.toDegrees(atan2(sumSin, sumCos))
        val resultant = if (sampleCount == 0) 0.0 else kotlin.math.sqrt(sumSin * sumSin + sumCos * sumCos) / sampleCount
        return VehicleAlignment(
            yawOffsetDegrees = offset,
            pitchDegrees = pitch,
            rollDegrees = roll,
            confidencePercentage = (resultant * 100.0 * (sampleCount / 12.0).coerceAtMost(1.0)).toInt().coerceIn(0, 100),
            sampleCount = sampleCount
        )
    }
}
