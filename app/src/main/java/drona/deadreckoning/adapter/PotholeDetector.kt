package drona.deadreckoning.adapter

import android.util.Log
import kotlin.math.abs
import kotlin.math.roundToInt
import kotlin.math.sqrt

data class PotholeDetectionResult(
    val detected: Boolean,
    val severity: String,
    val confidence: Int
)

/**
 * A mount-aware, multi-sample impact detector. Its baseline adapts only while the
 * device is quiet, so a vehicle's normal vibration is not confused with an impact.
 */
class PotholeDetector {
    companion object {
        private const val TAG = "PotholeDetector"
        private const val GRAVITY = 9.81f
        private const val WINDOW_SIZE = 5
        private const val REQUIRED_IMPACT_SAMPLES = 3
    }

    private val impactWindow = ArrayDeque<Float>()
    private val rotationWindow = ArrayDeque<Float>()
    private var accelerationBaseline = 0f
    private var rotationBaseline = 0f
    private var baselineSamples = 0

    fun update(
        accelX: Float,
        accelY: Float,
        accelZ: Float,
        gyroX: Float,
        gyroY: Float,
        gyroZ: Float
    ): PotholeDetectionResult {
        val accelerationMagnitude = magnitude(accelX, accelY, accelZ)
        val rotationMagnitude = magnitude(gyroX, gyroY, gyroZ)
        val gravityDelta = abs(accelerationMagnitude - GRAVITY)

        updateBaseline(gravityDelta, rotationMagnitude)
        val impact = abs(gravityDelta - accelerationBaseline)
        val rotation = abs(rotationMagnitude - rotationBaseline)
        impactWindow.addLast(impact)
        rotationWindow.addLast(rotation)
        while (impactWindow.size > WINDOW_SIZE) impactWindow.removeFirst()
        while (rotationWindow.size > WINDOW_SIZE) rotationWindow.removeFirst()

        val impactThreshold = maxOf(3.5f, accelerationBaseline * 4f + 1.2f)
        val rotationThreshold = maxOf(0.8f, rotationBaseline * 4f + 0.35f)
        val impactSamples = impactWindow.count { it >= impactThreshold }
        val rotationSamples = rotationWindow.count { it >= rotationThreshold }
        val averageImpact = impactWindow.average().toFloat()
        val peakImpact = impactWindow.maxOrNull() ?: 0f
        val averageRotation = rotationWindow.average().toFloat()

        val detected = impactWindow.size >= REQUIRED_IMPACT_SAMPLES &&
            impactSamples >= REQUIRED_IMPACT_SAMPLES &&
            rotationSamples >= 2 &&
            averageImpact >= impactThreshold * 0.8f &&
            averageRotation >= rotationThreshold * 0.7f

        if (!detected) return PotholeDetectionResult(false, "None", 0)

        val severity = when {
            impactSamples >= 4 && peakImpact >= impactThreshold * 2.6f && averageImpact >= impactThreshold * 1.8f -> "Severe"
            peakImpact >= impactThreshold * 1.8f || averageImpact >= impactThreshold * 1.25f -> "Moderate"
            else -> "Minor"
        }
        val confidence = confidence(
            averageImpact / impactThreshold,
            peakImpact / impactThreshold,
            averageRotation / rotationThreshold,
            impactSamples
        )
        try {
            Log.i(TAG, "Pothole detected: $severity ($confidence%), impact=$peakImpact, samples=$impactSamples")
        } catch (_: RuntimeException) {
            // android.util.Log is unavailable in local JVM unit tests.
        }
        return PotholeDetectionResult(true, severity, confidence)
    }

    private fun updateBaseline(gravityDelta: Float, rotationMagnitude: Float) {
        if (gravityDelta > 1.2f || rotationMagnitude > 0.5f) return
        if (baselineSamples == 0) {
            accelerationBaseline = gravityDelta
            rotationBaseline = rotationMagnitude
        } else {
            accelerationBaseline += (gravityDelta - accelerationBaseline) * 0.04f
            rotationBaseline += (rotationMagnitude - rotationBaseline) * 0.04f
        }
        baselineSamples++
    }

    private fun confidence(impactRatio: Float, peakRatio: Float, rotationRatio: Float, samples: Int): Int {
        val strength = ((impactRatio - 0.8f) * 0.32f + (peakRatio - 1f) * 0.25f +
            (rotationRatio - 0.7f) * 0.07f + (samples - REQUIRED_IMPACT_SAMPLES) * 0.1f)
            .coerceIn(0f, 1f)
        return (58 + strength * 40).roundToInt()
    }

    private fun magnitude(x: Float, y: Float, z: Float): Float =
        sqrt(x * x + y * y + z * z)
}
