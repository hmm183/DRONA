package drona.deadreckoning.ml

import android.content.Context
import android.util.Log
import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import com.google.gson.Gson
import drona.deadreckoning.core.spec.PreprocessingSpec
import java.nio.FloatBuffer
import java.util.ArrayDeque
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.sqrt

/**
 * One model output window.
 *
 * The uncertainty fields come from the network's own log-variance heads, which were
 * exported in the ONNX graph all along but never read. Previously the motion-class softmax
 * probability was reused as the confidence for speed, for position and for navigation as a
 * whole, which is meaningless: how sure the network is that the vehicle is turning says
 * nothing about how sure it is of the speed.
 *
 * @param motionConfidencePercentage confidence in the motion CLASS only. Not a navigation
 *   confidence and not a speed confidence.
 */
data class V8Prediction(
    val speedMps: Float,
    val forwardMeters: Float,
    val lateralMeters: Float,
    val headingDeltaRadians: Float,
    val motionClass: MotionClass,
    val motionConfidencePercentage: Int,
    val inferenceTimeMs: Long,
    val speedUncertaintyMps: Float = 0f,
    val forwardUncertaintyMeters: Float = 0f,
    val lateralUncertaintyMeters: Float = 0f,
    val headingUncertaintyRadians: Float = 0f
)

enum class MotionClass(val label: String) {
    STATIONARY("Stationary"),
    STRAIGHT("Driving straight"),
    TURNING("Turning");

    companion object {
        fun fromIndex(index: Int) = entries.getOrElse(index) { STRAIGHT }
    }
}

private data class V8Normalization(
    val imu_mean: List<Float>,
    val imu_std: List<Float>,
    val speed_mean: Float,
    val speed_std: Float,
    val position_mean: List<Float>,
    val position_std: List<Float>
)

/**
 * Runs the V8 model over a rolling IMU buffer.
 *
 * ## Why the stride matters
 *
 * V8's position output is the displacement across a whole window, not an instantaneous
 * velocity. Emitting overlapping windows and accumulating their displacements would
 * count the same motion many times over: a 1.9 s displacement arriving every 0.2 s would
 * inflate distance travelled by roughly ten times.
 *
 * So the stride is taken from the preprocessing contract rather than hardcoded, and
 * [PreprocessingSpec.LEGACY_V8] deliberately keeps a non-overlapping stride so the
 * existing baseline behaves exactly as before. Raising the output rate safely requires a
 * model that predicts velocity rather than window displacement, which is what
 * [PreprocessingSpec.IDR_V1] targets; the estimator already has
 * [drona.deadreckoning.fusion.VehicleFusionEkf.predictVelocity] waiting for it.
 */
class V8DeadReckoningEngine(
    context: Context,
    private val spec: PreprocessingSpec = PreprocessingSpec.LEGACY_V8
) : AutoCloseable {
    companion object {
        private const val TAG = "V8DeadReckoning"
        private const val CHANNEL_COUNT = 6
    }

    private val windowSize = spec.windowSamples
    private val strideSamples = spec.strideSamples

    /** Minimum spacing between accepted samples, derived from the contract's sample rate. */
    private val sampleIntervalNs = (1_000_000_000L / spec.sampleRateHz)

    private val environment = OrtEnvironment.getEnvironment()
    private val session: OrtSession
    private val normalization: V8Normalization
    val manifest: V8ModelManifest = ModelArtifactValidator.validate(context)
    private val window = ArrayDeque<FloatArray>(windowSize)
    private val rawWindow = ArrayDeque<FloatArray>(windowSize)
    private var lastAcceptedTimestampNs = 0L
    private var samplesSincePrediction = 0

    init {
        val model = context.assets.open("ml/v8_dead_reckoning.onnx").use { it.readBytes() }
        normalization = context.assets.open("ml/v8_normalization.json").reader().use {
            Gson().fromJson(it, V8Normalization::class.java)
        }
        session = environment.createSession(model, OrtSession.SessionOptions())
        Log.i(TAG, "Loaded ${manifest.model}; ${manifest.deployment_status}")
        Log.i(TAG, "Contract: ${spec.describe()}")
    }

    fun addSample(
        timestampNs: Long,
        accelX: Float,
        accelY: Float,
        accelZ: Float,
        gyroX: Float,
        gyroY: Float,
        gyroZ: Float,
        initialSpeedMps: Float
    ): V8Prediction? {
        if (lastAcceptedTimestampNs != 0L && timestampNs - lastAcceptedTimestampNs < sampleIntervalNs) return null
        lastAcceptedTimestampNs = timestampNs
        val raw = floatArrayOf(accelX, accelY, accelZ, gyroX, gyroY, gyroZ)
        val normalized = FloatArray(CHANNEL_COUNT) { index ->
            (raw[index] - normalization.imu_mean[index]) / normalization.imu_std[index]
        }
        if (window.size == windowSize) window.removeFirst()
        if (rawWindow.size == windowSize) rawWindow.removeFirst()
        window.addLast(normalized)
        rawWindow.addLast(raw)
        samplesSincePrediction++
        if (window.size < windowSize || samplesSincePrediction < strideSamples) return null
        samplesSincePrediction = 0
        return infer(initialSpeedMps)
    }

    private fun infer(initialSpeedMps: Float): V8Prediction {
        val startedAt = System.nanoTime()
        val imu = FloatArray(windowSize * CHANNEL_COUNT)
        window.forEachIndexed { sampleIndex, sample ->
            sample.copyInto(imu, sampleIndex * CHANNEL_COUNT)
        }
        val normalizedSpeed = (initialSpeedMps - normalization.speed_mean) / normalization.speed_std
        val imuTensor = OnnxTensor.createTensor(environment, FloatBuffer.wrap(imu), longArrayOf(1, windowSize.toLong(), CHANNEL_COUNT.toLong()))
        val speedTensor = OnnxTensor.createTensor(environment, FloatBuffer.wrap(floatArrayOf(normalizedSpeed)), longArrayOf(1))
        imuTensor.use { input ->
            speedTensor.use { state ->
                session.run(mapOf("imu" to input, "initial_speed_normalized" to state)).use { output ->
                    val speed = (output["speed"]!!.get().value as FloatArray)[0] * normalization.speed_std + normalization.speed_mean
                    val position = (output["position"]!!.get().value as Array<FloatArray>)[0]
                    val headingDelta = (output["heading_delta"]!!.get().value as FloatArray)[0]
                    val logits = (output["motion_logits"]!!.get().value as Array<FloatArray>)[0]
                    val probabilities = softmax(logits)
                    val motionIndex = probabilities.indices.maxBy { probabilities[it] }

                    // Log-variance heads. Speed and position were trained on normalised
                    // targets, so their sigmas must be scaled back by the same std.
                    // heading_delta was trained on raw radians, so its sigma is already real.
                    val speedSigma = sigmaFrom(output, "speed_log_variance") * normalization.speed_std
                    val positionSigma = sigmaPairFrom(output, "position_log_variance")
                    val headingSigma = sigmaFrom(output, "heading_delta_log_variance")

                    val prediction = V8Prediction(
                        speedMps = speed.coerceAtLeast(0f),
                        forwardMeters = position[0] * normalization.position_std[0] + normalization.position_mean[0],
                        lateralMeters = position[1] * normalization.position_std[1] + normalization.position_mean[1],
                        headingDeltaRadians = headingDelta,
                        motionClass = MotionClass.fromIndex(motionIndex),
                        motionConfidencePercentage = (probabilities[motionIndex] * 100).toInt().coerceIn(0, 100),
                        inferenceTimeMs = (System.nanoTime() - startedAt) / 1_000_000L,
                        speedUncertaintyMps = speedSigma,
                        forwardUncertaintyMeters = positionSigma.first * normalization.position_std[0],
                        lateralUncertaintyMeters = positionSigma.second * normalization.position_std[1],
                        headingUncertaintyRadians = headingSigma
                    )
                    if (isStationary()) {
                        return prediction.copy(
                            speedMps = 0f,
                            forwardMeters = 0f,
                            lateralMeters = 0f,
                            headingDeltaRadians = 0f,
                            motionClass = MotionClass.STATIONARY,
                            motionConfidencePercentage = 95,
                            speedUncertaintyMps = 0.1f,
                            forwardUncertaintyMeters = 0.2f,
                            lateralUncertaintyMeters = 0.2f,
                            headingUncertaintyRadians = 0.01f
                        )
                    }
                    Log.d(
                        TAG,
                        "Inference: ${prediction.motionClass.label}, speed=${prediction.speedMps * 3.6f} km/h " +
                            "+/-${prediction.speedUncertaintyMps * 3.6f}, motionConf=${prediction.motionConfidencePercentage}%, " +
                            "${prediction.inferenceTimeMs} ms"
                    )
                    return prediction
                }
            }
        }
    }

    /**
     * Standard deviation from a scalar log-variance head, in normalised units.
     *
     * Returns 0 when the head is absent so an older artifact still loads. Clamped because
     * an unbounded exp() on an untrained head can overflow to infinity and poison the filter.
     */
    private fun sigmaFrom(output: OrtSession.Result, name: String): Float {
        val value = runCatching { (output[name]!!.get().value as FloatArray)[0] }.getOrNull() ?: return 0f
        if (!value.isFinite()) return 0f
        return exp(0.5 * value.coerceIn(-20f, 10f).toDouble()).toFloat()
    }

    /** Standard deviations from a two-element log-variance head, in normalised units. */
    private fun sigmaPairFrom(output: OrtSession.Result, name: String): Pair<Float, Float> {
        val values = runCatching { (output[name]!!.get().value as Array<FloatArray>)[0] }.getOrNull()
            ?: return 0f to 0f
        fun convert(value: Float): Float {
            if (!value.isFinite()) return 0f
            return exp(0.5 * value.coerceIn(-20f, 10f).toDouble()).toFloat()
        }
        return convert(values.getOrElse(0) { 0f }) to convert(values.getOrElse(1) { 0f })
    }

    private fun softmax(logits: FloatArray): FloatArray {
        val highest = logits.maxOrNull() ?: 0f
        val unnormalized = FloatArray(logits.size) { exp((logits[it] - highest).toDouble()).toFloat() }
        val total = unnormalized.sum().coerceAtLeast(0.0001f)
        return FloatArray(logits.size) { unnormalized[it] / total }
    }

    private fun isStationary(): Boolean {
        val averageLinearAcceleration = rawWindow.map { sample ->
            abs(sqrt(sample[0] * sample[0] + sample[1] * sample[1] + sample[2] * sample[2]) - 9.81f)
        }.average()
        val averageRotation = rawWindow.map { sample ->
            sqrt(sample[3] * sample[3] + sample[4] * sample[4] + sample[5] * sample[5])
        }.average()
        return averageLinearAcceleration < 0.35 && averageRotation < 0.12
    }

    override fun close() = session.close()
}
