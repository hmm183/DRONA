package drona.deadreckoning.ml

import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import android.content.Context
import android.util.Log
import com.google.gson.Gson
import java.nio.FloatBuffer
import java.security.MessageDigest
import java.util.ArrayDeque
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt
import drona.deadreckoning.core.spec.PreprocessingSpec

/**
 * Driving events classified for physics-aware adaptive threshold gating.
 */
enum class V9DrivingEvent(val id: Int, val defaultThresholdMeters: Float) {
    STOP(0, 2.0f),
    HIGH_RATTLE(1, 12.0f),
    ROUNDABOUT_CANDIDATE(2, 10.0f),
    TURN(3, 8.0f),
    ACCEL(4, 6.0f),
    BRAKE(5, 6.0f),
    STRAIGHT(6, 5.0f),
    CRUISE(7, 5.0f)
}

/**
 * One V9 Adaptive Projection checkpoint prediction.
 */
data class V9Prediction(
    val errorStateCorrection: FloatArray, // 6D: [delta_E, delta_N, delta_v, delta_psi, delta_ba, delta_bg]
    val confidence: Float,
    val drivingEvent: V9DrivingEvent,
    val discrepancyMeters: Float,
    val thresholdMeters: Float,
    val projectionTriggered: Boolean,
    val inferenceTimeMs: Long,
    val referenceSpeedMps: Float,
    val correctedSpeedMps: Float,
    val correctedHeadingDeltaRad: Float
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is V9Prediction) return false
        return errorStateCorrection.contentEquals(other.errorStateCorrection) &&
            confidence == other.confidence &&
            drivingEvent == other.drivingEvent &&
            discrepancyMeters == other.discrepancyMeters &&
            thresholdMeters == other.thresholdMeters &&
            projectionTriggered == other.projectionTriggered
    }

    override fun hashCode(): Int {
        var result = errorStateCorrection.contentHashCode()
        result = 31 * result + confidence.hashCode()
        result = 31 * result + drivingEvent.hashCode()
        result = 31 * result + discrepancyMeters.hashCode()
        result = 31 * result + thresholdMeters.hashCode()
        result = 31 * result + projectionTriggered.hashCode()
        return result
    }
}

/** Deployment manifest for the packaged V9 ONNX model artifact. */
data class V9Manifest(
    val model: String = "v9 Adaptive Projection Dead Reckoning",
    val architecture: String = "DepthwiseSeparableConv1D + GRU + Context MLP",
    val deployment_status: String = "v9 Adaptive Projection DR Production",
    val preprocessing_version: String = "v9-projection",
    val parameters: Int = 16895,
    val sha256: String = "",
    val checkpoint_interval_steps: Int = 50,
    val checkpoint_interval_seconds: Double = 5.0,
    val min_confidence_gate: Float = 0.20f,
    val adaptive_thresholds_m: Map<String, Float> = emptyMap()
)

/** Normalization and scaling metadata for the V9 model artifact. */
data class V9Normalization(
    val preprocessing_version: String = "v9-projection",
    val discrepancy_m_scale: Float = 10.0f,
    val discrepancy_speed_scale: Float = 5.0f,
    val discrepancy_yaw_scale: Float = 1.0f,
    val current_speed_scale: Float = 30.0f
)

/**
 * On-device V9 Adaptive Projection Dead Reckoning Engine.
 *
 * Implements:
 * 1. 10 Hz uniform decimation & 8-feature temporal context buffer (50 steps = 5.0s).
 * 2. Event Classifier: STOP, RATTLE, ROUNDABOUT, TURN, ACCEL, BRAKE, STRAIGHT, CRUISE.
 * 3. Physical Reference Synthesizer (Kinematic integration + NHC + Centripetal yaw constraint).
 * 4. 5-Second Periodic Adaptive Trajectory Projection (PATP) Gating.
 * 5. On-Device ONNX Runtime inference for `AdaptiveStateProjectionNet`.
 * 6. Soft Error-State Extended Kalman Filter (ES-EKF) innovation computation.
 */
class V9AdaptiveProjectionEngine internal constructor(
    modelBytes: ByteArray,
    val manifest: V9Manifest,
    val normalization: V9Normalization = V9Normalization(),
    private val spec: PreprocessingSpec = PreprocessingSpec.V9_PROJECTION
) : AutoCloseable {

    constructor(
        context: Context,
        spec: PreprocessingSpec = PreprocessingSpec.V9_PROJECTION
    ) : this(
        modelBytes = context.assets.open(MODEL_ASSET).use { it.readBytes() },
        manifest = ModelArtifactValidator.validateV9(context),
        normalization = loadNormalization(context),
        spec = spec
    )

    companion object {
        private const val TAG = "V9ProjectionEngine"
        private const val MODEL_ASSET = "ml/v9_adaptive_projection.onnx"
        private const val NORMALIZATION_ASSET = "ml/v9_normalization.json"
        private const val CHANNEL_COUNT = 8
        private const val WINDOW_SIZE = 50 // 5.0 s at 10 Hz

        internal fun loadNormalization(context: Context): V9Normalization {
            return try {
                context.assets.open(NORMALIZATION_ASSET).reader().use {
                    Gson().fromJson(it, V9Normalization::class.java)
                } ?: V9Normalization()
            } catch (_: Throwable) {
                V9Normalization()
            }
        }
    }

    private val environment = OrtEnvironment.getEnvironment()
    private val session: OrtSession

    // Rolling 50-step sequence buffer: 8 features per step
    private val seqBuffer = ArrayDeque<FloatArray>(WINDOW_SIZE)

    // Window kinematic tracking for reference synthesis
    private var windowStartSpeedMps = 0.0f
    private var currentEstimatedSpeedMps = 0.0f
    private var currentEstimatedYawRad = 0.0f
    private var stepCountSinceCheckpoint = 0
    private var prevAFwd = 0.0f

    // Kinematic reference accumulation
    private var refPosE = 0.0
    private var refPosN = 0.0
    private var refSpeed = 0.0

    // Nominal DR accumulation over the window
    private var drPosE = 0.0
    private var drPosN = 0.0

    // Turn tracking for roundabout candidate detection
    private var sustainedTurnSteps = 0

    // Last evaluated driving event
    var currentDrivingEvent: V9DrivingEvent = V9DrivingEvent.CRUISE
        private set

    var totalProjectionsTriggered: Int = 0
        private set

    init {
        val sessionOptions = OrtSession.SessionOptions().apply {
            setIntraOpNumThreads(2)
        }
        session = environment.createSession(modelBytes, sessionOptions)
        try {
            Log.i(TAG, "Loaded ${manifest.model} (${manifest.parameters} params, opset 18)")
        } catch (_: Throwable) {
            // Log unavailable in local JVM tests
        }
    }

    fun reset(initialSpeedMps: Float = 0.0f, initialYawRad: Float = 0.0f) {
        seqBuffer.clear()
        windowStartSpeedMps = initialSpeedMps
        currentEstimatedSpeedMps = initialSpeedMps
        currentEstimatedYawRad = initialYawRad
        stepCountSinceCheckpoint = 0
        prevAFwd = 0.0f
        refPosE = 0.0
        refPosN = 0.0
        refSpeed = initialSpeedMps.toDouble()
        drPosE = 0.0
        drPosN = 0.0
        sustainedTurnSteps = 0
        currentDrivingEvent = V9DrivingEvent.CRUISE
        totalProjectionsTriggered = 0
    }

    /**
     * Ingests one 10 Hz IMU step, updates kinematic reference and DR trajectory,
     * and evaluates the 5-second checkpoint when 50 steps accumulate.
     *
     * @param aFwd Forward vehicle acceleration (m/s^2)
     * @param aLat Lateral vehicle acceleration (m/s^2)
     * @param wYaw Yaw rate about vehicle Down axis (rad/s)
     * @param stepDisplacementMeters Forward displacement step from motion backbone (m)
     * @param stepHeadingDeltaRad Heading delta step from motion backbone (rad)
     * @param isStationary ZUPT flag from hardware or motion model
     * @return V9Prediction if checkpoint evaluated, or null during intermediate steps
     */
    fun addSample(
        aFwd: Float,
        aLat: Float,
        wYaw: Float,
        stepDisplacementMeters: Float,
        stepHeadingDeltaRad: Float,
        isStationary: Boolean
    ): V9Prediction? {
        val dt = 0.1f // 10 Hz

        // Update estimated state
        currentEstimatedYawRad += stepHeadingDeltaRad
        val effectiveDisp = if (isStationary) 0.0f else max(0.0f, stepDisplacementMeters)
        currentEstimatedSpeedMps = effectiveDisp / dt

        // Nominal DR integration over the window
        drPosE += effectiveDisp * sin(currentEstimatedYawRad)
        drPosN += effectiveDisp * cos(currentEstimatedYawRad)

        // Classify driving event
        val event = classifyEvent(aFwd, aLat, wYaw, currentEstimatedSpeedMps, isStationary)
        currentDrivingEvent = event

        // Synthesize physical kinematic reference
        if (isStationary) {
            refSpeed = 0.0
        } else {
            val aCent = currentEstimatedSpeedMps * wYaw
            val aCentResidual = aLat - aCent
            val linAccel = aFwd.toDouble().coerceIn(-8.0, 8.0)
            refSpeed = max(0.0, refSpeed + linAccel * dt)
            // Centripetal velocity bound
            if (abs(wYaw) > 0.05f) {
                val vCent = sqrt((abs(aLat) / abs(wYaw)).toDouble()).coerceIn(0.0, 45.0)
                refSpeed = 0.85 * refSpeed + 0.15 * vCent
            }
        }
        refPosE += refSpeed * dt * sin(currentEstimatedYawRad)
        refPosN += refSpeed * dt * cos(currentEstimatedYawRad)

        // Compute 8 temporal features:
        // [a_fwd, a_lat, w_yaw, speed_norm, a_cent, cent_res, delta_psi, jerk]
        val speedNorm = (currentEstimatedSpeedMps / normalization.current_speed_scale).coerceIn(0.0f, 2.0f)
        val aCent = currentEstimatedSpeedMps * wYaw
        val centRes = aLat - aCent
        val jerk = ((aFwd - prevAFwd) / dt).coerceIn(-50.0f, 50.0f)
        prevAFwd = aFwd
        val feat8 = floatArrayOf(
            aFwd.coerceIn(-8.0f, 8.0f),
            aLat.coerceIn(-8.0f, 8.0f),
            wYaw.coerceIn(-1.0f, 1.0f),
            speedNorm,
            aCent.coerceIn(-8.0f, 8.0f),
            centRes.coerceIn(-8.0f, 8.0f),
            stepHeadingDeltaRad.coerceIn(-0.5f, 0.5f),
            jerk
        )

        if (seqBuffer.size >= WINDOW_SIZE) {
            seqBuffer.removeFirst()
        }
        seqBuffer.addLast(feat8)
        stepCountSinceCheckpoint++

        // Checkpoint evaluated every 50 steps (5.0 seconds)
        if (stepCountSinceCheckpoint < WINDOW_SIZE) {
            return null
        }

        // --- 5-SECOND CHECKPOINT EVALUATION ---
        stepCountSinceCheckpoint = 0

        val dE = refPosE - drPosE
        val dN = refPosN - drPosN
        val discrepancyM = sqrt(dE * dE + dN * dN).toFloat()
        val discrepancySpeed = abs((refSpeed - currentEstimatedSpeedMps).toFloat())
        val discrepancyYaw = abs(stepHeadingDeltaRad)

        val threshold = manifest.adaptive_thresholds_m[event.name]
            ?: event.defaultThresholdMeters

        // Projection gate: discrepancy exceeds adaptive threshold and window is filled
        if (discrepancyM < threshold || seqBuffer.size < WINDOW_SIZE) {
            // Discrepancy is acceptable; reset window integration origins
            drPosE = 0.0
            drPosN = 0.0
            refPosE = 0.0
            refPosN = 0.0
            windowStartSpeedMps = currentEstimatedSpeedMps
            return V9Prediction(
                errorStateCorrection = FloatArray(6),
                confidence = 0.0f,
                drivingEvent = event,
                discrepancyMeters = discrepancyM,
                thresholdMeters = threshold,
                projectionTriggered = false,
                inferenceTimeMs = 0L,
                referenceSpeedMps = refSpeed.toFloat(),
                correctedSpeedMps = currentEstimatedSpeedMps,
                correctedHeadingDeltaRad = 0.0f
            )
        }

        // Assemble 50x8 temporal sequence
        val seqFlat = FloatArray(WINDOW_SIZE * CHANNEL_COUNT)
        var offset = 0
        for (stepFeat in seqBuffer) {
            System.arraycopy(stepFeat, 0, seqFlat, offset, CHANNEL_COUNT)
            offset += CHANNEL_COUNT
        }

        // Assemble 12D context vector:
        // [d_p_scaled, d_v_scaled, d_yaw_scaled, speed_curr_scaled, 8-dim event onehot]
        val ctxFlat = FloatArray(12)
        ctxFlat[0] = (discrepancyM / normalization.discrepancy_m_scale).coerceIn(0.0f, 5.0f)
        ctxFlat[1] = (discrepancySpeed / normalization.discrepancy_speed_scale).coerceIn(0.0f, 5.0f)
        ctxFlat[2] = (discrepancyYaw / normalization.discrepancy_yaw_scale).coerceIn(0.0f, 5.0f)
        ctxFlat[3] = (currentEstimatedSpeedMps / normalization.current_speed_scale).coerceIn(0.0f, 2.0f)
        ctxFlat[4 + event.id] = 1.0f

        val startTime = System.nanoTime()
        var seqTensor: OnnxTensor? = null
        var ctxTensor: OnnxTensor? = null

        var deltaX = FloatArray(6)
        var confidence = 0.0f

        try {
            seqTensor = OnnxTensor.createTensor(
                environment,
                FloatBuffer.wrap(seqFlat),
                longArrayOf(1, WINDOW_SIZE.toLong(), CHANNEL_COUNT.toLong())
            )
            ctxTensor = OnnxTensor.createTensor(
                environment,
                FloatBuffer.wrap(ctxFlat),
                longArrayOf(1, 12L)
            )
            val inputs = mapOf("x_seq" to seqTensor, "x_ctx" to ctxTensor)
            session.run(inputs).use { results ->
                @Suppress("UNCHECKED_CAST")
                val deltaXRaw = results[0].value as Array<FloatArray>
                deltaX = deltaXRaw[0].copyOf()

                @Suppress("UNCHECKED_CAST")
                val confRaw = results[1].value as Array<FloatArray>
                confidence = confRaw[0][0]
            }
        } catch (e: Exception) {
            Log.e(TAG, "ONNX inference error in V9 model", e)
        } finally {
            seqTensor?.close()
            ctxTensor?.close()
        }
        val inferenceMs = (System.nanoTime() - startTime) / 1_000_000L

        // Gate by minimum confidence
        val shouldTrigger = confidence >= manifest.min_confidence_gate
        if (shouldTrigger) {
            totalProjectionsTriggered++
        }

        // Reset window origins for next 50-step period
        drPosE = 0.0
        drPosN = 0.0
        refPosE = 0.0
        refPosN = 0.0
        windowStartSpeedMps = currentEstimatedSpeedMps

        val correctedSpeed = (currentEstimatedSpeedMps + deltaX[2] * confidence).coerceAtLeast(0.0f)
        val correctedHeadingDelta = deltaX[3] * confidence

        return V9Prediction(
            errorStateCorrection = deltaX,
            confidence = confidence,
            drivingEvent = event,
            discrepancyMeters = discrepancyM,
            thresholdMeters = threshold,
            projectionTriggered = shouldTrigger,
            inferenceTimeMs = inferenceMs,
            referenceSpeedMps = refSpeed.toFloat(),
            correctedSpeedMps = correctedSpeed,
            correctedHeadingDeltaRad = correctedHeadingDelta
        )
    }

    internal fun classifyEvent(
        aFwd: Float,
        aLat: Float,
        wYaw: Float,
        speedMps: Float,
        isStationary: Boolean
    ): V9DrivingEvent {
        if (isStationary || speedMps < 0.3f) {
            sustainedTurnSteps = 0
            return V9DrivingEvent.STOP
        }
        if (abs(aFwd) > 6.0f || abs(aLat) > 6.0f) {
            return V9DrivingEvent.HIGH_RATTLE
        }
        if (abs(wYaw) > 0.15f) {
            sustainedTurnSteps++
            if (sustainedTurnSteps > 25) { // 2.5s sustained turning
                return V9DrivingEvent.ROUNDABOUT_CANDIDATE
            }
            return V9DrivingEvent.TURN
        } else {
            sustainedTurnSteps = max(0, sustainedTurnSteps - 1)
        }
        if (abs(wYaw) > 0.08f) {
            return V9DrivingEvent.TURN
        }
        if (aFwd > 0.8f) {
            return V9DrivingEvent.ACCEL
        }
        if (aFwd < -0.8f) {
            return V9DrivingEvent.BRAKE
        }
        if (speedMps >= 15.0f) {
            return V9DrivingEvent.CRUISE
        }
        return V9DrivingEvent.STRAIGHT
    }

    override fun close() {
        try {
            session.close()
        } catch (_: Throwable) {
            // OrtSession close error
        }
    }
}
