package nisargpatel.deadreckoning.ml

import android.content.Context
import com.google.gson.Gson
import java.security.MessageDigest

data class V8ModelManifest(
    val model: String,
    val input: V8ManifestInput,
    val outputs: List<String>,
    val checkpoint_epoch: Int,
    val sha256: String,
    val deployment_status: String
)

data class V8ManifestInput(val imu: List<Int>, val sample_rate_hz: Int, val initial_speed: String)

object ModelArtifactValidator {
    fun validate(context: Context): V8ModelManifest {
        val manifest = context.assets.open("ml/v8_manifest.json").reader().use { Gson().fromJson(it, V8ModelManifest::class.java) }
        require(manifest.input.imu == listOf(20, 6)) { "Unexpected V8 IMU shape" }
        require(manifest.input.sample_rate_hz == 10) { "Unexpected V8 sample rate" }
        require(setOf("speed", "position", "heading_delta", "motion_logits").all(manifest.outputs::contains)) { "V8 output contract mismatch" }
        val hash = context.assets.open("ml/v8_dead_reckoning.onnx").use { stream ->
            MessageDigest.getInstance("SHA-256").digest(stream.readBytes()).joinToString("") { "%02x".format(it) }
        }
        require(hash.equals(manifest.sha256, ignoreCase = true)) { "V8 model hash mismatch" }
        return manifest
    }

    fun validateV9(context: Context): V9Manifest {
        val manifest = context.assets.open("ml/v9_manifest.json").reader().use { Gson().fromJson(it, V9Manifest::class.java) }
        require(manifest.checkpoint_interval_steps == 50) { "Unexpected V9 checkpoint interval" }
        val hash = context.assets.open("ml/v9_adaptive_projection.onnx").use { stream ->
            MessageDigest.getInstance("SHA-256").digest(stream.readBytes()).joinToString("") { "%02x".format(it) }
        }
        require(hash.equals(manifest.sha256, ignoreCase = true)) { "V9 model hash mismatch" }
        return manifest
    }
}
