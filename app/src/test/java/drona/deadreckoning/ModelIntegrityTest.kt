package drona.deadreckoning

import com.google.gson.Gson
import com.google.gson.JsonObject
import java.io.File
import java.security.MessageDigest
import drona.deadreckoning.support.locateAsset
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Validates the cryptographic integrity of all packaged on-device ONNX neural models
 * against their deployment manifests.
 *
 * Ensures models cannot be tampered with, corrupted during packaging/asset compression,
 * or silently downgraded in production.
 */
class ModelIntegrityTest {

    private fun sha256(bytes: ByteArray): String {
        val digest = MessageDigest.getInstance("SHA-256")
        return digest.digest(bytes).joinToString("") { "%02x".format(it) }
    }

    @Test
    fun `pino v3 onnx model matches manifest sha256`() {
        val modelFile = locateAsset("src/main/assets/ml/v3_pino_dr.onnx")
        val manifestFile = locateAsset("src/main/assets/ml/v3_pino_manifest.json")

        val modelBytes = modelFile.readBytes()
        val computedHash = sha256(modelBytes)

        val json = Gson().fromJson(manifestFile.readText(), JsonObject::class.java)
        val manifestHash = json.get("sha256")?.asString

        assertTrue("Manifest must have sha256 field", !manifestHash.isNullOrBlank())
        assertEquals(
            "PINO-DR v3 ONNX model SHA-256 does not match manifest!",
            manifestHash!!.lowercase(),
            computedHash.lowercase()
        )
    }

    @Test
    fun `idr v1 onnx model matches manifest sha256`() {
        val modelFile = locateAsset("src/main/assets/ml/idr_v1.onnx")
        val manifestFile = locateAsset("src/main/assets/ml/idr_v1_manifest.json")

        val modelBytes = modelFile.readBytes()
        val computedHash = sha256(modelBytes)

        val json = Gson().fromJson(manifestFile.readText(), JsonObject::class.java)
        val manifestHash = json.get("sha256")?.asString

        assertTrue("Manifest must have sha256 field", !manifestHash.isNullOrBlank())
        assertEquals(
            "IDR-V1 ONNX model SHA-256 does not match manifest!",
            manifestHash!!.lowercase(),
            computedHash.lowercase()
        )
    }

    @Test
    fun `v8 onnx model integrity verified`() {
        val modelFile = locateAsset("src/main/assets/ml/v8_dead_reckoning.onnx")
        val modelBytes = modelFile.readBytes()
        val computedHash = sha256(modelBytes)

        val expectedHash = "f9dbbf12fd9c28acd8c5cb4f951c51afb2485bdcc27ee2278a293733ed246908"
        assertEquals(
            "V8 dead reckoning ONNX model SHA-256 mismatch",
            expectedHash.lowercase(),
            computedHash.lowercase()
        )
    }

    @Test
    fun `tampered model detection fails validation`() {
        val modelFile = locateAsset("src/main/assets/ml/idr_v1.onnx")
        val tamperedBytes = modelFile.readBytes().apply {
            this[0] = (this[0] + 1).toByte()
        }
        val computedHash = sha256(tamperedBytes)

        val manifestFile = locateAsset("src/main/assets/ml/idr_v1_manifest.json")
        val json = Gson().fromJson(manifestFile.readText(), JsonObject::class.java)
        val manifestHash = json.get("sha256")?.asString

        assertNotEquals(manifestHash?.lowercase(), computedHash.lowercase())
    }

    @Test
    fun `v9 adaptive projection onnx model matches manifest sha256`() {
        val modelFile = locateAsset("src/main/assets/ml/v9_adaptive_projection.onnx")
        val manifestFile = locateAsset("src/main/assets/ml/v9_manifest.json")

        val modelBytes = modelFile.readBytes()
        val computedHash = sha256(modelBytes)

        val json = Gson().fromJson(manifestFile.readText(), JsonObject::class.java)
        val manifestHash = json.get("sha256")?.asString

        assertTrue("Manifest must have sha256 field", !manifestHash.isNullOrBlank())
        assertEquals(
            "V9 Adaptive Projection ONNX model SHA-256 does not match manifest!",
            manifestHash!!.lowercase(),
            computedHash.lowercase()
        )
    }

    @Test
    fun `v9 adaptive projection onnx model runs JVM inference cleanly`() {
        val modelFile = locateAsset("src/main/assets/ml/v9_adaptive_projection.onnx")
        val env = ai.onnxruntime.OrtEnvironment.getEnvironment()
        val session = env.createSession(modelFile.absolutePath, ai.onnxruntime.OrtSession.SessionOptions())

        // 50 timesteps x 8 channels
        val seqData = FloatArray(1 * 50 * 8)
        val ctxData = FloatArray(1 * 12)
        ctxData[0] = 0.5f // discrepancy_m_scaled
        ctxData[4 + 7] = 1.0f // CRUISE event

        val seqTensor = ai.onnxruntime.OnnxTensor.createTensor(
            env,
            java.nio.FloatBuffer.wrap(seqData),
            longArrayOf(1, 50, 8)
        )
        val ctxTensor = ai.onnxruntime.OnnxTensor.createTensor(
            env,
            java.nio.FloatBuffer.wrap(ctxData),
            longArrayOf(1, 12)
        )

        val inputs = mapOf("x_seq" to seqTensor, "x_ctx" to ctxTensor)
        val result = session.run(inputs)

        @Suppress("UNCHECKED_CAST")
        val deltaX = (result[0].value as Array<FloatArray>)[0]
        @Suppress("UNCHECKED_CAST")
        val conf = (result[1].value as Array<FloatArray>)[0][0]

        assertEquals("V9 must output 6 error states", 6, deltaX.size)
        assertTrue("Confidence must be in [0, 1]", conf in 0.0f..1.0f)

        seqTensor.close()
        ctxTensor.close()
        result.close()
        session.close()
    }
}
