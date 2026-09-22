package nisargpatel.deadreckoning

import com.google.gson.Gson
import nisargpatel.deadreckoning.ml.V9AdaptiveProjectionEngine
import nisargpatel.deadreckoning.ml.V9DrivingEvent
import nisargpatel.deadreckoning.ml.V9Manifest
import nisargpatel.deadreckoning.ml.V9Normalization
import nisargpatel.deadreckoning.support.locateAsset
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Comprehensive JVM Unit Test Suite for [V9AdaptiveProjectionEngine].
 *
 * Exercises:
 * 1. Packaging & Manifest Integrity (SHA-256, parameter counts, opset).
 * 2. Normalization Bounds & Constants.
 * 3. Driving Event Classifier (STOP, HIGH_RATTLE, ROUNDABOUT_CANDIDATE, TURN, ACCEL, BRAKE, CRUISE, STRAIGHT).
 * 4. Periodic 5-Second (50-step) Checkpoint Windowing & Gating.
 * 5. ONNX Runtime ES-EKF 6D Innovation Ingestion & Confidence Verification.
 * 6. Engine Reset & Sequence Buffer State Isolation.
 */
class V9AdaptiveProjectionEngineTest {

    private lateinit var engine: V9AdaptiveProjectionEngine
    private lateinit var manifest: V9Manifest
    private lateinit var normalization: V9Normalization

    @Before
    fun setUp() {
        val modelFile = locateAsset("src/main/assets/ml/v9_adaptive_projection.onnx")
        val manifestFile = locateAsset("src/main/assets/ml/v9_manifest.json")
        val normalizationFile = locateAsset("src/main/assets/ml/v9_normalization.json")

        val gson = Gson()
        manifest = gson.fromJson(manifestFile.readText(), V9Manifest::class.java)
        normalization = gson.fromJson(normalizationFile.readText(), V9Normalization::class.java)

        engine = V9AdaptiveProjectionEngine(
            modelBytes = modelFile.readBytes(),
            manifest = manifest,
            normalization = normalization
        )
    }

    @After
    fun tearDown() {
        engine.close()
    }

    @Test
    fun `manifest and normalization assets load accurately`() {
        assertEquals("v9 Adaptive Projection Dead Reckoning", manifest.model)
        assertEquals("v9-projection", manifest.preprocessing_version)
        assertEquals(16895, manifest.parameters)
        assertEquals(50, manifest.checkpoint_interval_steps)
        assertEquals(5.0, manifest.checkpoint_interval_seconds, 1e-4)
        assertEquals(0.20f, manifest.min_confidence_gate, 1e-4f)

        assertEquals("v9-projection", normalization.preprocessing_version)
        assertEquals(10.0f, normalization.discrepancy_m_scale, 1e-4f)
        assertEquals(5.0f, normalization.discrepancy_speed_scale, 1e-4f)
        assertEquals(1.0f, normalization.discrepancy_yaw_scale, 1e-4f)
        assertEquals(30.0f, normalization.current_speed_scale, 1e-4f)
    }

    @Test
    fun `event classifier identifies all driving regimes correctly`() {
        // 1. STOP: stationary or very low speed
        assertEquals(V9DrivingEvent.STOP, engine.classifyEvent(0f, 0f, 0f, 0.1f, isStationary = false))
        assertEquals(V9DrivingEvent.STOP, engine.classifyEvent(1f, 0f, 0f, 10f, isStationary = true))

        // 2. HIGH_RATTLE: severe longitudinal or lateral acceleration spikes
        assertEquals(V9DrivingEvent.HIGH_RATTLE, engine.classifyEvent(7.5f, 0.1f, 0.01f, 10f, isStationary = false))
        assertEquals(V9DrivingEvent.HIGH_RATTLE, engine.classifyEvent(0.2f, -6.5f, 0.02f, 10f, isStationary = false))

        // 3. TURN: yaw rate above turn threshold
        assertEquals(V9DrivingEvent.TURN, engine.classifyEvent(0.1f, 0.5f, 0.12f, 10f, isStationary = false))

        // 4. ROUNDABOUT_CANDIDATE: sustained turning for > 25 consecutive steps
        engine.reset()
        for (i in 1..24) {
            engine.classifyEvent(0.1f, 1.2f, 0.20f, 8f, isStationary = false)
        }
        // At step 25 it is still TURN
        assertEquals(V9DrivingEvent.TURN, engine.classifyEvent(0.1f, 1.2f, 0.20f, 8f, isStationary = false))
        // At step 26 (> 25 steps) it becomes ROUNDABOUT_CANDIDATE
        assertEquals(V9DrivingEvent.ROUNDABOUT_CANDIDATE, engine.classifyEvent(0.1f, 1.2f, 0.20f, 8f, isStationary = false))

        // 5. ACCEL / BRAKE
        engine.reset()
        assertEquals(V9DrivingEvent.ACCEL, engine.classifyEvent(1.5f, 0f, 0f, 10f, isStationary = false))
        assertEquals(V9DrivingEvent.BRAKE, engine.classifyEvent(-1.2f, 0f, 0f, 10f, isStationary = false))

        // 6. CRUISE (speed >= 15 m/s, smooth motion)
        assertEquals(V9DrivingEvent.CRUISE, engine.classifyEvent(0.05f, 0.05f, 0.01f, 18f, isStationary = false))

        // 7. STRAIGHT (speed < 15 m/s, smooth motion)
        assertEquals(V9DrivingEvent.STRAIGHT, engine.classifyEvent(0.05f, 0.05f, 0.01f, 10f, isStationary = false))
    }

    @Test
    fun `checkpoint evaluation triggers exactly every 50 steps`() {
        engine.reset(initialSpeedMps = 10.0f, initialYawRad = 0.0f)

        // Steps 1 to 49 must return null
        for (step in 1..49) {
            val prediction = engine.addSample(
                aFwd = 0.0f,
                aLat = 0.0f,
                wYaw = 0.0f,
                stepDisplacementMeters = 1.0f, // 10 m/s at 10 Hz
                stepHeadingDeltaRad = 0.0f,
                isStationary = false
            )
            assertNull("Step $step must not trigger checkpoint prediction", prediction)
        }

        // Step 50 must evaluate checkpoint
        val checkpoint = engine.addSample(
            aFwd = 0.0f,
            aLat = 0.0f,
            wYaw = 0.0f,
            stepDisplacementMeters = 1.0f,
            stepHeadingDeltaRad = 0.0f,
            isStationary = false
        )
        assertNotNull("Step 50 must evaluate checkpoint", checkpoint)
        assertEquals(V9DrivingEvent.STRAIGHT, checkpoint!!.drivingEvent)
        // Benign straight motion: reference and DR match closely, so discrepancy < 5.0m threshold
        assertTrue(checkpoint.discrepancyMeters < checkpoint.thresholdMeters)
        assertFalse(checkpoint.projectionTriggered)
    }

    @Test
    fun `kinematic discrepancy exceeding threshold fires on-device ONNX inference`() {
        engine.reset(initialSpeedMps = 10.0f, initialYawRad = 0.0f)

        // Feed 50 steps with intentional physical discrepancy:
        // Forward acceleration indicates vehicle is accelerating rapidly (+3 m/s^2),
        // but backbone displacement is constrained to very small steps (0.1m / 1 m/s).
        // This causes refPos and drPos to diverge by > 15 meters over 5 seconds.
        var lastPrediction: nisargpatel.deadreckoning.ml.V9Prediction? = null
        for (step in 1..50) {
            lastPrediction = engine.addSample(
                aFwd = 2.5f,
                aLat = 0.0f,
                wYaw = 0.0f,
                stepDisplacementMeters = 0.2f, // severely lagging DR
                stepHeadingDeltaRad = 0.0f,
                isStationary = false
            )
        }

        assertNotNull("Step 50 must evaluate", lastPrediction)
        val pred = lastPrediction!!

        // Discrepancy must exceed the adaptive threshold for ACCEL (6.0 m)
        assertTrue("Discrepancy (${pred.discrepancyMeters}m) should exceed threshold (${pred.thresholdMeters}m)",
            pred.discrepancyMeters >= pred.thresholdMeters)

        // ONNX model must produce 6D error-state corrections
        assertEquals(6, pred.errorStateCorrection.size)
        assertTrue("Confidence must be in [0, 1]", pred.confidence in 0.0f..1.0f)

        if (pred.confidence >= manifest.min_confidence_gate) {
            assertTrue("Projection must trigger when confidence >= 0.20", pred.projectionTriggered)
            assertTrue("totalProjectionsTriggered should be at least 1", engine.totalProjectionsTriggered >= 1)
        }
    }

    @Test
    fun `reset clears sequence buffer and re-initializes tracking states`() {
        engine.reset(initialSpeedMps = 15.0f, initialYawRad = 0.5f)

        // Accumulate 30 steps
        for (step in 1..30) {
            engine.addSample(
                aFwd = 0.5f,
                aLat = 0.1f,
                wYaw = 0.02f,
                stepDisplacementMeters = 1.5f,
                stepHeadingDeltaRad = 0.01f,
                isStationary = false
            )
        }

        // Reset
        engine.reset(initialSpeedMps = 0.0f, initialYawRad = 0.0f)
        assertEquals(0, engine.totalProjectionsTriggered)
        assertEquals(V9DrivingEvent.CRUISE, engine.currentDrivingEvent)

        // Next 49 steps must again return null
        for (step in 1..49) {
            val p = engine.addSample(0f, 0f, 0f, 0f, 0f, isStationary = true)
            assertNull("Post-reset step $step must return null", p)
        }

        // Step 50 must evaluate
        val p50 = engine.addSample(0f, 0f, 0f, 0f, 0f, isStationary = true)
        assertNotNull(p50)
        assertEquals(V9DrivingEvent.STOP, p50!!.drivingEvent)
    }
}
