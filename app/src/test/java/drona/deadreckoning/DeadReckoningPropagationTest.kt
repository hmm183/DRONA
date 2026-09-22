package drona.deadreckoning

import kotlin.math.abs
import drona.deadreckoning.fusion.HeadingPolicy
import drona.deadreckoning.fusion.VehicleFusionEkf
import drona.deadreckoning.support.SyntheticDrive
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.osmdroid.util.GeoPoint

/**
 * Stage 2 verification.
 *
 * Every test feeds the estimator *perfect* model predictions derived from ground truth
 * the same way the training pipeline builds its targets. Any residual error is therefore
 * attributable to the fusion mathematics alone, which is what isolates the heading
 * double-count and the start-of-window frame error from model quality.
 */
class DeadReckoningPropagationTest {

    private val origin = GeoPoint(16.5, 80.6)

    /**
     * Ground-truth position as a GeoPoint, using the same flat-earth scaling the
     * estimator uses so the comparison isolates fusion error rather than projection
     * differences.
     */
    private fun expectedPosition(truth: SyntheticDrive.Truth, poseIndex: Int): GeoPoint {
        val pose = truth.poses[poseIndex]
        return GeoPoint(
            origin.latitude + pose.northMeters / 111_111.0,
            origin.longitude + pose.eastMeters / (111_111.0 * Math.cos(Math.toRadians(origin.latitude)))
        )
    }

    /**
     * Runs a whole drive through the estimator and reports the position error at the
     * last sample the windows actually cover.
     *
     * Comparing against the very final pose would be wrong: whole windows are consumed,
     * so up to one window of samples remains unmodelled at the tail. That tail is truth
     * the estimator was never given, not estimator error.
     */
    private fun replay(
        truth: SyntheticDrive.Truth,
        policy: HeadingPolicy,
        feedGyro: Boolean
    ): Double {
        val fusion = VehicleFusionEkf(policy)
        val first = truth.poses.first()
        fusion.reset(origin, first.speedMps, first.headingDegrees, 5.0)

        val dt = 1.0 / SyntheticDrive.SAMPLE_RATE_HZ

        truth.windows.forEach { window ->
            if (feedGyro) {
                window.gyroYawRates.forEach { rate -> fusion.predictGyro(rate, dt) }
            }
            fusion.predict(
                forwardMeters = window.forwardMeters,
                lateralMeters = window.lateralMeters,
                headingDeltaRadians = window.headingDeltaRadians,
                intervalSeconds = window.spanSeconds
            )
        }

        val covered = truth.windows.last().endIndex
        return fusion.state().position.distanceToAsDouble(expectedPosition(truth, covered))
    }

    @Test
    fun `straight drive reconstructs to sub metre accuracy`() {
        val truth = SyntheticDrive.straightDrive(seconds = 30.0, speedMps = 15.0)

        val error = replay(truth, HeadingPolicy.GYRO_ONLY, feedGyro = true)

        assertTrue("path should be roughly 450 m, was ${truth.pathLengthMeters}", truth.pathLengthMeters > 400.0)
        assertTrue("straight-line error was $error m", error < 1.0)
    }

    @Test
    fun `stationary vehicle does not drift`() {
        val truth = SyntheticDrive.stationary(seconds = 20.0)

        val error = replay(truth, HeadingPolicy.GYRO_ONLY, feedGyro = true)

        assertTrue("stationary drift was $error m", error < 0.5)
    }

    @Test
    fun `gyro only policy reconstructs a turning drive`() {
        val truth = SyntheticDrive.turningDrive()

        val error = replay(truth, HeadingPolicy.GYRO_ONLY, feedGyro = true)
        val relative = error / truth.pathLengthMeters

        assertTrue(
            "turning error $error m over ${truth.pathLengthMeters} m = ${relative * 100}%",
            relative < 0.02
        )
    }

    @Test
    fun `model only policy reconstructs a turning drive`() {
        val truth = SyntheticDrive.turningDrive()

        // No gyro fed at all: the model's per-window heading change is the sole source.
        val error = replay(truth, HeadingPolicy.MODEL_ONLY, feedGyro = false)
        val relative = error / truth.pathLengthMeters

        assertTrue(
            "turning error $error m over ${truth.pathLengthMeters} m = ${relative * 100}%",
            relative < 0.02
        )
    }

    @Test
    fun `combined policy agrees with gyro when the model is consistent`() {
        val truth = SyntheticDrive.turningDrive()

        val error = replay(truth, HeadingPolicy.GYRO_WITH_MODEL_UPDATE, feedGyro = true)
        val relative = error / truth.pathLengthMeters

        assertTrue(
            "combined policy error $error m = ${relative * 100}%",
            relative < 0.02
        )
    }

    @Test
    fun `heading is not counted twice on a ninety degree turn`() {
        val truth = SyntheticDrive.rightAngleTurn()
        val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_WITH_MODEL_UPDATE)
        val first = truth.poses.first()
        fusion.reset(origin, first.speedMps, first.headingDegrees, 5.0)

        val dt = 1.0 / SyntheticDrive.SAMPLE_RATE_HZ
        truth.windows.forEach { window ->
            window.gyroYawRates.forEach { rate -> fusion.predictGyro(rate, dt) }
            fusion.predict(
                forwardMeters = window.forwardMeters,
                lateralMeters = window.lateralMeters,
                headingDeltaRadians = window.headingDeltaRadians,
                intervalSeconds = window.spanSeconds
            )
        }

        val expectedHeading = ((truth.finalPose.headingDegrees % 360.0) + 360.0) % 360.0
        val actualHeading = fusion.state().headingDegrees
        val headingError = abs(((actualHeading - expectedHeading + 540.0) % 360.0) - 180.0)

        // Ground truth turns 90 degrees. Double counting would land near 180.
        assertEquals("truth should be a 90 degree turn", 90.0, expectedHeading, 1.0)
        assertTrue("heading error was $headingError deg (actual $actualHeading)", headingError < 3.0)
    }

    @Test
    fun `double counting would be detectable if both sources were applied`() {
        // Guards the regression directly: applying gyro AND the model delta as two
        // independent rotations must produce a visibly worse heading than the policy path.
        val truth = SyntheticDrive.rightAngleTurn()
        val dt = 1.0 / SyntheticDrive.SAMPLE_RATE_HZ

        val correct = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY)
        correct.reset(origin, truth.poses.first().speedMps, truth.poses.first().headingDegrees, 5.0)
        truth.windows.forEach { window ->
            window.gyroYawRates.forEach { correct.predictGyro(it, dt) }
            correct.predict(window.forwardMeters, window.lateralMeters, window.headingDeltaRadians, window.spanSeconds)
        }

        val doubled = VehicleFusionEkf(HeadingPolicy.MODEL_ONLY)
        doubled.reset(origin, truth.poses.first().speedMps, truth.poses.first().headingDegrees, 5.0)
        truth.windows.forEach { window ->
            // Feed gyro as well, which MODEL_ONLY then overrides from the window anchor,
            // plus an extra explicit model rotation to emulate the old additive behaviour.
            window.gyroYawRates.forEach { doubled.predictGyro(it, dt) }
            doubled.predict(window.forwardMeters, window.lateralMeters, window.headingDeltaRadians * 2.0, window.spanSeconds)
        }

        val expected = ((truth.finalPose.headingDegrees % 360.0) + 360.0) % 360.0
        fun headingError(value: Double) = abs(((value - expected + 540.0) % 360.0) - 180.0)

        val correctError = headingError(correct.state().headingDegrees)
        val doubledError = headingError(doubled.state().headingDegrees)

        assertTrue("correct heading error $correctError should be small", correctError < 3.0)
        assertTrue(
            "doubled heading error $doubledError should be much larger than $correctError",
            doubledError > correctError + 40.0
        )
    }

    @Test
    fun `start of window rotation beats end of window rotation on turns`() {
        // The original code advanced heading and then rotated the displacement, which
        // uses the wrong frame. Emulate that by pre-rotating the displacement into the
        // end-of-window frame and confirm it is measurably worse.
        val truth = SyntheticDrive.turningDrive(speedMps = 14.0, yawRateDegPerSec = 12.0)
        val dt = 1.0 / SyntheticDrive.SAMPLE_RATE_HZ

        fun run(useEndFrame: Boolean): Double {
            val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY)
            val first = truth.poses.first()
            fusion.reset(origin, first.speedMps, first.headingDegrees, 5.0)
            truth.windows.forEach { window ->
                window.gyroYawRates.forEach { fusion.predictGyro(it, dt) }
                var forward = window.forwardMeters
                var lateral = window.lateralMeters
                if (useEndFrame) {
                    // Re-express the same displacement in the end-of-window frame.
                    val delta = window.headingDeltaRadians
                    val rotatedForward = forward * Math.cos(delta) + lateral * Math.sin(delta)
                    val rotatedLateral = -forward * Math.sin(delta) + lateral * Math.cos(delta)
                    forward = rotatedForward
                    lateral = rotatedLateral
                }
                fusion.predict(forward, lateral, window.headingDeltaRadians, window.spanSeconds)
            }
            val covered = truth.windows.last().endIndex
            return fusion.state().position.distanceToAsDouble(expectedPosition(truth, covered))
        }

        val correctFrameError = run(useEndFrame = false)
        val wrongFrameError = run(useEndFrame = true)

        assertTrue("correct-frame error was $correctFrameError m", correctFrameError < 5.0)
        assertTrue(
            "wrong-frame error $wrongFrameError m should exceed correct $correctFrameError m",
            wrongFrameError > correctFrameError * 3.0
        )
    }

    @Test
    fun `implausible model output is rejected instead of corrupting heading`() {
        val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_WITH_MODEL_UPDATE)
        fusion.reset(origin, 10.0, 0.0, 5.0)

        // Gyro says we went essentially straight.
        repeat(19) { fusion.predictGyro(0.0, 0.1) }
        // Model claims a 90 degree turn in the same window.
        fusion.predict(19.0, 0.0, Math.toRadians(90.0), 1.9)

        assertEquals("one update should have been rejected", 1, fusion.rejectedHeadingUpdates)
        assertTrue(
            "heading should stay near zero, was ${fusion.state().headingDegrees}",
            abs(((fusion.state().headingDegrees + 180.0) % 360.0) - 180.0) < 5.0
        )
    }

    @Test
    fun `absurd displacement is ignored`() {
        val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY)
        fusion.reset(origin, 10.0, 0.0, 5.0)
        val before = fusion.state().position

        fusion.predict(5_000.0, 0.0, 0.0, 1.9)

        assertEquals(
            "position must not move on an implausible window",
            0.0,
            fusion.state().position.distanceToAsDouble(before),
            0.5
        )
    }

    @Test
    fun `non finite input does not corrupt the filter`() {
        val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY)
        fusion.reset(origin, 10.0, 0.0, 5.0)

        assertEquals(null, fusion.predict(Double.NaN, 0.0, 0.0, 1.9))
        assertEquals(null, fusion.predict(1.0, Double.POSITIVE_INFINITY, 0.0, 1.9))
        assertEquals(null, fusion.predictGyro(Double.NaN, 0.1))

        val state = fusion.state()
        assertTrue("latitude stayed finite", state.position.latitude.isFinite())
        assertTrue("heading stayed finite", state.headingDegrees.isFinite())
    }

    @Test
    fun `heading innovation surfaces model gyro disagreement`() {
        val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_WITH_MODEL_UPDATE)
        fusion.reset(origin, 10.0, 0.0, 5.0)

        repeat(19) { fusion.predictGyro(Math.toRadians(5.0), 0.1) }
        val gyroAccumulated = Math.toRadians(5.0) * 1.9
        fusion.predict(19.0, 0.0, gyroAccumulated, 1.9)

        assertEquals(
            "a consistent model should show near-zero innovation",
            0.0,
            fusion.lastHeadingInnovationRadians,
            1e-6
        )
        assertEquals(0, fusion.rejectedHeadingUpdates)
    }
}
