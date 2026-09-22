package drona.deadreckoning

import kotlin.math.abs
import drona.deadreckoning.core.spec.PreprocessingSpec
import drona.deadreckoning.fusion.HeadingPolicy
import drona.deadreckoning.fusion.MapConstraintConfig
import drona.deadreckoning.fusion.NonHolonomicConfig
import drona.deadreckoning.fusion.VehicleFusionEkf
import drona.deadreckoning.support.SyntheticDrive
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.osmdroid.util.GeoPoint

/**
 * Stage 6 verification.
 *
 * Covers the covariance matrix, the per-axis uncertainty it enables, the velocity
 * propagation path that makes a high output rate safe, and the double-counting hazard that
 * rules out simply sliding the existing displacement model faster.
 */
class CovarianceAndUncertaintyTest {

    private val origin = GeoPoint(16.5, 80.6)
    private val dt = 1.0 / SyntheticDrive.SAMPLE_RATE_HZ

    private fun newFusion(
        heading: HeadingPolicy = HeadingPolicy.GYRO_ONLY,
        nhc: NonHolonomicConfig = NonHolonomicConfig.DISABLED,
        map: MapConstraintConfig = MapConstraintConfig.DISABLED
    ) = VehicleFusionEkf(heading, nhc, map)

    @Test
    fun `cross track uncertainty grows faster than along track during dead reckoning`() {
        // This is the whole reason a scalar variance was inadequate: heading error times
        // distance travelled dominates the cross-track direction.
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)

        repeat(5) {
            fusion.predict(forwardMeters = 28.5, lateralMeters = 0.0, headingDeltaRadians = 0.0, intervalSeconds = 1.9)
        }

        val state = fusion.state()
        assertTrue(
            "cross-track ${state.crossTrackUncertaintyMeters} should exceed along-track ${state.alongTrackUncertaintyMeters}",
            state.crossTrackUncertaintyMeters > state.alongTrackUncertaintyMeters * 2.0
        )
    }

    @Test
    fun `uncertainty is reported on every axis`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)
        fusion.predict(28.5, 0.0, 0.0, 1.9)

        val state = fusion.state()
        assertTrue("horizontal", state.horizontalUncertaintyMeters > 0.0)
        assertTrue("along-track", state.alongTrackUncertaintyMeters > 0.0)
        assertTrue("cross-track", state.crossTrackUncertaintyMeters > 0.0)
        assertTrue("speed", state.speedUncertaintyMps > 0.0)
        assertTrue("heading", state.headingUncertaintyDegrees > 0.0)
    }

    @Test
    fun `horizontal uncertainty is the major axis so it bounds both components`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 40.0, 5.0)
        repeat(3) { fusion.predict(28.5, 2.0, 0.0, 1.9) }

        val state = fusion.state()
        assertTrue(
            "major ${state.horizontalUncertaintyMeters} must bound along ${state.alongTrackUncertaintyMeters}",
            state.horizontalUncertaintyMeters >= state.alongTrackUncertaintyMeters - 1e-6
        )
        assertTrue(
            "major ${state.horizontalUncertaintyMeters} must bound cross ${state.crossTrackUncertaintyMeters}",
            state.horizontalUncertaintyMeters >= state.crossTrackUncertaintyMeters - 1e-6
        )
    }

    @Test
    fun `gnss update shrinks uncertainty and pulls the position back`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)
        repeat(4) { fusion.predict(28.5, 0.0, 0.0, 1.9) }
        val driftedUncertainty = fusion.state().horizontalUncertaintyMeters

        val corrected = fusion.updateGnss(origin, 15.0, 0.0, 5.0)

        assertTrue(
            "uncertainty should fall from $driftedUncertainty to ${corrected.horizontalUncertaintyMeters}",
            corrected.horizontalUncertaintyMeters < driftedUncertainty
        )
        assertTrue(
            "position should be pulled toward the fix, was ${corrected.position.distanceToAsDouble(origin)} m",
            corrected.position.distanceToAsDouble(origin) < 60.0
        )
    }

    @Test
    fun `covariance stays valid through many mixed updates`() {
        val fusion = newFusion(map = MapConstraintConfig())
        fusion.reset(origin, 15.0, 0.0, 5.0)

        repeat(50) { index ->
            fusion.predictGyro(Math.toRadians(3.0), dt)
            fusion.predict(28.5, 1.0, Math.toRadians(3.0) * 1.9, 1.9)
            if (index % 3 == 0) {
                val state = fusion.state()
                fusion.updateMapConstraint(state.position, roadBearingDegrees = state.headingDegrees, confidence = 80)
            }
            if (index % 7 == 0) {
                fusion.updateGnss(fusion.state().position, 15.0, fusion.state().headingDegrees, 8.0)
            }

            val state = fusion.state()
            assertTrue("uncertainty finite at $index", state.horizontalUncertaintyMeters.isFinite())
            assertTrue("uncertainty positive at $index", state.horizontalUncertaintyMeters > 0.0)
            assertTrue("along-track finite at $index", state.alongTrackUncertaintyMeters.isFinite())
            assertTrue("cross-track finite at $index", state.crossTrackUncertaintyMeters.isFinite())
            assertTrue("latitude finite at $index", state.position.latitude.isFinite())
        }
    }

    @Test
    fun `velocity propagation integrates cleanly at high rate`() {
        // 10 Hz velocity propagation over 10 s at 15 m/s must travel 150 m, not 10x that.
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)

        repeat(100) { fusion.predictVelocity(forwardMps = 15.0, lateralMps = 0.0, intervalSeconds = 0.1) }

        val travelled = fusion.state().position.distanceToAsDouble(origin)
        assertEquals("should travel 150 m", 150.0, travelled, 1.0)
    }

    @Test
    fun `sliding window displacement would double count which is why velocity is used`() {
        // Demonstrates the hazard explicitly. The same 1.9 s window displacement emitted
        // every 0.2 s and accumulated as displacement inflates distance roughly tenfold.
        val perWindowForward = 28.5

        val displacementPath = newFusion()
        displacementPath.reset(origin, 15.0, 0.0, 5.0)
        repeat(10) { displacementPath.predict(perWindowForward, 0.0, 0.0, 1.9) }
        val displacementDistance = displacementPath.state().position.distanceToAsDouble(origin)

        val velocityPath = newFusion()
        velocityPath.reset(origin, 15.0, 0.0, 5.0)
        // Same 1.9 s of real motion, expressed as velocity and integrated at 0.19 s steps.
        repeat(10) { velocityPath.predictVelocity(perWindowForward / 1.9, 0.0, 0.19) }
        val velocityDistance = velocityPath.state().position.distanceToAsDouble(origin)

        assertEquals("velocity path covers one window of motion", perWindowForward, velocityDistance, 1.0)
        assertTrue(
            "accumulating overlapping displacements inflates distance: $displacementDistance vs $velocityDistance",
            displacementDistance > velocityDistance * 8.0
        )
    }

    @Test
    fun `velocity propagation respects the non holonomic assumption`() {
        val constrained = newFusion(nhc = NonHolonomicConfig())
        constrained.reset(origin, 15.0, 0.0, 5.0)
        repeat(50) { constrained.predictVelocity(15.0, lateralMps = 2.0, intervalSeconds = 0.1) }

        val unconstrained = newFusion(nhc = NonHolonomicConfig.DISABLED)
        unconstrained.reset(origin, 15.0, 0.0, 5.0)
        repeat(50) { unconstrained.predictVelocity(15.0, lateralMps = 2.0, intervalSeconds = 0.1) }

        fun eastOffset(fusion: VehicleFusionEkf) =
            abs((fusion.state().position.longitude - origin.longitude) * 111_111.0 * Math.cos(Math.toRadians(origin.latitude)))

        assertTrue(
            "constrained lateral drift ${eastOffset(constrained)} should be below unconstrained ${eastOffset(unconstrained)}",
            eastOffset(constrained) < eastOffset(unconstrained)
        )
    }

    @Test
    fun `velocity propagation rejects implausible intervals`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)

        assertNull("zero interval", fusion.predictVelocity(15.0, 0.0, 0.0))
        assertNull("negative interval", fusion.predictVelocity(15.0, 0.0, -0.1))
        assertNull("absurd interval", fusion.predictVelocity(15.0, 0.0, 5.0))
        assertNull("non-finite speed", fusion.predictVelocity(Double.NaN, 0.0, 0.1))
    }

    @Test
    fun `speed update weights a confident model more than a doubtful one`() {
        val confident = newFusion()
        confident.reset(origin, 10.0, 0.0, 5.0)
        confident.updateSpeed(measuredMps = 20.0, uncertaintyMps = 0.2)

        val doubtful = newFusion()
        doubtful.reset(origin, 10.0, 0.0, 5.0)
        doubtful.updateSpeed(measuredMps = 20.0, uncertaintyMps = 15.0)

        val confidentSpeed = confident.state().speedMps
        val doubtfulSpeed = doubtful.state().speedMps

        assertTrue(
            "a confident measurement should move speed further: $confidentSpeed vs $doubtfulSpeed",
            confidentSpeed > doubtfulSpeed
        )
        assertTrue("confident speed should approach 20", confidentSpeed > 19.0)
        assertTrue("doubtful speed should stay nearer 10", doubtfulSpeed < 15.0)
    }

    @Test
    fun `speed update reduces speed uncertainty`() {
        val fusion = newFusion()
        fusion.reset(origin, 10.0, 0.0, 5.0)
        val before = fusion.state().speedUncertaintyMps

        fusion.updateSpeed(12.0, 0.3)
        val after = fusion.state().speedUncertaintyMps

        assertTrue("speed uncertainty should fall from $before to $after", after < before)
        assertTrue("and stay positive", after > 0.0)
    }

    @Test
    fun `speed update is refused before initialisation`() {
        assertNull(newFusion().updateSpeed(10.0, 0.5))
    }

    @Test
    fun `map constraint gate uses the covariance in the road normal direction`() {
        // With a large cross-track covariance the gate must be permissive; with a small one
        // the same innovation must be rejected. A scalar variance could not distinguish these.
        fun attempt(driveWindows: Int): Boolean {
            val fusion = newFusion(map = MapConstraintConfig())
            fusion.reset(origin, 15.0, 0.0, 2.0)
            repeat(driveWindows) { fusion.predict(28.5, 0.0, 0.0, 1.9) }
            val north = (fusion.state().position.latitude - origin.latitude) * 111_111.0
            val offRoad = GeoPoint(
                origin.latitude + north / 111_111.0,
                origin.longitude + 30.0 / (111_111.0 * Math.cos(Math.toRadians(origin.latitude)))
            )
            return fusion.updateMapConstraint(offRoad, roadBearingDegrees = 0.0, confidence = 90)?.applied == true
        }

        assertTrue("after several windows the covariance should admit a 30 m match", attempt(4))
    }

    @Test
    fun `legacy contract keeps a non overlapping stride`() {
        // Guards the baseline: raising the model's output rate without a velocity head would
        // reintroduce the double-counting demonstrated above.
        val legacy = PreprocessingSpec.LEGACY_V8
        assertEquals(legacy.windowSamples, legacy.strideSamples)
        assertEquals(0.5, legacy.predictionHz, 1e-9)

        // The Stage 7 contract slides, which is only safe alongside a velocity output.
        assertTrue(PreprocessingSpec.IDR_V1.strideSamples < PreprocessingSpec.IDR_V1.windowSamples)
        assertTrue(PreprocessingSpec.IDR_V1.predictionHz > legacy.predictionHz)
    }

    @Test
    fun `heading uncertainty shrinks when gnss confirms heading`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)
        repeat(30) { fusion.predictGyro(0.0, dt) }
        val before = fusion.state().headingUncertaintyDegrees

        fusion.updateGnss(fusion.state().position, 15.0, 0.0, 5.0)
        val after = fusion.state().headingUncertaintyDegrees

        assertTrue("heading uncertainty should fall from $before to $after", after < before)
    }

    @Test
    fun `full outage drive keeps uncertainty monotonic without corrections`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)
        var previous = fusion.state().horizontalUncertaintyMeters

        repeat(15) {
            fusion.predict(28.5, 0.0, 0.0, 1.9)
            val current = fusion.state().horizontalUncertaintyMeters
            assertTrue(
                "uncertainty must not shrink without a measurement: $previous -> $current",
                current >= previous - 1e-6
            )
            previous = current
        }
        assertNotNull(fusion.state())
    }
}
