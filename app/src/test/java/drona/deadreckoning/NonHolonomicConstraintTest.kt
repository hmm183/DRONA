package drona.deadreckoning

import kotlin.math.abs
import kotlin.math.tan
import kotlin.random.Random
import drona.deadreckoning.fusion.HeadingPolicy
import drona.deadreckoning.fusion.NonHolonomicConfig
import drona.deadreckoning.fusion.VehicleFusionEkf
import drona.deadreckoning.support.SyntheticDrive
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.osmdroid.util.GeoPoint

/**
 * Stage 3 verification.
 *
 * Two properties matter and they pull against each other:
 *
 * 1. The constraint must not damage a physically honest turn. A turning vehicle really
 *    does accumulate lateral displacement in its start-of-window frame.
 * 2. The constraint must remove lateral error that a vehicle could not physically have
 *    produced, which is where the drift reduction comes from.
 *
 * A test that only checked the second property would pass with `lateral = 0`, which
 * would break every turn, so both are asserted.
 */
class NonHolonomicConstraintTest {

    private val origin = GeoPoint(16.5, 80.6)
    private val dt = 1.0 / SyntheticDrive.SAMPLE_RATE_HZ

    private fun expectedPosition(truth: SyntheticDrive.Truth, poseIndex: Int): GeoPoint {
        val pose = truth.poses[poseIndex]
        return GeoPoint(
            origin.latitude + pose.northMeters / 111_111.0,
            origin.longitude + pose.eastMeters / (111_111.0 * Math.cos(Math.toRadians(origin.latitude)))
        )
    }

    /**
     * @param lateralErrorMeters spurious sideslip injected into every window, simulating
     *   a model that reports lateral motion a car cannot perform.
     */
    private fun replay(
        truth: SyntheticDrive.Truth,
        nhc: NonHolonomicConfig,
        lateralErrorMeters: Double = 0.0,
        feedGyro: Boolean = true
    ): Double {
        val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY, nhc)
        val first = truth.poses.first()
        fusion.reset(origin, first.speedMps, first.headingDegrees, 5.0)

        truth.windows.forEach { window ->
            if (feedGyro) window.gyroYawRates.forEach { fusion.predictGyro(it, dt) }
            fusion.predict(
                forwardMeters = window.forwardMeters,
                lateralMeters = window.lateralMeters + lateralErrorMeters,
                headingDeltaRadians = window.headingDeltaRadians,
                intervalSeconds = window.spanSeconds
            )
        }

        val covered = truth.windows.last().endIndex
        return fusion.state().position.distanceToAsDouble(expectedPosition(truth, covered))
    }

    @Test
    fun `constraint does not damage an honest turning drive`() {
        val truth = SyntheticDrive.turningDrive()

        val withoutNhc = replay(truth, NonHolonomicConfig.DISABLED)
        val withNhc = replay(truth, NonHolonomicConfig())

        // Ground truth already satisfies the constraint, so enabling it must be near-neutral.
        assertTrue("without NHC error $withoutNhc m", withoutNhc < 6.0)
        assertTrue("with NHC error $withNhc m should stay close to $withoutNhc m", withNhc < withoutNhc + 1.0)
    }

    @Test
    fun `constraint removes injected sideslip on a straight drive`() {
        val truth = SyntheticDrive.straightDrive(seconds = 30.0, speedMps = 15.0)
        val injected = 2.5

        val withoutNhc = replay(truth, NonHolonomicConfig.DISABLED, lateralErrorMeters = injected)
        val withNhc = replay(truth, NonHolonomicConfig(), lateralErrorMeters = injected)

        assertTrue("uncorrected sideslip should be large, was $withoutNhc m", withoutNhc > 20.0)
        assertTrue(
            "NHC should cut sideslip drift substantially: $withNhc m vs $withoutNhc m",
            withNhc < withoutNhc * 0.4
        )
    }

    @Test
    fun `constraint removes injected sideslip on a turning drive`() {
        val truth = SyntheticDrive.turningDrive()
        val injected = 2.5

        val withoutNhc = replay(truth, NonHolonomicConfig.DISABLED, lateralErrorMeters = injected)
        val withNhc = replay(truth, NonHolonomicConfig(), lateralErrorMeters = injected)

        assertTrue(
            "NHC should reduce drift on a turn too: $withNhc m vs $withoutNhc m",
            withNhc < withoutNhc * 0.5
        )
    }

    @Test
    fun `constraint reduces drift across a range of injected sideslip`() {
        val truth = SyntheticDrive.turningDrive()

        listOf(-3.0, -1.5, 1.5, 3.0).forEach { injected ->
            val withoutNhc = replay(truth, NonHolonomicConfig.DISABLED, lateralErrorMeters = injected)
            val withNhc = replay(truth, NonHolonomicConfig(), lateralErrorMeters = injected)
            assertTrue(
                "injected $injected m: NHC $withNhc m should beat $withoutNhc m",
                withNhc < withoutNhc
            )
        }
    }

    @Test
    fun `disabled constraint is a no-op`() {
        val truth = SyntheticDrive.turningDrive()

        val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY, NonHolonomicConfig.DISABLED)
        fusion.reset(origin, truth.poses.first().speedMps, truth.poses.first().headingDegrees, 5.0)
        truth.windows.forEach { window ->
            window.gyroYawRates.forEach { fusion.predictGyro(it, dt) }
            fusion.predict(window.forwardMeters, window.lateralMeters, window.headingDeltaRadians, window.spanSeconds)
        }

        assertEquals("no constraint updates expected", 0, fusion.nonHolonomicUpdates)
        assertEquals(0.0, fusion.lastNonHolonomicCorrectionMeters, 1e-12)
    }

    @Test
    fun `straight driving implies zero lateral displacement`() {
        val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY, NonHolonomicConfig())
        fusion.reset(origin, 15.0, 0.0, 5.0)

        // Perfectly straight: no yaw at all.
        repeat(19) { fusion.predictGyro(0.0, dt) }
        // Model wrongly reports 3 m of sideways motion.
        fusion.predict(forwardMeters = 28.5, lateralMeters = 3.0, headingDeltaRadians = 0.0, intervalSeconds = 1.9)

        assertTrue("constraint should have engaged", fusion.nonHolonomicUpdates > 0)
        // Implied lateral is zero, so the correction opposes the full 3 m at the configured gain.
        assertEquals(-3.0 * 0.7, fusion.lastNonHolonomicCorrectionMeters, 1e-6)
    }

    @Test
    fun `gyro integral path agrees with the closed form on a constant turn`() {
        // With a constant yaw rate the shape-integral ratio must equal tan(delta / 2).
        val yawRate = Math.toRadians(9.0)
        val span = 1.9
        val headingDelta = yawRate * span

        val withGyro = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY, NonHolonomicConfig())
        withGyro.reset(origin, 12.0, 0.0, 5.0)
        repeat(19) { withGyro.predictGyro(yawRate, dt) }
        withGyro.predict(22.35, 0.0, headingDelta, span)
        val gyroCorrection = withGyro.lastNonHolonomicCorrectionMeters

        val withoutGyro = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY, NonHolonomicConfig())
        withoutGyro.reset(origin, 12.0, 0.0, 5.0)
        withoutGyro.predict(22.35, 0.0, headingDelta, span)
        val closedFormCorrection = withoutGyro.lastNonHolonomicCorrectionMeters

        assertEquals(
            "gyro-integral and closed-form constraint should agree",
            closedFormCorrection,
            gyroCorrection,
            0.05
        )

        // And both should match the analytic expectation.
        val expected = -(0.0 - 22.35 * tan(headingDelta / 2.0)) * 0.7
        assertEquals(expected, gyroCorrection, 0.05)
    }

    @Test
    fun `constraint is skipped when stationary`() {
        val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY, NonHolonomicConfig())
        fusion.reset(origin, 0.0, 0.0, 5.0)

        // No gyro history and no heading change: nothing observable to constrain against,
        // but the closed-form fallback is still well defined at zero.
        fusion.predict(0.0, 0.0, 0.0, 1.9)

        assertTrue("state must stay finite", fusion.state().position.latitude.isFinite())
        assertEquals(0.0, fusion.lastNonHolonomicCorrectionMeters, 1e-9)
    }

    @Test
    fun `constraint stays finite through a hard turn`() {
        val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY, NonHolonomicConfig())
        fusion.reset(origin, 8.0, 0.0, 5.0)

        // 60 deg/s for 1.9 s is a very hard turn, near the fallback's singularity.
        val yawRate = Math.toRadians(60.0)
        repeat(19) { fusion.predictGyro(yawRate, dt) }
        fusion.predict(10.0, 4.0, yawRate * 1.9, 1.9)

        val state = fusion.state()
        assertTrue("latitude finite", state.position.latitude.isFinite())
        assertTrue("longitude finite", state.position.longitude.isFinite())
        assertTrue("heading finite", state.headingDegrees.isFinite())
        assertTrue(
            "correction clamped, was ${fusion.lastNonHolonomicCorrectionMeters}",
            abs(fusion.lastNonHolonomicCorrectionMeters) <= 6.0 + 1e-9
        )
    }

    @Test
    fun `correction is clamped for absurd lateral claims`() {
        val fusion = VehicleFusionEkf(HeadingPolicy.GYRO_ONLY, NonHolonomicConfig())
        fusion.reset(origin, 15.0, 0.0, 5.0)

        repeat(19) { fusion.predictGyro(0.0, dt) }
        fusion.predict(28.5, 40.0, 0.0, 1.9)

        assertEquals(
            "correction must clamp at maxCorrectionMeters",
            -6.0,
            fusion.lastNonHolonomicCorrectionMeters,
            1e-9
        )
    }

    @Test
    fun `constraint holds up under randomised drives`() {
        val random = Random(11)

        repeat(40) {
            val truth = SyntheticDrive.generate(
                listOf(
                    SyntheticDrive.Segment(4.0, random.nextDouble(5.0, 20.0), 0.0),
                    SyntheticDrive.Segment(6.0, random.nextDouble(5.0, 20.0), random.nextDouble(-12.0, 12.0)),
                    SyntheticDrive.Segment(4.0, random.nextDouble(5.0, 20.0), 0.0)
                )
            )
            val injected = random.nextDouble(-3.0, 3.0)
            if (abs(injected) < 0.5) return@repeat

            val withoutNhc = replay(truth, NonHolonomicConfig.DISABLED, lateralErrorMeters = injected)
            val withNhc = replay(truth, NonHolonomicConfig(), lateralErrorMeters = injected)

            assertTrue(
                "injected $injected m: NHC $withNhc m should not be worse than $withoutNhc m",
                withNhc <= withoutNhc + 0.5
            )
        }
    }
}
