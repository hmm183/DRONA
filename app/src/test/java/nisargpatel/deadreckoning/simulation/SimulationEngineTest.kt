package nisargpatel.deadreckoning.simulation

import org.junit.Assert.*
import org.junit.Test
import org.osmdroid.util.GeoPoint

class SimulationEngineTest {

    private val sampleRoute = listOf(
        GeoPoint(16.5160, 80.5780),
        GeoPoint(16.5130, 80.6000),
        GeoPoint(16.5100, 80.6200),
        GeoPoint(16.5062, 80.6480)
    )

    @Test
    fun testGroundTruthTrajectoryGeneration() {
        val trajectory = GroundTruthTrajectoryGenerator.generate(
            routePoints = sampleRoute,
            targetSpeedMps = 12.0,
            dtSeconds = 0.1
        )

        assertTrue("Trajectory should have generated states", trajectory.size > 100)
        assertEquals("First point matches source", sampleRoute.first().latitude, trajectory.first().position.latitude, 0.0001)
        assertEquals("Step indices increment monotonically", 1, trajectory[1].stepIndex)
        assertTrue("Distance increments monotonically", trajectory.last().distanceMeters > trajectory.first().distanceMeters)
    }

    @Test
    fun testGnssBlackoutStrictStarvation() {
        val config = SimulationConfig(
            blackoutStartPct = 0.30f,
            blackoutEndPct = 0.70f,
            randomSeed = 26168L
        )
        val generator = SyntheticSensorGenerator(config)

        val trajectory = GroundTruthTrajectoryGenerator.generate(
            routePoints = sampleRoute,
            targetSpeedMps = 12.0,
            dtSeconds = 0.1
        )

        var blackoutObservationsCount = 0
        var activeObservationsCount = 0

        trajectory.forEach { gt ->
            val gnss = generator.generateGnss(gt, gt.timestampSeconds)
            if (gt.progressFraction in 0.30..0.70) {
                assertTrue("During blackout, fix must be null", gnss.fix == null)
                assertTrue("During blackout, isBlackout must be true", gnss.isBlackout)
                blackoutObservationsCount++
            } else {
                assertFalse("Outside blackout, isBlackout must be false", gnss.isBlackout)
                assertNotNull("Outside blackout, fix must be non-null", gnss.fix)
                activeObservationsCount++
            }
        }

        assertTrue("Must have blackout samples", blackoutObservationsCount > 50)
        assertTrue("Must have active GNSS samples", activeObservationsCount > 50)
    }

    @Test
    fun testSimulationDeterminismForSameSeed() {
        val config1 = SimulationConfig(randomSeed = 26168L)
        val config2 = SimulationConfig(randomSeed = 26168L)

        val gen1 = SyntheticSensorGenerator(config1)
        val gen2 = SyntheticSensorGenerator(config2)

        val trajectory = GroundTruthTrajectoryGenerator.generate(
            routePoints = sampleRoute,
            targetSpeedMps = 12.0,
            dtSeconds = 0.1
        )

        for (i in 0 until 50) {
            val gt = trajectory[i]
            val imu1 = gen1.generateImuStep(gt, 0.1)
            val imu2 = gen2.generateImuStep(gt, 0.1)
            assertEquals("Forward meters must match deterministically", imu1.forwardMeters, imu2.forwardMeters, 1e-9)
            assertEquals("Heading delta must match deterministically", imu1.headingDeltaRadians, imu2.headingDeltaRadians, 1e-9)
        }
    }

    @Test
    fun testNaiveDeadReckoningDivergesUnderGyroBias() {
        val config = SimulationConfig(
            blackoutStartPct = 0.0f,
            blackoutEndPct = 1.0f, // entire run is blackout
            gyroBiasDps = 1.5,
            randomSeed = 26168L
        )
        val generator = SyntheticSensorGenerator(config)
        val naive = NaiveDeadReckoningBaseline()

        val trajectory = GroundTruthTrajectoryGenerator.generate(
            routePoints = sampleRoute,
            targetSpeedMps = 12.0,
            dtSeconds = 0.1
        )

        val first = trajectory.first()
        naive.reset(first.position, first.headingDegrees, first.speedMps)

        trajectory.forEach { gt ->
            val imu = generator.generateImuStep(gt, 0.1)
            val gnss = generator.generateGnss(gt, gt.timestampSeconds)
            naive.step(imu, gnss)
        }

        val lastGt = trajectory.last().position
        val lastNaive = naive.currentPosition!!
        val error = lastNaive.distanceToAsDouble(lastGt)

        assertTrue("Naive DR under 1.5 dps uncompensated bias must drift significantly (> 15m)", error > 15.0)
    }

    @Test
    fun testFiveSecondRollingDriftComputation() {
        val engine = SimulationMetricsEngine()
        val gt = GroundTruthState(
            stepIndex = 0,
            timestampSeconds = 0.0,
            position = sampleRoute.first(),
            pe = 0.0,
            pn = 0.0,
            speedMps = 12.0,
            headingDegrees = 45.0,
            yawRateDps = 0.0,
            accelMps2 = 0.0,
            distanceMeters = 0.0,
            progressFraction = 0.0,
            motionMode = nisargpatel.deadreckoning.fusion.VehicleFusionImmUkf.MotionMode.CONSTANT_VELOCITY
        )

        // Step across 6 seconds
        var latestMetrics = engine.step(
            stepIndex = 0,
            timeSec = 0.0,
            isBlackout = true,
            gt = gt,
            naivePos = sampleRoute.first(),
            hybridPos = sampleRoute.first(),
            immPos = sampleRoute.first(),
            matchedPos = sampleRoute.first(),
            estimatedHeadingDeg = 45.0,
            estimatedSpeedMps = 12.0
        )

        for (i in 1..60) {
            val t = i * 0.1
            // Simulated slowly drifting position
            val lat = sampleRoute.first().latitude + (i * 0.00001)
            val drifted = GeoPoint(lat, sampleRoute.first().longitude)
            latestMetrics = engine.step(
                stepIndex = i,
                timeSec = t,
                isBlackout = true,
                gt = gt,
                naivePos = drifted,
                hybridPos = drifted,
                immPos = drifted,
                matchedPos = drifted,
                estimatedHeadingDeg = 45.0,
                estimatedSpeedMps = 12.0
            )
        }

        assertTrue("5s rolling drift must be greater than zero", latestMetrics.hybridMaxDrift5s > 0.0)
        assertTrue("5s rolling drift should be constrained to window size", latestMetrics.hybridMaxDrift5s < 100.0)
    }
}
