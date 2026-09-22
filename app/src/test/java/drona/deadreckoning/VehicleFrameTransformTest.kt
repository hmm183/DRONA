package drona.deadreckoning

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.random.Random
import drona.deadreckoning.core.frame.Vector3
import drona.deadreckoning.core.frame.VehicleFrameTransform
import drona.deadreckoning.core.spec.DisplacementStatsGuard
import drona.deadreckoning.core.spec.PreprocessingSpec
import drona.deadreckoning.support.Mountings
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Stage 1 verification.
 *
 * The central property under test is mount invariance: the same physical vehicle
 * motion must produce the same vehicle-frame output no matter how the phone is
 * oriented. Before Stage 1 nothing rotated IMU vectors, so this property did not
 * hold and the model's notion of "forward" changed with the mounting.
 */
class VehicleFrameTransformTest {

    private val tolerance = 1e-3

    @Test
    fun `stationary phone reports zero linear acceleration in every mounting`() {
        Mountings.ALL.forEach { (name, matrix) ->
            val reading = Mountings.accelerometerReadingInPhoneFrame(Vector3.ZERO, matrix)
            val vehicle = VehicleFrameTransform.phoneToVehicle(reading, matrix, headingDegrees = 0.0)
            val linear = VehicleFrameTransform.removeGravity(vehicle)

            assertEquals("$name forward", 0.0, linear.x, tolerance)
            assertEquals("$name right", 0.0, linear.y, tolerance)
            assertEquals("$name down", 0.0, linear.z, tolerance)
        }
    }

    @Test
    fun `gravity alone lands on the vehicle down axis`() {
        Mountings.ALL.forEach { (name, matrix) ->
            val reading = Mountings.accelerometerReadingInPhoneFrame(Vector3.ZERO, matrix)
            val vehicle = VehicleFrameTransform.phoneToVehicle(reading, matrix, headingDegrees = 0.0)

            assertEquals("$name forward", 0.0, vehicle.x, tolerance)
            assertEquals("$name right", 0.0, vehicle.y, tolerance)
            assertEquals("$name down", -VehicleFrameTransform.GRAVITY, vehicle.z, tolerance)
        }
    }

    @Test
    fun `forward acceleration is mount independent`() {
        // Vehicle heading North, accelerating 2.0 m/s2 along its direction of travel.
        val trueAccelEnu = Vector3(0.0, 2.0, 0.0)

        Mountings.ALL.forEach { (name, matrix) ->
            val reading = Mountings.accelerometerReadingInPhoneFrame(trueAccelEnu, matrix)
            val linear = VehicleFrameTransform.removeGravity(
                VehicleFrameTransform.phoneToVehicle(reading, matrix, headingDegrees = 0.0)
            )

            assertEquals("$name forward", 2.0, linear.x, tolerance)
            assertEquals("$name right", 0.0, linear.y, tolerance)
            assertEquals("$name down", 0.0, linear.z, tolerance)
        }
    }

    @Test
    fun `raw phone axes disagree across mountings while vehicle frame agrees`() {
        val trueAccelEnu = Vector3(0.0, 2.0, 0.0)

        val rawForwardCandidates = Mountings.ALL.map { (_, matrix) ->
            // What the legacy pipeline fed the model as "channel 0".
            Mountings.accelerometerReadingInPhoneFrame(trueAccelEnu, matrix).x
        }
        val vehicleForward = Mountings.ALL.map { (_, matrix) ->
            val reading = Mountings.accelerometerReadingInPhoneFrame(trueAccelEnu, matrix)
            VehicleFrameTransform.removeGravity(
                VehicleFrameTransform.phoneToVehicle(reading, matrix, headingDegrees = 0.0)
            ).x
        }

        // The legacy raw channel is not consistent across mountings.
        assertTrue(
            "raw phone channel should vary across mountings, got $rawForwardCandidates",
            rawForwardCandidates.maxOrNull()!! - rawForwardCandidates.minOrNull()!! > 1.0
        )
        // The vehicle-frame channel is.
        assertTrue(
            "vehicle-frame forward should be consistent, got $vehicleForward",
            vehicleForward.maxOrNull()!! - vehicleForward.minOrNull()!! < tolerance
        )
    }

    @Test
    fun `forward acceleration is mount independent for arbitrary rotations`() {
        val random = Random(42)
        val trueAccelEnu = Vector3(0.0, 2.0, 0.0)

        repeat(300) {
            val matrix = Mountings.arbitraryRotation(
                yaw = random.nextDouble(-Math.PI, Math.PI),
                pitch = random.nextDouble(-Math.PI / 2, Math.PI / 2),
                roll = random.nextDouble(-Math.PI, Math.PI)
            )
            assertTrue("generated matrix must be a proper rotation", VehicleFrameTransform.isProperRotation(matrix))

            val reading = Mountings.accelerometerReadingInPhoneFrame(trueAccelEnu, matrix)
            val linear = VehicleFrameTransform.removeGravity(
                VehicleFrameTransform.phoneToVehicle(reading, matrix, headingDegrees = 0.0)
            )

            assertEquals(2.0, linear.x, 1e-2)
            assertEquals(0.0, linear.y, 1e-2)
            assertEquals(0.0, linear.z, 1e-2)
        }
    }

    @Test
    fun `lateral acceleration is resolved onto the right axis`() {
        // Vehicle heading North; a push toward East is a push to the vehicle's right.
        val trueAccelEnu = Vector3(1.5, 0.0, 0.0)

        Mountings.ALL.forEach { (name, matrix) ->
            val reading = Mountings.accelerometerReadingInPhoneFrame(trueAccelEnu, matrix)
            val linear = VehicleFrameTransform.removeGravity(
                VehicleFrameTransform.phoneToVehicle(reading, matrix, headingDegrees = 0.0)
            )

            assertEquals("$name forward", 0.0, linear.x, tolerance)
            assertEquals("$name right", 1.5, linear.y, tolerance)
        }
    }

    @Test
    fun `heading rotates the vehicle frame`() {
        // Travelling East at heading 90; acceleration toward East is now forward.
        val trueAccelEnu = Vector3(2.0, 0.0, 0.0)
        val matrix = Mountings.PORTRAIT_DASH_NORTH

        val reading = Mountings.accelerometerReadingInPhoneFrame(trueAccelEnu, matrix)
        val linear = VehicleFrameTransform.removeGravity(
            VehicleFrameTransform.phoneToVehicle(reading, matrix, headingDegrees = 90.0)
        )

        assertEquals(2.0, linear.x, tolerance)
        assertEquals(0.0, linear.y, tolerance)
    }

    @Test
    fun `enuToVehicle matches the training pipeline projection`() {
        // Guards against runtime and training drifting apart on what "forward" means.
        // Mirrors global_to_vehicle_frame() in src/data/targets.py.
        val random = Random(7)
        repeat(200) {
            val deltaEast = random.nextDouble(-50.0, 50.0)
            val deltaNorth = random.nextDouble(-50.0, 50.0)
            val headingDegrees = random.nextDouble(0.0, 360.0)
            val heading = Math.toRadians(headingDegrees)

            val expectedForward = deltaEast * sin(heading) + deltaNorth * cos(heading)
            val expectedRight = deltaEast * cos(heading) - deltaNorth * sin(heading)

            val actual = VehicleFrameTransform.enuToVehicle(
                Vector3(deltaEast, deltaNorth, 0.0),
                headingDegrees
            )

            assertEquals(expectedForward, actual.x, 1e-9)
            assertEquals(expectedRight, actual.y, 1e-9)
        }
    }

    @Test
    fun `idr v1 channel packing follows the declared channel order`() {
        val linear = Vector3(1.0, 2.0, 3.0)
        // Vehicle-frame angular rate: about forward, right, down.
        val gyro = Vector3(0.1, 0.2, 0.3)

        val packed = VehicleFrameTransform.packIdrV1(linear, gyro)
        val names = PreprocessingSpec.IDR_V1.channelNames

        assertEquals(6, packed.size)
        assertEquals(names.size, packed.size)
        assertEquals("lin_accel_forward", 1.0f, packed[0], 1e-6f)
        assertEquals("lin_accel_right", 2.0f, packed[1], 1e-6f)
        assertEquals("lin_accel_down", 3.0f, packed[2], 1e-6f)
        assertEquals("gyro_yaw is the rate about Down", 0.3f, packed[3], 1e-6f)
        assertEquals("gyro_pitch is the rate about Right", 0.2f, packed[4], 1e-6f)
        assertEquals("gyro_roll is the rate about Forward", 0.1f, packed[5], 1e-6f)
    }

    @Test
    fun `improper rotation matrices are rejected`() {
        assertFalse("all zeros", VehicleFrameTransform.isProperRotation(FloatArray(9)))
        assertFalse("wrong size", VehicleFrameTransform.isProperRotation(FloatArray(6)))
        assertFalse(
            "reflection has determinant -1",
            VehicleFrameTransform.isProperRotation(
                floatArrayOf(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, -1f)
            )
        )
        assertTrue("identity", VehicleFrameTransform.isProperRotation(Mountings.FLAT_FACING_NORTH))
        assertTrue("landscape", VehicleFrameTransform.isProperRotation(Mountings.LANDSCAPE_DASH_NORTH))
    }

    @Test
    fun `preprocessing spec reports the window span that displacement targets cover`() {
        val spec = PreprocessingSpec.IDR_V1

        assertEquals(2.0, spec.windowSeconds, 1e-9)
        // 20 samples at 10 Hz span 1.9 s between first and last sample, not 2.0 s.
        assertEquals(1.9, spec.windowSpanSeconds, 1e-9)
        assertEquals(5.0, spec.predictionHz, 1e-9)
        assertEquals(6, spec.channelCount)

        // The legacy contract only produces a prediction every 2 s.
        assertEquals(0.5, PreprocessingSpec.LEGACY_V8.predictionHz, 1e-9)
    }

    @Test
    fun `displacement guard rejects the shipped v8 statistics`() {
        // Exactly the values in app/src/main/assets/ml/v8_normalization.json.
        val verdict = DisplacementStatsGuard.check(
            speedMeanMps = 11.62447452545166,
            forwardMeanMeters = -2.996354579925537,
            lateralMeanMeters = 3.694854974746704,
            forwardStdMeters = 19.846437454223633,
            lateralStdMeters = 18.584957122802734,
            windowSpanSeconds = PreprocessingSpec.IDR_V1.windowSpanSeconds
        )

        assertFalse("shipped V8 stats must be flagged", verdict.plausible)
        assertTrue("expected several independent failures", verdict.reasons.size >= 3)
    }

    @Test
    fun `displacement guard accepts physically consistent statistics`() {
        val speedMeanMps = 11.6
        val span = PreprocessingSpec.IDR_V1.windowSpanSeconds

        val verdict = DisplacementStatsGuard.check(
            speedMeanMps = speedMeanMps,
            forwardMeanMeters = speedMeanMps * span,
            lateralMeanMeters = 0.05,
            forwardStdMeters = 8.0,
            lateralStdMeters = 1.2,
            windowSpanSeconds = span
        )

        assertTrue("plausible stats were rejected: ${verdict.reasons}", verdict.plausible)
    }

    @Test
    fun `angular rate rotates like a vector`() {
        // A pure yaw rate about ENU Up must appear entirely on the vehicle yaw channel.
        // Vehicle Down is the negative of ENU Up, so a positive Up rate reads negative.
        val yawRateEnu = Vector3(0.0, 0.0, 0.4)

        Mountings.ALL.forEach { (name, matrix) ->
            val phoneRate = Mountings.enuToPhone(yawRateEnu, matrix)
            val vehicleRate = VehicleFrameTransform.phoneToVehicle(phoneRate, matrix, headingDegrees = 0.0)

            assertEquals("$name roll rate", 0.0, vehicleRate.x, tolerance)
            assertEquals("$name pitch rate", 0.0, vehicleRate.y, tolerance)
            assertEquals("$name yaw rate", -0.4, vehicleRate.z, tolerance)
            assertTrue("$name yaw dominates", abs(vehicleRate.z) > abs(vehicleRate.x) + abs(vehicleRate.y))
        }
    }
}
