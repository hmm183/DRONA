package drona.deadreckoning.core.frame

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** Minimal 3-vector. Pure Kotlin so the navigation core stays portable. */
data class Vector3(val x: Double, val y: Double, val z: Double) {
    val norm: Double get() = sqrt(x * x + y * y + z * z)
    operator fun minus(other: Vector3) = Vector3(x - other.x, y - other.y, z - other.z)
    operator fun plus(other: Vector3) = Vector3(x + other.x, y + other.y, z + other.z)

    companion object {
        val ZERO = Vector3(0.0, 0.0, 0.0)
    }
}

/**
 * Rotates IMU measurements out of the orientation-dependent phone frame into the
 * vehicle body frame.
 *
 * Previously nothing in this project rotated IMU vectors at all. The alignment
 * calibrator learned a yaw offset but applied it only to the heading shown in the
 * UI, so the model received raw device axes and its interpretation of "forward"
 * changed whenever the phone was re-mounted, rotated or tilted.
 *
 * ## Frames
 *
 * Phone (Android device): X right across the screen, Y up the screen, Z out of the
 * screen toward the user.
 *
 * ENU world: X East, Y North, Z Up. This is the frame Android's rotation matrix maps
 * into, via `SensorManager.getRotationMatrixFromVector`.
 *
 * Vehicle FRD body frame: X forward, Y right, Z down. Right-handed.
 *
 * ## Strategy
 *
 *     v_vehicle = M(heading) . R_enu_from_phone . v_phone
 *
 * `R_enu_from_phone` comes from Android's own attitude estimator, which already
 * fuses accelerometer, gyroscope and magnetometer. That reduces the unknown to a
 * single scalar, the vehicle heading, which the alignment calibrator learns from
 * GNSS course while moving. Deriving roll and pitch independently would duplicate
 * work Android already does well.
 *
 * ## Convention check
 *
 * [enuToVehicle] reduces to exactly the projection the IO-VNBD training pipeline
 * uses in `global_to_vehicle_frame`:
 *
 *     forward = dEast * sin(heading) + dNorth * cos(heading)
 *     right   = dEast * cos(heading) - dNorth * sin(heading)
 *
 * so runtime and training agree on what "forward" and "lateral" mean.
 */
object VehicleFrameTransform {

    /** Standard gravity, m/s^2. */
    const val GRAVITY = 9.80665

    /**
     * Rotate a phone-frame vector into ENU.
     *
     * @param rotationMatrix row-major 3x3 (9 elements) or 4x4 (16 elements) matrix as
     *   produced by `SensorManager.getRotationMatrixFromVector`, mapping device to world.
     */
    fun phoneToEnu(vector: Vector3, rotationMatrix: FloatArray): Vector3 {
        val stride = when (rotationMatrix.size) {
            9 -> 3
            16 -> 4
            else -> error("Rotation matrix must have 9 or 16 elements, got ${rotationMatrix.size}")
        }
        fun row(index: Int): Vector3 {
            val base = index * stride
            return Vector3(
                rotationMatrix[base].toDouble(),
                rotationMatrix[base + 1].toDouble(),
                rotationMatrix[base + 2].toDouble()
            )
        }
        fun dot(a: Vector3, b: Vector3) = a.x * b.x + a.y * b.y + a.z * b.z
        return Vector3(dot(row(0), vector), dot(row(1), vector), dot(row(2), vector))
    }

    /**
     * Rotate an ENU vector into the vehicle FRD frame.
     *
     * @param headingDegrees vehicle travel direction, 0 = North, 90 = East.
     */
    fun enuToVehicle(vector: Vector3, headingDegrees: Double): Vector3 {
        val heading = Math.toRadians(headingDegrees)
        val sinHeading = sin(heading)
        val cosHeading = cos(heading)
        return Vector3(
            x = vector.x * sinHeading + vector.y * cosHeading,
            y = vector.x * cosHeading - vector.y * sinHeading,
            z = -vector.z
        )
    }

    /** Full phone to vehicle rotation. */
    fun phoneToVehicle(vector: Vector3, rotationMatrix: FloatArray, headingDegrees: Double): Vector3 =
        enuToVehicle(phoneToEnu(vector, rotationMatrix), headingDegrees)

    /**
     * Remove the gravity component from a vehicle-frame accelerometer reading.
     *
     * An accelerometer measures specific force, so at rest it reads `-g` along the
     * body Down axis, that is `(0, 0, -9.80665)` in FRD. True acceleration is
     * therefore the reading plus the gravity vector `(0, 0, +9.80665)`.
     */
    fun removeGravity(accelVehicle: Vector3): Vector3 =
        Vector3(accelVehicle.x, accelVehicle.y, accelVehicle.z + GRAVITY)

    /**
     * Pack vehicle-frame measurements into the six channels of
     * [drona.deadreckoning.core.spec.PreprocessingSpec.IDR_V1], in order:
     * forward, right, down linear acceleration, then yaw, pitch, roll rate.
     *
     * Yaw rate is the rate about Down, pitch about Right, roll about Forward, which
     * is why the gyro components appear reversed relative to the acceleration ones.
     */
    fun packIdrV1(linearAccelVehicle: Vector3, gyroVehicle: Vector3): FloatArray = floatArrayOf(
        linearAccelVehicle.x.toFloat(),
        linearAccelVehicle.y.toFloat(),
        linearAccelVehicle.z.toFloat(),
        gyroVehicle.z.toFloat(),
        gyroVehicle.y.toFloat(),
        gyroVehicle.x.toFloat()
    )

    /**
     * True when a matrix is a proper rotation: orthonormal rows and determinant +1.
     * Guards against feeding a stale, zeroed or uninitialised matrix into the pipeline.
     */
    fun isProperRotation(rotationMatrix: FloatArray, tolerance: Double = 1e-3): Boolean {
        val stride = when (rotationMatrix.size) {
            9 -> 3
            16 -> 4
            else -> return false
        }
        val rows = (0..2).map { index ->
            val base = index * stride
            Vector3(
                rotationMatrix[base].toDouble(),
                rotationMatrix[base + 1].toDouble(),
                rotationMatrix[base + 2].toDouble()
            )
        }
        fun dot(a: Vector3, b: Vector3) = a.x * b.x + a.y * b.y + a.z * b.z
        rows.forEach { if (abs(it.norm - 1.0) > tolerance) return false }
        if (abs(dot(rows[0], rows[1])) > tolerance) return false
        if (abs(dot(rows[0], rows[2])) > tolerance) return false
        if (abs(dot(rows[1], rows[2])) > tolerance) return false
        val determinant =
            rows[0].x * (rows[1].y * rows[2].z - rows[1].z * rows[2].y) -
                rows[0].y * (rows[1].x * rows[2].z - rows[1].z * rows[2].x) +
                rows[0].z * (rows[1].x * rows[2].y - rows[1].y * rows[2].x)
        return abs(determinant - 1.0) <= tolerance
    }
}
