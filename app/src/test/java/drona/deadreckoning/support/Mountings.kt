package drona.deadreckoning.support

import kotlin.math.cos
import kotlin.math.sin
import drona.deadreckoning.core.frame.Vector3
import drona.deadreckoning.core.frame.VehicleFrameTransform

/**
 * Device-to-ENU rotation matrices for representative phone mountings, plus helpers to
 * synthesise the accelerometer reading a phone would report for a known physical
 * vehicle motion.
 *
 * Each matrix is expressed by the ENU images of the three phone axes, which are the
 * matrix columns, since `v_enu = R . v_phone`.
 */
object Mountings {

    /** Flat on a horizontal surface, top of screen pointing North. */
    val FLAT_FACING_NORTH = rowMajor(
        xAxisEnu = Vector3(1.0, 0.0, 0.0),
        yAxisEnu = Vector3(0.0, 1.0, 0.0),
        zAxisEnu = Vector3(0.0, 0.0, 1.0)
    )

    /** Upright in a dash cradle, portrait, screen facing the driver, vehicle heading North. */
    val PORTRAIT_DASH_NORTH = rowMajor(
        xAxisEnu = Vector3(1.0, 0.0, 0.0),
        yAxisEnu = Vector3(0.0, 0.0, 1.0),
        zAxisEnu = Vector3(0.0, -1.0, 0.0)
    )

    /** Same cradle rotated 90 degrees into landscape, vehicle heading North. */
    val LANDSCAPE_DASH_NORTH = rowMajor(
        xAxisEnu = Vector3(0.0, 0.0, 1.0),
        yAxisEnu = Vector3(-1.0, 0.0, 0.0),
        zAxisEnu = Vector3(0.0, -1.0, 0.0)
    )

    /** Flat but yawed 90 degrees, top of screen pointing East. */
    val FLAT_FACING_EAST = rowMajor(
        xAxisEnu = Vector3(0.0, -1.0, 0.0),
        yAxisEnu = Vector3(1.0, 0.0, 0.0),
        zAxisEnu = Vector3(0.0, 0.0, 1.0)
    )

    /** Portrait cradle leaning back 20 degrees, a very common real mounting. */
    val PORTRAIT_TILTED_BACK_20 = multiply(PORTRAIT_DASH_NORTH, rotationAboutX(Math.toRadians(20.0)))

    val ALL: List<Pair<String, FloatArray>> = listOf(
        "flat facing north" to FLAT_FACING_NORTH,
        "portrait dash" to PORTRAIT_DASH_NORTH,
        "landscape dash" to LANDSCAPE_DASH_NORTH,
        "flat facing east" to FLAT_FACING_EAST,
        "portrait tilted back 20deg" to PORTRAIT_TILTED_BACK_20
    )

    /**
     * A proper rotation built from intrinsic Z-Y-X angles. Any combination yields a
     * valid device attitude, which makes randomised mount-invariance testing possible.
     */
    fun arbitraryRotation(yaw: Double, pitch: Double, roll: Double): FloatArray =
        multiply(multiply(rotationAboutZ(yaw), rotationAboutY(pitch)), rotationAboutX(roll))

    /**
     * The accelerometer reading, in phone axes, for a vehicle undergoing [trueAccelEnu]
     * while the phone sits at attitude [rotationMatrix].
     *
     * An accelerometer measures specific force, so gravity appears as `+g` along Up:
     * at rest the device reads `(0, 0, +9.80665)` in ENU.
     */
    fun accelerometerReadingInPhoneFrame(trueAccelEnu: Vector3, rotationMatrix: FloatArray): Vector3 {
        val measuredEnu = Vector3(trueAccelEnu.x, trueAccelEnu.y, trueAccelEnu.z + VehicleFrameTransform.GRAVITY)
        return enuToPhone(measuredEnu, rotationMatrix)
    }

    /** Inverse of the device-to-ENU rotation. For a rotation the inverse is the transpose. */
    fun enuToPhone(vectorEnu: Vector3, rotationMatrix: FloatArray): Vector3 = Vector3(
        rotationMatrix[0] * vectorEnu.x + rotationMatrix[3] * vectorEnu.y + rotationMatrix[6] * vectorEnu.z,
        rotationMatrix[1] * vectorEnu.x + rotationMatrix[4] * vectorEnu.y + rotationMatrix[7] * vectorEnu.z,
        rotationMatrix[2] * vectorEnu.x + rotationMatrix[5] * vectorEnu.y + rotationMatrix[8] * vectorEnu.z
    )

    private fun rowMajor(xAxisEnu: Vector3, yAxisEnu: Vector3, zAxisEnu: Vector3) = floatArrayOf(
        xAxisEnu.x.toFloat(), yAxisEnu.x.toFloat(), zAxisEnu.x.toFloat(),
        xAxisEnu.y.toFloat(), yAxisEnu.y.toFloat(), zAxisEnu.y.toFloat(),
        xAxisEnu.z.toFloat(), yAxisEnu.z.toFloat(), zAxisEnu.z.toFloat()
    )

    private fun rotationAboutX(angle: Double) = floatArrayOf(
        1f, 0f, 0f,
        0f, cos(angle).toFloat(), (-sin(angle)).toFloat(),
        0f, sin(angle).toFloat(), cos(angle).toFloat()
    )

    private fun rotationAboutY(angle: Double) = floatArrayOf(
        cos(angle).toFloat(), 0f, sin(angle).toFloat(),
        0f, 1f, 0f,
        (-sin(angle)).toFloat(), 0f, cos(angle).toFloat()
    )

    private fun rotationAboutZ(angle: Double) = floatArrayOf(
        cos(angle).toFloat(), (-sin(angle)).toFloat(), 0f,
        sin(angle).toFloat(), cos(angle).toFloat(), 0f,
        0f, 0f, 1f
    )

    private fun multiply(left: FloatArray, right: FloatArray): FloatArray {
        val product = FloatArray(9)
        for (row in 0..2) {
            for (column in 0..2) {
                var sum = 0f
                for (index in 0..2) sum += left[row * 3 + index] * right[index * 3 + column]
                product[row * 3 + column] = sum
            }
        }
        return product
    }
}
