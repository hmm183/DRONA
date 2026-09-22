package drona.deadreckoning.imu

/** Sensor-neutral contract shared by the phone adapter and future BLE/USB edge IMUs. */
data class ImuSample(
    val timestampNs: Long,
    val accelX: Float,
    val accelY: Float,
    val accelZ: Float,
    val gyroX: Float,
    val gyroY: Float,
    val gyroZ: Float,
    val magnetometerX: Float? = null,
    val magnetometerY: Float? = null,
    val magnetometerZ: Float? = null,
    val source: ImuSource = ImuSource.PHONE
)

enum class ImuSource { PHONE, BLE, USB, FILE_REPLAY }

interface ImuSourceAdapter {
    val source: ImuSource
    fun start(onSample: (ImuSample) -> Unit)
    fun stop()
}
