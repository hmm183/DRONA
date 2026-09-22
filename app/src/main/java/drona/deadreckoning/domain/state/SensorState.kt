package drona.deadreckoning.domain.state

data class SensorState(
    val accelX: Float = 0f,
    val accelY: Float = 0f,
    val accelZ: Float = 0f,
    val accelMagnitude: Float = 0f,
    val gyroX: Float = 0f,
    val gyroY: Float = 0f,
    val gyroZ: Float = 0f,
    val gyroBiasX: Float = 0f,
    val gyroBiasY: Float = 0f,
    val gyroBiasZ: Float = 0f,
    val magX: Float = 0f,
    val magY: Float = 0f,
    val magZ: Float = 0f,
    val rollDegrees: Float = 0f,
    val pitchDegrees: Float = 0f,
    val yawDegrees: Float = 0f,
    val vehicleHeadingDegrees: Float = 0f,
    val yawAlignmentOffsetDegrees: Float = 0f,
    val mountStabilityPercentage: Int = 0,
    val alignmentConfidencePercentage: Int = 0,
    // Stage 1: vehicle-frame (FRD) linear acceleration and angular rate.
    // Gravity removed, mount independent. Populated only when a valid device
    // attitude matrix and a usable vehicle heading are both available.
    val vehicleAccelForward: Float = 0f,
    val vehicleAccelRight: Float = 0f,
    val vehicleAccelDown: Float = 0f,
    val vehicleGyroYaw: Float = 0f,
    val vehicleGyroPitch: Float = 0f,
    val vehicleGyroRoll: Float = 0f,
    val isVehicleFrameValid: Boolean = false,
    val isStationary: Boolean = false,
    val isMountChanged: Boolean = false,
    val usesRotationVector: Boolean = false,
    val imuSamplingHz: Int = 0,
    val overallHealthPercentage: Int = 0,
    val isAccelAvailable: Boolean = true,
    val isGyroAvailable: Boolean = true,
    val isMagAvailable: Boolean = true
)
