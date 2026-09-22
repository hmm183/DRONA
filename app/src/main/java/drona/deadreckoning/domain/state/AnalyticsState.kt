package drona.deadreckoning.domain.state

data class AnalyticsState(
    val totalDistanceKm: Double = 0.0,
    val totalDurationSeconds: Long = 0L,
    val outageCount: Int = 0,
    val totalOutageDurationSeconds: Long = 0L,
    val averageDriftMeters: Double = 0.0,
    val maxDriftMeters: Double = 0.0,
    val positionErrorMeters: Double = 0.0,
    val speedErrorKmh: Double = 0.0,
    val headingErrorDegrees: Double = 0.0,
    val aiSpeedRmseKmh: Double = 0.0,
    val mapMatchingAccuracyPercentage: Int = 0,
    val gnssRecoveryTimeSeconds: Double = 0.0
)
