package drona.deadreckoning.domain.state

import drona.deadreckoning.core.gnss.GnssQuality

data class GNSSState(
    val isAvailable: Boolean = false,
    val satelliteCount: Int = 0,
    val accuracyMeters: Double = 0.0,
    val latitude: Double = 0.0,
    val longitude: Double = 0.0,
    val speedKmh: Double = 0.0,
    val bearingDegrees: Double = 0.0,
    val signalQualityPercentage: Int = 0,
    val fixStatus: String = "NO FIX",
    val outageDurationSeconds: Long = 0L,
    val hdop: Float = 0.0f,
    val vdop: Float = 0.0f,
    // Stage 4: measurement-derived quality rather than "a callback arrived".
    val quality: GnssQuality = GnssQuality.DENIED,
    val usableForFusion: Boolean = false,
    val effectiveAccuracyMeters: Double = 0.0,
    val fixAgeMillis: Long = 0L,
    val provider: String = "unknown",
    val isFromMockProvider: Boolean = false,
    val satellitesUsedInFix: Int = 0,
    val rejectedFixCount: Int = 0,
    /** Why the monitor reached its current verdict. Empty when nothing is wrong. */
    val qualityReasons: List<String> = emptyList()
)
