package drona.deadreckoning.simulation

import org.osmdroid.util.GeoPoint

/**
 * Represents a single GNSS outage interval as [start, end] progress fractions (0.0 to 1.0).
 */
data class OutageInterval(
    val startPct: Float,
    val endPct: Float
) {
    fun contains(progressFraction: Double): Boolean =
        progressFraction >= startPct && progressFraction <= endPct
}

/**
 * Configuration parameters for the GNSS-Denied Intelligent Dead Reckoning Simulation.
 * Deterministic by default via fixed [randomSeed].
 */
data class SimulationConfig(
    val randomSeed: Long = 26168L,
    val simulationSpeedMultiplier: Float = 1.0f,
    val blackoutStartPct: Float = 0.04f,
    val blackoutEndPct: Float = 0.08f,
    val stepFrequencyHz: Double = 10.0,
    val vehicleSpeedKmh: Double = 42.0,

    // Multiple outage intervals (empty by default so custom blackoutStartPct..blackoutEndPct works as expected)
    val outageIntervals: List<OutageInterval> = emptyList(),
    
    // Sensor error parameters (realistic automotive grade MEMS)
    val accelNoiseStdMps2: Double = 0.12,
    val gyroBiasDps: Double = 1.25, // Uncompensated angular drift causing naive divergence
    val gyroNoiseStdDps: Double = 0.25,
    val gnssNoiseStdMeters: Double = 2.2,
    val aiSpeedNoiseStdMps: Double = 0.35,

    // Default route: Mandadam -> VIT-AP (Double Turn road corridor)
    val sourcePoint: GeoPoint = GeoPoint(16.5190, 80.5520),
    val destinationPoint: GeoPoint = GeoPoint(16.4975, 80.5005),
    val routeName: String = "Mandadam ↔ VIT-AP (Double Turn)"
) {
    companion object {
        val DEFAULT_MULTI_OUTAGES = listOf(
            OutageInterval(0.04f, 0.08f),
            OutageInterval(0.12f, 0.16f)
        )
    }

    val dtSeconds: Double get() = 1.0 / stepFrequencyHz

    /** All outage intervals — uses multi-outage list if provided, else falls back to single blackout */
    val effectiveOutages: List<OutageInterval>
        get() = if (outageIntervals.isNotEmpty()) outageIntervals
                else listOf(OutageInterval(blackoutStartPct, blackoutEndPct))

    /** Check if a given progress fraction is inside any outage window */
    fun isInBlackout(progressFraction: Double): Boolean =
        effectiveOutages.any { it.contains(progressFraction) }
}

