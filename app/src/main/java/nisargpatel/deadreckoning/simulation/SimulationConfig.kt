package nisargpatel.deadreckoning.simulation

import org.osmdroid.util.GeoPoint

/**
 * Configuration parameters for the GNSS-Denied Intelligent Dead Reckoning Simulation.
 * Deterministic by default via fixed [randomSeed].
 */
data class SimulationConfig(
    val randomSeed: Long = 26168L,
    val simulationSpeedMultiplier: Float = 1.0f,
    val blackoutStartPct: Float = 0.35f,
    val blackoutEndPct: Float = 0.70f,
    val stepFrequencyHz: Double = 10.0,
    val vehicleSpeedKmh: Double = 42.0,
    
    // Sensor error parameters (realistic automotive grade MEMS)
    val accelNoiseStdMps2: Double = 0.12,
    val gyroBiasDps: Double = 1.25, // Uncompensated angular drift causing naive divergence
    val gyroNoiseStdDps: Double = 0.25,
    val gnssNoiseStdMeters: Double = 2.2,
    val aiSpeedNoiseStdMps: Double = 0.35,

    // Default route: Mandadam -> Vijayawada
    val sourcePoint: GeoPoint = GeoPoint(16.5160, 80.5780),
    val destinationPoint: GeoPoint = GeoPoint(16.5062, 80.6480),
    val routeName: String = "Mandadam to Vijayawada"
) {
    val dtSeconds: Double get() = 1.0 / stepFrequencyHz
}
