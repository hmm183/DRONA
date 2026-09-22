package drona.deadreckoning.domain.state

import drona.deadreckoning.domain.model.NavigationMode

data class NavigationState(
    val mode: NavigationMode = NavigationMode.GNSS_INS,
    val speedKmh: Double = 0.0,
    val headingDegrees: Double = 0.0,
    val accuracyMeters: Double = 0.0,
    val latitude: Double = 0.0,
    val longitude: Double = 0.0,
    val totalDistanceKm: Double = 0.0,
    val outageDurationSeconds: Long = 0L,
    /**
     * Navigation confidence built from estimator covariance, GNSS quality and model
     * uncertainty. Never the motion-class softmax probability, which says nothing about
     * positional trust.
     */
    val confidencePercentage: Int = 100,
    // Stage 6: real per-axis uncertainty from the position covariance.
    val alongTrackUncertaintyMeters: Double = 0.0,
    val crossTrackUncertaintyMeters: Double = 0.0,
    val speedUncertaintyKmh: Double = 0.0,
    val headingUncertaintyDegrees: Double = 0.0,
    val isNavigating: Boolean = false,
    val isDemoMode: Boolean = false
)
