package drona.deadreckoning.domain.state

sealed class NavigationEvent {
    object GNSSLost : NavigationEvent()
    object GNSSRestored : NavigationEvent()
    data class PotholeDetected(val severity: String = "Moderate") : NavigationEvent()
    object PhoneMisaligned : NavigationEvent()
    object CalibrationRequired : NavigationEvent()
    object MapUnavailable : NavigationEvent()
    object SensorUnavailable : NavigationEvent()
    object NavigationStarted : NavigationEvent()
    object NavigationStopped : NavigationEvent()
    data class NavigationError(val message: String) : NavigationEvent()
}
