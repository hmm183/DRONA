package drona.deadreckoning.domain.model

enum class NavigationMode(val displayName: String, val description: String) {
    GNSS_INS("GNSS + INS", "Normal multi-sensor navigation"),
    AI_DEAD_RECKONING("AI DEAD RECKONING", "GNSS lost - IMU & AI active"),
    GNSS_RECOVERY("GNSS RECOVERY", "Reconciling GNSS and DR position"),
    OFFLINE("OFFLINE", "Offline map-matched navigation"),
    CALIBRATION("CALIBRATION", "Initializing sensors & mount"),
    ERROR("ERROR", "Navigation system issue")
}
