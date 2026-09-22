package drona.deadreckoning.domain.state

import com.google.gson.annotations.SerializedName

data class RouteEndpoints(
    @SerializedName("start") val start: String? = "",
    @SerializedName("end") val end: String? = ""
)

data class LatLngPoint(
    val latitude: Double,
    val longitude: Double
)

data class NavigationSession(
    @SerializedName("trip_id") val id: String? = "",
    @SerializedName("dateString") val dateString: String? = "",
    @SerializedName("durationString") val durationString: String? = "",
    @SerializedName("distanceKm") val distanceKm: Double = 0.0,
    @SerializedName("outageCount") val outageCount: Int = 0,
    @SerializedName("drDurationSeconds") val drDurationSeconds: Long = 0L,
    @SerializedName("maxErrorMeters") val maxErrorMeters: Double = 0.0,
    @SerializedName("avgErrorMeters") val avgErrorMeters: Double = 0.0,
    @SerializedName("status") val status: String? = "COMPLETED",

    // Phase 4 Schema Additions:
    @SerializedName("label") val label: String? = "",
    @SerializedName("source") val source: String? = "Field test — own vehicle",
    @SerializedName("route_endpoints") val routeEndpoints: RouteEndpoints? = null,
    @SerializedName("distance_m") val distanceMeters: Double = distanceKm * 1000.0,
    @SerializedName("duration_s") val durationSeconds: Long = 0L,
    @SerializedName("avg_speed_mps") val avgSpeedMps: Double = 0.0,
    @SerializedName("max_speed_mps") val maxSpeedMps: Double = 0.0,
    @SerializedName("outage_start_s") val outageStartSeconds: Long = 0L,
    @SerializedName("outage_duration_s") val outageDurationSeconds: Long = drDurationSeconds,
    @SerializedName("final_drift_m") val finalDriftMeters: Double = maxErrorMeters,
    @SerializedName("drift_pct_of_distance") val driftPctOfDistance: Double = 0.0,
    @SerializedName("target_threshold_pct") val targetThresholdPct: Double = 10.0,
    @SerializedName("meets_target") val meetsTarget: Boolean = true,
    @SerializedName("path_gnss") val rawPathGnss: List<List<Double>>? = null,
    @SerializedName("path_dr_estimate") val rawPathDrEstimate: List<List<Double>>? = null,
    @SerializedName("path_reference_actual") val rawPathReferenceActual: List<List<Double>>? = null
) {
    val sessionId: String
        get() = if (!id.isNullOrBlank()) id else "session_${System.currentTimeMillis()}"

    val sessionStatus: String
        get() = if (!status.isNullOrBlank()) status else "COMPLETED"

    val sessionSource: String
        get() = if (!source.isNullOrBlank()) source else "Field test — own vehicle"

    val sessionDateString: String
        get() = if (!dateString.isNullOrBlank()) dateString else ""

    val pathGnss: List<LatLngPoint>
        get() = rawPathGnss?.mapNotNull { if (it.size >= 2) LatLngPoint(it[0], it[1]) else null } ?: emptyList()

    val pathDrEstimate: List<LatLngPoint>
        get() = rawPathDrEstimate?.mapNotNull { if (it.size >= 2) LatLngPoint(it[0], it[1]) else null } ?: emptyList()

    val pathReferenceActual: List<LatLngPoint>
        get() = rawPathReferenceActual?.mapNotNull { if (it.size >= 2) LatLngPoint(it[0], it[1]) else null } ?: emptyList()

    val displayTitle: String
        get() = when {
            !label.isNullOrBlank() -> label
            routeEndpoints != null && !routeEndpoints.start.isNullOrBlank() -> "${routeEndpoints.start} → ${routeEndpoints.end}"
            else -> "Trip #${id ?: "1"}"
        }

    val displayDistanceKm: Double
        get() = if (distanceKm > 0.0) distanceKm else (distanceMeters / 1000.0)

    val displayDurationString: String
        get() = if (!durationString.isNullOrBlank()) durationString else {
            val mins = durationSeconds / 60
            val secs = durationSeconds % 60
            "%02d:%02d".format(mins, secs)
        }

    val displayDriftMeters: Double
        get() = if (finalDriftMeters > 0.0) finalDriftMeters else maxErrorMeters

    val displayDriftPct: Double
        get() = if (driftPctOfDistance > 0.0) driftPctOfDistance else if (displayDistanceKm > 0.0) (displayDriftMeters / (displayDistanceKm * 1000.0) * 100.0) else 0.0
}

data class SessionState(
    val sessions: List<NavigationSession> = emptyList(),
    val selectedSession: NavigationSession? = null,
    val isLoading: Boolean = false
)
