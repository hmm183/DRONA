package drona.deadreckoning.data

import android.content.Context
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import drona.deadreckoning.domain.state.AnalyticsState
import drona.deadreckoning.domain.state.NavigationSession

/** Persistent history for completed navigation sessions and their measured metrics. */
class NavigationHistoryStore(private val context: Context) {
    private val preferences = context.getSharedPreferences("navigation_history", Context.MODE_PRIVATE)
    private val gson = Gson()
    private val sessionsType = object : TypeToken<List<NavigationSession>>() {}.type

    fun load(): List<NavigationSession> {
        val storedJson = preferences.getString("sessions", null)
        if (!storedJson.isNullOrBlank() && storedJson != "[]") {
            val parsed = runCatching {
                gson.fromJson<List<NavigationSession>>(storedJson, sessionsType)
            }.getOrNull()
            if (!parsed.isNullOrEmpty()) {
                return parsed
            }
        }
        // Fallback: Load real field test trips from assets
        return loadBundledFieldTrips()
    }

    fun loadBundledFieldTrips(): List<NavigationSession> = runCatching {
        context.assets.open("trips/field_trips.json").bufferedReader().use { reader ->
            gson.fromJson<List<NavigationSession>>(reader, sessionsType) ?: emptyList()
        }
    }.getOrDefault(emptyList())

    fun save(sessions: List<NavigationSession>) {
        preferences.edit().putString("sessions", gson.toJson(sessions.take(25))).apply()
    }

    fun aggregate(sessions: List<NavigationSession>): AnalyticsState {
        if (sessions.isEmpty()) return AnalyticsState()
        val totalDistance = sessions.sumOf { it.displayDistanceKm }
        val totalOutage = sessions.sumOf { if (it.outageDurationSeconds > 0) it.outageDurationSeconds else it.drDurationSeconds }
        val validDurations = sessions.sumOf {
            if (it.durationSeconds > 0) it.durationSeconds else parseDurationSeconds(it.displayDurationString)
        }
        val avgDrift = sessions.map { it.displayDriftMeters }.average()
        val maxDrift = sessions.maxOfOrNull { it.displayDriftMeters } ?: 0.0
        return AnalyticsState(
            totalDistanceKm = totalDistance,
            totalDurationSeconds = validDurations,
            outageCount = sessions.sumOf { if (it.outageCount > 0) it.outageCount else 1 },
            totalOutageDurationSeconds = totalOutage,
            averageDriftMeters = avgDrift,
            maxDriftMeters = maxDrift,
            positionErrorMeters = avgDrift
        )
    }

    private fun parseDurationSeconds(value: String?): Long = value?.split(":")?.let {
        (it.getOrNull(0)?.toLongOrNull() ?: 0L) * 60 + (it.getOrNull(1)?.toLongOrNull() ?: 0L)
    } ?: 0L
}
