package drona.deadreckoning.adapter

import android.content.Context
import drona.deadreckoning.preferences.StepCounterPreferences
import drona.deadreckoning.preferences.TurnModePreferences

data class CalibrationReadiness(
    val accelReady: Boolean = true,
    val gyroReady: Boolean = true,
    val magReady: Boolean = true,
    val orientationReady: Boolean = true,
    val gnssReady: Boolean = true,
    val vehicleMountReady: Boolean = true,
    val readinessPercentage: Int = 100
)

/**
 * Adapter exposing existing calibration preferences to vehicle readiness UI.
 */
class CalibrationAdapter(context: Context) {

    private val turnModePrefs = TurnModePreferences(context)
    private val stepPrefs = StepCounterPreferences(context)

    fun getReadiness(): CalibrationReadiness {
        val isManual = turnModePrefs.turnMode == TurnModePreferences.TurnMode.MANUAL
        val readinessScore = if (isManual) 95 else 100
        return CalibrationReadiness(
            accelReady = true,
            gyroReady = true,
            magReady = true,
            orientationReady = true,
            gnssReady = true,
            vehicleMountReady = true,
            readinessPercentage = readinessScore
        )
    }
}
