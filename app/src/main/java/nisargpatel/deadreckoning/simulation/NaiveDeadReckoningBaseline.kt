package nisargpatel.deadreckoning.simulation

import org.osmdroid.util.GeoPoint
import kotlin.math.cos
import kotlin.math.sin

/**
 * Baseline: Unconstrained Naive IMU Dead Reckoning.
 * Integrates noisy displacement and uncompensated gyroscope heading delta without
 * kinematic filtering, non-holonomic constraints, or road geometry matching.
 */
class NaiveDeadReckoningBaseline {

    var currentPosition: GeoPoint? = null
        private set

    var currentHeadingDegrees: Double = 0.0
        private set

    var currentSpeedMps: Double = 0.0
        private set

    private var peMeters: Double = 0.0
    private var pnMeters: Double = 0.0
    private var origin: GeoPoint? = null

    fun reset(initialPosition: GeoPoint, initialHeadingDegrees: Double, initialSpeedMps: Double) {
        origin = initialPosition
        currentPosition = initialPosition
        currentHeadingDegrees = initialHeadingDegrees
        currentSpeedMps = initialSpeedMps
        peMeters = 0.0
        pnMeters = 0.0
    }

    fun step(imuStep: SyntheticImuStep, gnssObs: SyntheticGnssObservation) {
        val orig = origin ?: return

        // When GNSS is healthy, re-anchor naive DR to GNSS fix
        if (!gnssObs.isBlackout && gnssObs.position != null) {
            currentPosition = gnssObs.position
            currentHeadingDegrees = gnssObs.fix?.bearingDegrees?.toDouble() ?: currentHeadingDegrees
            currentSpeedMps = gnssObs.fix?.speedMps ?: currentSpeedMps

            val latScale = 111_132.95
            val lonScale = latScale * cos(Math.toRadians(orig.latitude))
            peMeters = (gnssObs.position.longitude - orig.longitude) * lonScale
            pnMeters = (gnssObs.position.latitude - orig.latitude) * latScale
            return
        }

        // During GNSS blackout, integrate raw noisy heading and forward distance
        val headingRad = Math.toRadians(currentHeadingDegrees)
        currentHeadingDegrees = (currentHeadingDegrees + Math.toDegrees(imuStep.headingDeltaRadians) + 360.0) % 360.0

        peMeters += imuStep.forwardMeters * sin(headingRad)
        pnMeters += imuStep.forwardMeters * cos(headingRad)
        currentSpeedMps = imuStep.forwardMeters / imuStep.intervalSeconds

        val latScale = 111_132.95
        val lonScale = latScale * cos(Math.toRadians(orig.latitude))

        val lat = orig.latitude + (pnMeters / latScale)
        val lon = orig.longitude + (peMeters / lonScale)
        currentPosition = GeoPoint(lat, lon)
    }
}
