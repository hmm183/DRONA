package nisargpatel.deadreckoning.simulation

import nisargpatel.deadreckoning.fusion.VehicleFusionImmUkf
import org.osmdroid.util.GeoPoint
import kotlin.math.*

data class GroundTruthState(
    val stepIndex: Int,
    val timestampSeconds: Double,
    val position: GeoPoint,
    val pe: Double,
    val pn: Double,
    val speedMps: Double,
    val headingDegrees: Double,
    val yawRateDps: Double,
    val accelMps2: Double,
    val distanceMeters: Double,
    val progressFraction: Double,
    val motionMode: VehicleFusionImmUkf.MotionMode
)

object GroundTruthTrajectoryGenerator {

    /**
     * Converts a route of geographic waypoints into a dense, smooth sequence
     * of physical vehicle ground-truth states at a uniform sample rate [dtSeconds].
     */
    fun generate(
        routePoints: List<GeoPoint>,
        targetSpeedMps: Double,
        dtSeconds: Double
    ): List<GroundTruthState> {
        if (routePoints.size < 2) return emptyList()

        val origin = routePoints.first()
        val latScale = 111_132.95
        val lonScale = latScale * cos(Math.toRadians(origin.latitude))

        // Precompute cumulative arc lengths
        val segmentDistances = mutableListOf<Double>()
        var totalDistance = 0.0
        for (i in 0 until routePoints.size - 1) {
            val d = routePoints[i].distanceToAsDouble(routePoints[i + 1])
            segmentDistances.add(d)
            totalDistance += d
        }

        if (totalDistance <= 0.0) return emptyList()

        val states = mutableListOf<GroundTruthState>()
        var currentDist = 0.0
        var currentSec = 0.0
        var step = 0
        var lastHeading = 0.0
        var lastSpeed = targetSpeedMps

        val stepDistance = (targetSpeedMps * dtSeconds).coerceAtLeast(0.1)

        while (currentDist <= totalDistance) {
            // Find which segment we are on
            var accumulated = 0.0
            var segIdx = 0
            while (segIdx < segmentDistances.size - 1 && accumulated + segmentDistances[segIdx] < currentDist) {
                accumulated += segmentDistances[segIdx]
                segIdx++
            }

            val segDist = segmentDistances[segIdx].coerceAtLeast(0.001)
            val segFraction = ((currentDist - accumulated) / segDist).coerceIn(0.0, 1.0)
            val pA = routePoints[segIdx]
            val pB = routePoints[segIdx + 1]

            // Interpolate position
            val lat = pA.latitude + (pB.latitude - pA.latitude) * segFraction
            val lon = pA.longitude + (pB.longitude - pA.longitude) * segFraction
            val pos = GeoPoint(lat, lon)

            // Local ENU coordinates
            val pe = (lon - origin.longitude) * lonScale
            val pn = (lat - origin.latitude) * latScale

            // Compute true heading along segment
            val segHeading = normalizeDegrees(pA.bearingTo(pB).toDouble())
            val heading = if (step == 0) segHeading else {
                // Smooth heading transition across corners
                interpolateHeading(lastHeading, segHeading, 0.35)
            }

            // Compute yaw rate
            val headingDelta = normalizeAngleDelta(heading - lastHeading)
            val yawRateDps = if (step == 0) 0.0 else headingDelta / dtSeconds

            // Motion mode classification based on kinematics
            val motionMode = when {
                abs(yawRateDps) > 2.5 -> VehicleFusionImmUkf.MotionMode.CONSTANT_TURN_RATE
                abs(targetSpeedMps - lastSpeed) > 0.5 -> VehicleFusionImmUkf.MotionMode.CONSTANT_ACCELERATION
                else -> VehicleFusionImmUkf.MotionMode.CONSTANT_VELOCITY
            }

            val accel = (targetSpeedMps - lastSpeed) / dtSeconds

            states.add(
                GroundTruthState(
                    stepIndex = step,
                    timestampSeconds = currentSec,
                    position = pos,
                    pe = pe,
                    pn = pn,
                    speedMps = targetSpeedMps,
                    headingDegrees = heading,
                    yawRateDps = yawRateDps,
                    accelMps2 = accel,
                    distanceMeters = currentDist,
                    progressFraction = (currentDist / totalDistance).coerceIn(0.0, 1.0),
                    motionMode = motionMode
                )
            )

            lastHeading = heading
            lastSpeed = targetSpeedMps
            currentDist += stepDistance
            currentSec += dtSeconds
            step++
        }

        return states
    }

    private fun normalizeDegrees(deg: Double): Double {
        var d = deg % 360.0
        if (d < 0.0) d += 360.0
        return d
    }

    private fun normalizeAngleDelta(delta: Double): Double {
        var d = delta
        while (d > 180.0) d -= 360.0
        while (d < -180.0) d += 360.0
        return d
    }

    private fun interpolateHeading(from: Double, to: Double, alpha: Double): Double {
        val diff = normalizeAngleDelta(to - from)
        return normalizeDegrees(from + diff * alpha)
    }
}
