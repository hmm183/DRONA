package drona.deadreckoning.util

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.sqrt
import org.osmdroid.util.GeoPoint

/**
 * @param bearingDegrees direction of the matched road segment, 0 = North. Null when it
 *   could not be determined. The estimator needs this because a road match constrains
 *   position across the road but carries almost no information along it.
 */
data class RouteMatch(
    val point: GeoPoint,
    val distanceMeters: Double,
    val confidence: Int,
    val bearingDegrees: Double? = null,
    val segmentIndex: Int = 0
)

/** Projects a dead-reckoned point onto the active, locally retained route geometry. */
object RouteMapMatcher {
    fun match(
        point: GeoPoint,
        route: List<GeoPoint>,
        headingDegrees: Double? = null,
        preferredSegmentIndex: Int? = null
    ): RouteMatch? {
        if (route.size < 2) return null
        var bestScore = Double.MAX_VALUE
        var bestDistance = Double.MAX_VALUE
        var bestCandidate: GeoPoint? = null
        var bestBearing: Double? = null
        var bestIndex = 0
        var bestFraction = 0.5

        // Wider localized search window [pref-2, pref+8] to handle consecutive turns
        val segmentIndices = if (preferredSegmentIndex != null && preferredSegmentIndex in 0 until route.size - 1) {
            val minIdx = (preferredSegmentIndex - 2).coerceAtLeast(0)
            val maxIdx = (preferredSegmentIndex + 8).coerceAtMost(route.size - 2)
            (minIdx..maxIdx).toList()
        } else {
            (0 until route.size - 1).toList()
        }

        for (i in segmentIndices) {
            val result = scoreSegment(point, route, i, headingDegrees, preferredSegmentIndex)
            if (result.score < bestScore) {
                bestScore = result.score
                bestDistance = result.distance
                bestCandidate = result.candidate
                bestBearing = result.bearing
                bestIndex = i
                bestFraction = result.fraction
            }
        }

        // Global fallback if localized candidate was too far (> 50m)
        if ((bestCandidate == null || bestDistance > 50.0) && preferredSegmentIndex != null) {
            for (i in 0 until route.size - 1) {
                val result = scoreSegment(point, route, i, headingDegrees, null)
                if (result.score < bestScore) {
                    bestScore = result.score
                    bestDistance = result.distance
                    bestCandidate = result.candidate
                    bestBearing = result.bearing
                    bestIndex = i
                    bestFraction = result.fraction
                }
            }
        }

        val matched = bestCandidate ?: return null

        // Bearing interpolation: blend bearing from adjacent segments at segment boundaries
        val interpolatedBearing = interpolateBearing(route, bestIndex, bestFraction, bestBearing)

        return RouteMatch(
            point = matched,
            distanceMeters = bestDistance,
            confidence = (100.0 - bestDistance * 3.0).toInt().coerceIn(0, 100),
            bearingDegrees = interpolatedBearing,
            segmentIndex = bestIndex
        )
    }

    private data class SegmentScore(
        val score: Double,
        val distance: Double,
        val candidate: GeoPoint,
        val bearing: Double,
        val fraction: Double
    )

    private fun scoreSegment(
        point: GeoPoint,
        route: List<GeoPoint>,
        segIndex: Int,
        headingDegrees: Double?,
        preferredSegmentIndex: Int?
    ): SegmentScore {
        val start = route[segIndex]
        val end = route[segIndex + 1]
        val (candidate, fraction) = projectWithFraction(point, start, end)
        val distance = point.distanceToAsDouble(candidate)
        val segBearing = normalizeDeg(start.bearingTo(end).toDouble())

        // Smooth HMM-like heading emission cost (replaces binary 0/40 threshold)
        val headingCost = if (headingDegrees != null) {
            val diff = angleDiffDeg(headingDegrees, segBearing)
            val absDiff = abs(diff)
            // Smooth penalty: 0 at 0°, ~5 at 45°, ~20 at 90°, ~60 at 180°
            val penalty = 60.0 * (1.0 - exp(-absDiff * absDiff / (50.0 * 50.0)))
            penalty
        } else 0.0

        // Along-track progress penalty: mildly penalize backward matches
        val progressPenalty = if (preferredSegmentIndex != null && segIndex < preferredSegmentIndex - 1) {
            val backSteps = preferredSegmentIndex - segIndex
            if (distance < 5.0) 0.0 else backSteps * 8.0  // strong penalty for going backward
        } else 0.0

        val totalScore = distance + headingCost + progressPenalty
        return SegmentScore(totalScore, distance, candidate, segBearing, fraction)
    }

    /**
     * Interpolates bearing at segment boundaries. When the projection fraction is near
     * 0.0 or 1.0, blends the bearing from the adjacent segment for smooth transitions.
     */
    private fun interpolateBearing(route: List<GeoPoint>, segIndex: Int, fraction: Double, rawBearing: Double?): Double? {
        if (rawBearing == null) return null
        val segBearing = rawBearing

        // Near end of segment: blend with next segment's bearing
        if (fraction > 0.75 && segIndex + 2 < route.size) {
            val nextBearing = normalizeDeg(route[segIndex + 1].bearingTo(route[segIndex + 2]).toDouble())
            val blendAlpha = (fraction - 0.75) / 0.25  // 0 at 0.75, 1 at 1.0
            return blendBearing(segBearing, nextBearing, blendAlpha * 0.5) // max 50% blend
        }

        // Near start of segment: blend with previous segment's bearing
        if (fraction < 0.25 && segIndex > 0) {
            val prevBearing = normalizeDeg(route[segIndex - 1].bearingTo(route[segIndex]).toDouble())
            val blendAlpha = (0.25 - fraction) / 0.25  // 1 at 0.0, 0 at 0.25
            return blendBearing(segBearing, prevBearing, blendAlpha * 0.5)
        }

        return segBearing
    }

    private fun blendBearing(a: Double, b: Double, alpha: Double): Double {
        val diff = angleDiffDeg(b, a)
        return normalizeDeg(a + diff * alpha)
    }

    private fun projectWithFraction(point: GeoPoint, start: GeoPoint, end: GeoPoint): Pair<GeoPoint, Double> {
        val latitudeScale = 111_111.0
        val longitudeScale = latitudeScale * cos(Math.toRadians(point.latitude))
        val bx = (end.longitude - start.longitude) * longitudeScale
        val by = (end.latitude - start.latitude) * latitudeScale
        val px = (point.longitude - start.longitude) * longitudeScale
        val py = (point.latitude - start.latitude) * latitudeScale
        val lengthSquared = bx * bx + by * by
        if (lengthSquared == 0.0) return Pair(start, 0.0)
        val fraction = ((px * bx + py * by) / lengthSquared).coerceIn(0.0, 1.0)
        val projected = GeoPoint(
            start.latitude + by * fraction / latitudeScale,
            start.longitude + bx * fraction / longitudeScale
        )
        return Pair(projected, fraction)
    }

    private fun angleDiffDeg(a: Double, b: Double): Double {
        var d = a - b
        while (d > 180.0) d -= 360.0
        while (d < -180.0) d += 360.0
        return d
    }

    private fun normalizeDeg(d: Double): Double {
        var v = d % 360.0
        if (v < 0.0) v += 360.0
        return v
    }
}
