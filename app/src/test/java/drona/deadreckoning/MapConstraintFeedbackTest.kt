package drona.deadreckoning

import kotlin.math.abs
import drona.deadreckoning.fusion.HeadingPolicy
import drona.deadreckoning.fusion.MapConstraintConfig
import drona.deadreckoning.fusion.NonHolonomicConfig
import drona.deadreckoning.fusion.VehicleFusionEkf
import drona.deadreckoning.support.SyntheticDrive
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.osmdroid.util.GeoPoint

/**
 * Stage 5 verification.
 *
 * The defect being fixed is that map matches only moved the marker while the filter kept
 * its drifted state. Two properties therefore matter:
 *
 * 1. The correction must actually land inside the estimator, so the *next* propagation
 *    starts from the corrected position.
 * 2. The correction must be across the road only. Snapping to the nearest point on a road
 *    reveals nothing about progress along it, so an isotropic update would fabricate
 *    along-track knowledge and could even move the vehicle backwards.
 */
class MapConstraintFeedbackTest {

    private val origin = GeoPoint(16.5, 80.6)
    private val dt = 1.0 / SyntheticDrive.SAMPLE_RATE_HZ

    private fun metresToGeo(eastMeters: Double, northMeters: Double) = GeoPoint(
        origin.latitude + northMeters / 111_111.0,
        origin.longitude + eastMeters / (111_111.0 * Math.cos(Math.toRadians(origin.latitude)))
    )

    private fun newFusion(map: MapConstraintConfig = MapConstraintConfig()) = VehicleFusionEkf(
        headingPolicy = HeadingPolicy.GYRO_ONLY,
        nonHolonomic = NonHolonomicConfig.DISABLED,
        mapConstraint = map
    )

    @Test
    fun `correction lands inside the estimator not just on the display`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)

        // Drive 100 m north; the estimator is 20 m east of a road that runs due north.
        fusion.predict(forwardMeters = 100.0, lateralMeters = 20.0, headingDeltaRadians = 0.0, intervalSeconds = 1.9)
        val beforeEast = eastOffsetMeters(fusion.state().position)

        val result = fusion.updateMapConstraint(
            matchedPosition = metresToGeo(eastMeters = 0.0, northMeters = 100.0),
            roadBearingDegrees = 0.0,
            confidence = 90
        )

        assertNotNull(result)
        assertTrue("constraint should apply", result!!.applied)

        // The estimator's own state moved, which is the whole point.
        val afterEast = eastOffsetMeters(fusion.state().position)
        assertTrue(
            "estimator east offset should shrink from $beforeEast to nearer zero, got $afterEast",
            abs(afterEast) < abs(beforeEast)
        )
        assertEquals(1, fusion.mapConstraintUpdates)
    }

    @Test
    fun `subsequent propagation continues from the corrected position`() {
        val corrected = newFusion()
        corrected.reset(origin, 15.0, 0.0, 5.0)
        corrected.predict(100.0, 20.0, 0.0, 1.9)
        corrected.updateMapConstraint(metresToGeo(0.0, 100.0), 0.0, 90)
        corrected.predict(100.0, 0.0, 0.0, 1.9)

        val uncorrected = newFusion(MapConstraintConfig.DISABLED)
        uncorrected.reset(origin, 15.0, 0.0, 5.0)
        uncorrected.predict(100.0, 20.0, 0.0, 1.9)
        uncorrected.updateMapConstraint(metresToGeo(0.0, 100.0), 0.0, 90)
        uncorrected.predict(100.0, 0.0, 0.0, 1.9)

        val correctedEast = abs(eastOffsetMeters(corrected.state().position))
        val uncorrectedEast = abs(eastOffsetMeters(uncorrected.state().position))

        assertTrue(
            "corrected run should stay nearer the road: $correctedEast m vs $uncorrectedEast m",
            correctedEast < uncorrectedEast
        )
    }

    @Test
    fun `along track position is not fabricated`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)
        fusion.predict(forwardMeters = 100.0, lateralMeters = 0.0, headingDeltaRadians = 0.0, intervalSeconds = 1.9)

        val northBefore = northOffsetMeters(fusion.state().position)

        // The match sits on the same north-running road but claims we are 40 m further back.
        val result = fusion.updateMapConstraint(
            matchedPosition = metresToGeo(eastMeters = 0.0, northMeters = 60.0),
            roadBearingDegrees = 0.0,
            confidence = 90
        )

        assertTrue(result!!.applied)
        val northAfter = northOffsetMeters(fusion.state().position)

        // Along-track uncertainty is enormous, so the along-track correction is negligible.
        assertEquals(
            "along-track position must be essentially untouched",
            northBefore,
            northAfter,
            0.5
        )
    }

    @Test
    fun `cross track correction respects road orientation`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 90.0, 5.0)

        // Road runs due east. Vehicle is 15 m north of it after driving 100 m east.
        fusion.predict(forwardMeters = 100.0, lateralMeters = -15.0, headingDeltaRadians = 0.0, intervalSeconds = 1.9)
        val northBefore = northOffsetMeters(fusion.state().position)
        val eastBefore = eastOffsetMeters(fusion.state().position)

        fusion.updateMapConstraint(
            matchedPosition = metresToGeo(eastMeters = eastBefore, northMeters = 0.0),
            roadBearingDegrees = 90.0,
            confidence = 90
        )

        val northAfter = northOffsetMeters(fusion.state().position)
        val eastAfter = eastOffsetMeters(fusion.state().position)

        assertTrue(
            "cross-track (north) offset should shrink from $northBefore to $northAfter",
            abs(northAfter) < abs(northBefore)
        )
        assertEquals(
            "along-track (east) offset should be preserved",
            eastBefore,
            eastAfter,
            0.5
        )
    }

    @Test
    fun `unknown road bearing is refused rather than applied isotropically`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)
        fusion.predict(100.0, 20.0, 0.0, 1.9)
        val before = fusion.state().position

        val result = fusion.updateMapConstraint(metresToGeo(0.0, 100.0), roadBearingDegrees = null, confidence = 90)

        assertFalse(result!!.applied)
        assertEquals("road bearing unknown", result.rejectionReason)
        assertEquals(0.0, fusion.state().position.distanceToAsDouble(before), 1e-6)
        assertEquals(1, fusion.rejectedMapConstraints)
    }

    @Test
    fun `low confidence match is refused`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)
        fusion.predict(100.0, 20.0, 0.0, 1.9)
        val before = fusion.state().position

        val result = fusion.updateMapConstraint(metresToGeo(0.0, 100.0), 0.0, confidence = 10)

        assertFalse(result!!.applied)
        assertTrue(result.rejectionReason!!.contains("confidence"))
        assertEquals(0.0, fusion.state().position.distanceToAsDouble(before), 1e-6)
    }

    @Test
    fun `wildly distant match is gated out as a wrong road`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 2.0)
        fusion.predict(100.0, 0.0, 0.0, 1.9)
        val before = fusion.state().position

        // A road 800 m to the side is not our road.
        val result = fusion.updateMapConstraint(metresToGeo(800.0, 100.0), 0.0, 90)

        assertFalse("far match must be gated", result!!.applied)
        assertTrue(result.rejectionReason!!.contains("gate"))
        assertEquals(0.0, fusion.state().position.distanceToAsDouble(before), 1e-6)
        assertEquals(1, fusion.rejectedMapConstraints)
    }

    @Test
    fun `disabled constraint is a no-op`() {
        val fusion = newFusion(MapConstraintConfig.DISABLED)
        fusion.reset(origin, 15.0, 0.0, 5.0)
        fusion.predict(100.0, 20.0, 0.0, 1.9)
        val before = fusion.state().position

        val result = fusion.updateMapConstraint(metresToGeo(0.0, 100.0), 0.0, 90)

        assertFalse(result!!.applied)
        assertEquals("constraint disabled", result.rejectionReason)
        assertEquals(0.0, fusion.state().position.distanceToAsDouble(before), 1e-6)
        assertEquals(0, fusion.mapConstraintUpdates)
    }

    @Test
    fun `constraint reduces reported uncertainty`() {
        val fusion = newFusion()
        fusion.reset(origin, 15.0, 0.0, 5.0)
        fusion.predict(100.0, 10.0, 0.0, 1.9)
        val before = fusion.state().horizontalUncertaintyMeters

        fusion.updateMapConstraint(metresToGeo(0.0, 100.0), 0.0, 90)
        val after = fusion.state().horizontalUncertaintyMeters

        assertTrue("uncertainty should fall from $before to $after", after < before)
        assertTrue("uncertainty must stay positive", after > 0.0)
    }

    @Test
    fun `constraint is refused before initialisation`() {
        val fusion = newFusion()
        assertNull(fusion.updateMapConstraint(metresToGeo(0.0, 0.0), 0.0, 90))
    }

    @Test
    fun `repeated constraints keep the drive pinned to the road`() {
        // A drive whose model output carries a persistent lateral bias should stay near a
        // straight road when the constraint feeds back every window.
        val truth = SyntheticDrive.straightDrive(seconds = 30.0, speedMps = 15.0)

        fun run(map: MapConstraintConfig): Double {
            val fusion = newFusion(map)
            fusion.reset(origin, truth.poses.first().speedMps, 0.0, 5.0)
            truth.windows.forEach { window ->
                window.gyroYawRates.forEach { fusion.predictGyro(it, dt) }
                // Persistent 1.5 m per window lateral bias.
                fusion.predict(window.forwardMeters, window.lateralMeters + 1.5, window.headingDeltaRadians, window.spanSeconds)

                val north = northOffsetMeters(fusion.state().position)
                fusion.updateMapConstraint(
                    matchedPosition = metresToGeo(0.0, north),
                    roadBearingDegrees = 0.0,
                    confidence = 85
                )
            }
            return abs(eastOffsetMeters(fusion.state().position))
        }

        val constrained = run(MapConstraintConfig())
        val unconstrained = run(MapConstraintConfig.DISABLED)

        assertTrue(
            "constraint should hold the solution near the road: $constrained m vs $unconstrained m",
            constrained < unconstrained * 0.3
        )
    }

    @Test
    fun `route matcher reports the segment bearing`() {
        // The estimator cannot apply a cross-track-only correction without it.
        val route = listOf(metresToGeo(0.0, 0.0), metresToGeo(0.0, 200.0))
        val match = drona.deadreckoning.util.RouteMapMatcher.match(metresToGeo(15.0, 100.0), route)

        assertNotNull(match)
        val bearing = match!!.bearingDegrees
        assertNotNull("bearing must be reported", bearing)
        assertEquals("route runs due north", 0.0, ((bearing!! + 360.0) % 360.0), 1.0)
    }

    private fun eastOffsetMeters(position: GeoPoint): Double =
        (position.longitude - origin.longitude) * 111_111.0 * Math.cos(Math.toRadians(origin.latitude))

    private fun northOffsetMeters(position: GeoPoint): Double =
        (position.latitude - origin.latitude) * 111_111.0
}
