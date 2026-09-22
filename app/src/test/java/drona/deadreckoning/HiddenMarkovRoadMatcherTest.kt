package drona.deadreckoning

import drona.deadreckoning.data.RoadCandidate
import drona.deadreckoning.matching.HiddenMarkovRoadMatcher
import org.junit.Assert.assertEquals
import org.junit.Test
import org.osmdroid.util.GeoPoint

class HiddenMarkovRoadMatcherTest {
    @Test
    fun `prefers a continuous road over a similarly close alternate`() {
        val matcher = HiddenMarkovRoadMatcher()
        val first = matcher.update(
            GeoPoint(16.5, 80.6),
            listOf(
                RoadCandidate("Main Road", GeoPoint(16.5, 80.6), 2.0),
                RoadCandidate("Side Road", GeoPoint(16.50001, 80.60001), 2.1)
            )
        )
        val second = matcher.update(
            GeoPoint(16.5001, 80.6),
            listOf(
                RoadCandidate("Main Road", GeoPoint(16.5001, 80.6), 2.3),
                RoadCandidate("Side Road", GeoPoint(16.5001, 80.60001), 1.9)
            )
        )
        assertEquals("Main Road", first?.candidate?.roadName)
        assertEquals("Main Road", second?.candidate?.roadName)
    }
}
