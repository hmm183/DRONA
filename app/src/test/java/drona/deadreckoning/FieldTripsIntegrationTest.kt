package drona.deadreckoning

import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import drona.deadreckoning.domain.state.NavigationSession
import org.junit.Assert.*
import org.junit.Test
import java.io.File

class FieldTripsIntegrationTest {

    @Test
    fun `verify field_trips json conforms to Phase 4 schema and parses correctly`() {
        val tripsFile = File("src/main/assets/trips/field_trips.json")
        assertTrue("field_trips.json must exist in assets/trips", tripsFile.exists())

        val json = tripsFile.readText()
        val gson = Gson()
        val listType = object : TypeToken<List<NavigationSession>>() {}.type
        val trips: List<NavigationSession> = gson.fromJson(json, listType)

        assertEquals("Expected 3 field test trips", 3, trips.size)

        // Trip 1: Mandadam to Vijayawada
        val trip1 = trips[0]
        assertEquals("Mandadam to Vijayawada", trip1.label)
        assertEquals("Mandadam", trip1.routeEndpoints?.start)
        assertEquals("Vijayawada", trip1.routeEndpoints?.end)
        assertTrue("Source must indicate field test", trip1.sessionSource.contains("Field test — own vehicle"))
        assertTrue("Distance must be around 9.1 km", trip1.displayDistanceKm in 8.0..10.0)
        assertTrue("Drift % must be under 10.0%", trip1.driftPctOfDistance <= 10.0)
        assertTrue("Meets target must be true", trip1.meetsTarget)
        assertTrue("path_gnss must contain coordinates", trip1.pathGnss.size >= 10)
        assertTrue("path_dr_estimate must contain coordinates", trip1.pathDrEstimate.size >= 10)
        assertTrue("path_reference_actual must contain coordinates", trip1.pathReferenceActual.size >= 10)

        // Trip 2: Mandadam to VIT-AP
        val trip2 = trips[1]
        assertEquals("Mandadam to VIT-AP", trip2.label)
        assertEquals("Mandadam", trip2.routeEndpoints?.start)
        assertEquals("VIT-AP", trip2.routeEndpoints?.end)
        assertTrue("Drift % must be under 10.0%", trip2.driftPctOfDistance <= 10.0)
        assertTrue("Meets target must be true", trip2.meetsTarget)

        // Trip 3: VIT-AP to Mangalagiri
        val trip3 = trips[2]
        assertEquals("VIT-AP to Mangalagiri", trip3.label)
        assertEquals("VIT-AP", trip3.routeEndpoints?.start)
        assertEquals("Mangalagiri", trip3.routeEndpoints?.end)
        assertTrue("Drift % must be under 10.0%", trip3.driftPctOfDistance <= 10.0)
        assertTrue("Meets target must be true", trip3.meetsTarget)
    }
}
