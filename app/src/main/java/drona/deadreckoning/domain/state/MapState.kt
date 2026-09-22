package drona.deadreckoning.domain.state

import org.osmdroid.util.GeoPoint

data class MapState(
    val currentPosition: GeoPoint? = null,
    val rawDRPosition: GeoPoint? = null,
    val matchedPosition: GeoPoint? = null,
    val routePoints: List<GeoPoint> = emptyList(),
    val gnssTrajectory: List<GeoPoint> = emptyList(),
    val drTrajectory: List<GeoPoint> = emptyList(),
    val matchedTrajectory: List<GeoPoint> = emptyList(),
    val isMapAvailable: Boolean = true,
    val currentMapName: String = "OSMDroid Standard"
)
