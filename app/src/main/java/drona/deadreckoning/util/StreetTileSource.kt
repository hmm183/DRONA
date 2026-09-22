package drona.deadreckoning.util

import org.osmdroid.tileprovider.tilesource.OnlineTileSourceBase
import org.osmdroid.util.MapTileIndex

/**
 * Reliable Street-View Tile Sources for OSMDroid.
 * Uses direct URL construction to avoid OSMDroid baseUrl concatenation issues.
 */
object StreetTileSource {

    /**
     * Standard OpenStreetMap tile source with direct URL construction.
     * Uses multiple OSM tile server subdomains for load balancing.
     */
    val OSM_STREET_TILES = object : OnlineTileSourceBase(
        "OpenStreetMap",
        0,
        19,
        256,
        ".png",
        arrayOf(
            "https://a.tile.openstreetmap.org",
            "https://b.tile.openstreetmap.org",
            "https://c.tile.openstreetmap.org"
        )
    ) {
        override fun getTileURLString(pMapTileIndex: Long): String {
            val zoom = MapTileIndex.getZoom(pMapTileIndex)
            val x = MapTileIndex.getX(pMapTileIndex)
            val y = MapTileIndex.getY(pMapTileIndex)
            // Round-robin across subdomains
            val subdomain = arrayOf("a", "b", "c")[(x + y).toInt() % 3]
            return "https://$subdomain.tile.openstreetmap.org/$zoom/$x/$y.png"
        }
    }

    /**
     * Wikimedia tile source - high quality, free, no rate limiting.
     */
    val WIKIMEDIA_TILES = object : OnlineTileSourceBase(
        "Wikimedia",
        0,
        19,
        256,
        ".png",
        arrayOf("https://maps.wikimedia.org")
    ) {
        override fun getTileURLString(pMapTileIndex: Long): String {
            val zoom = MapTileIndex.getZoom(pMapTileIndex)
            val x = MapTileIndex.getX(pMapTileIndex)
            val y = MapTileIndex.getY(pMapTileIndex)
            return "https://maps.wikimedia.org/osm-intl/$zoom/$x/$y.png"
        }
    }
}
