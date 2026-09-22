package drona.deadreckoning.util;

import org.osmdroid.tileprovider.tilesource.OnlineTileSourceBase;
import org.osmdroid.util.MapTileIndex;

/**
 * Tile source configuration for ISRO Bhuvan WMS mapping services.
 * Uses WMS GetMap requests with EPSG:3857 (Web Mercator) projection to match osmdroid's
 * default coordinate system.
 *
 * Strictly adheres to official Survey of India boundaries
 * (including complete Jammu & Kashmir and Ladakh as integral parts of India).
 *
 * No API token required — Bhuvan is a free, government-hosted service by ISRO/NRSC.
 */
public class BhuvanTileSource {

    // Web Mercator constants for BBOX calculation
    private static final double ORIGIN_SHIFT = 20037508.342789244;
    private static final double MAP_SIZE = ORIGIN_SHIFT * 2;

    /**
     * Calculates the Web Mercator (EPSG:3857) bounding box for a given tile.
     * Returns [west, south, east, north] (minX, minY, maxX, maxY).
     */
    private static double[] getBBox(int x, int y, int zoom) {
        double tileSize = MAP_SIZE / Math.pow(2.0, zoom);
        double west  = -ORIGIN_SHIFT + x * tileSize;
        double north =  ORIGIN_SHIFT - y * tileSize;
        double east  = -ORIGIN_SHIFT + (x + 1) * tileSize;
        double south =  ORIGIN_SHIFT - (y + 1) * tileSize;
        return new double[]{west, south, east, north};
    }

    /**
     * Bhuvan 2D Base Map — administrative boundaries, roads, water bodies, settlements.
     * Uses Bhuvan WMS with EPSG:3857 (Web Mercator) projection.
     */
    public static final OnlineTileSourceBase BHUVAN_BASE = new OnlineTileSourceBase(
        "Bhuvan_2D",
        1,
        18,
        256,
        ".png",
        new String[] {
            "https://bhuvan-vec1.nrsc.gov.in/bhuvan/wms?"
        }
    ) {
        @Override
        public String getTileURLString(long pMapTileIndex) {
            int x = MapTileIndex.getX(pMapTileIndex);
            int y = MapTileIndex.getY(pMapTileIndex);
            int zoom = MapTileIndex.getZoom(pMapTileIndex);

            double[] bbox = getBBox(x, y, zoom);

            return getBaseUrl()
                + "service=WMS"
                + "&version=1.1.1"
                + "&request=GetMap"
                + "&layers=basemap:admin_group_ntl,basemap:inida_state_ql_new,sisdp_base:sisdp_basemap"
                + "&styles="
                + "&format=image/png"
                + "&transparent=true"
                + "&width=256"
                + "&height=256"
                + "&srs=EPSG:3857"
                + "&bbox=" + bbox[0] + "," + bbox[1] + "," + bbox[2] + "," + bbox[3];
        }
    };

    /**
     * Bhuvan Satellite Imagery layer (Resourcesat).
     */
    public static final OnlineTileSourceBase BHUVAN_SATELLITE = new OnlineTileSourceBase(
        "Bhuvan_Satellite",
        1,
        18,
        256,
        ".png",
        new String[] {
            "https://bhuvan-vec1.nrsc.gov.in/bhuvan/wms?"
        }
    ) {
        @Override
        public String getTileURLString(long pMapTileIndex) {
            int x = MapTileIndex.getX(pMapTileIndex);
            int y = MapTileIndex.getY(pMapTileIndex);
            int zoom = MapTileIndex.getZoom(pMapTileIndex);

            double[] bbox = getBBox(x, y, zoom);

            return getBaseUrl()
                + "service=WMS"
                + "&version=1.1.1"
                + "&request=GetMap"
                + "&layers=bhuvan_satellite"
                + "&styles="
                + "&format=image/png"
                + "&transparent=true"
                + "&width=256"
                + "&height=256"
                + "&srs=EPSG:3857"
                + "&bbox=" + bbox[0] + "," + bbox[1] + "," + bbox[2] + "," + bbox[3];
        }
    };
}
