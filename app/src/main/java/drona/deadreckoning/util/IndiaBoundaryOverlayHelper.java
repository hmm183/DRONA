package drona.deadreckoning.util;

import android.content.Context;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.util.Log;

import org.json.JSONArray;
import org.json.JSONObject;
import org.osmdroid.util.GeoPoint;
import org.osmdroid.views.MapView;
import org.osmdroid.views.overlay.Polyline;

import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * Loads the official Survey of India political boundary overlay (including complete
 * Jammu & Kashmir, Ladakh, and Arunachal Pradesh) and renders it over any basemap.
 */
public class IndiaBoundaryOverlayHelper {

    private static final String TAG = "IndiaBoundaryOverlay";

    public static void applyOfficialBoundary(Context context, MapView mapView) {
        try {
            InputStream is = context.getAssets().open("india_boundary.geojson");
            int size = is.available();
            byte[] buffer = new byte[size];
            is.read(buffer);
            is.close();

            String jsonString = new String(buffer, StandardCharsets.UTF_8);
            JSONObject root = new JSONObject(jsonString);
            JSONArray features = root.getJSONArray("features");

            for (int i = 0; i < features.length(); i++) {
                JSONObject feature = features.getJSONObject(i);
                JSONObject geometry = feature.getJSONObject("geometry");
                String type = geometry.getString("type");

                if ("LineString".equalsIgnoreCase(type)) {
                    JSONArray coords = geometry.getJSONArray("coordinates");
                    addPolyline(coords, mapView);
                } else if ("MultiLineString".equalsIgnoreCase(type)) {
                    JSONArray multiCoords = geometry.getJSONArray("coordinates");
                    for (int j = 0; j < multiCoords.length(); j++) {
                        JSONArray coords = multiCoords.getJSONArray(j);
                        addPolyline(coords, mapView);
                    }
                } else if ("Polygon".equalsIgnoreCase(type)) {
                    JSONArray rings = geometry.getJSONArray("coordinates");
                    for (int j = 0; j < rings.length(); j++) {
                        JSONArray coords = rings.getJSONArray(j);
                        addPolyline(coords, mapView);
                    }
                } else if ("MultiPolygon".equalsIgnoreCase(type)) {
                    JSONArray polys = geometry.getJSONArray("coordinates");
                    for (int j = 0; j < polys.length(); j++) {
                        JSONArray rings = polys.getJSONArray(j);
                        for (int k = 0; k < rings.length(); k++) {
                            JSONArray coords = rings.getJSONArray(k);
                            addPolyline(coords, mapView);
                        }
                    }
                }
            }
            mapView.invalidate();
            Log.d(TAG, "Official Survey of India boundary overlay added successfully.");
        } catch (Exception e) {
            Log.e(TAG, "Failed to load india_boundary.geojson overlay", e);
        }
    }

    private static void addPolyline(JSONArray coords, MapView mapView) {
        try {
            List<GeoPoint> points = new ArrayList<>();
            for (int i = 0; i < coords.length(); i++) {
                JSONArray pt = coords.getJSONArray(i);
                double lon = pt.getDouble(0);
                double lat = pt.getDouble(1);
                points.add(new GeoPoint(lat, lon));
            }
            if (!points.isEmpty()) {
                Polyline line = new Polyline();
                line.setPoints(points);
                line.getOutlinePaint().setColor(Color.parseColor("#424242"));
                line.getOutlinePaint().setStrokeWidth(5.0f);
                line.getOutlinePaint().setStrokeCap(Paint.Cap.ROUND);
                line.getOutlinePaint().setPathEffect(new DashPathEffect(new float[]{15, 10}, 0));
                mapView.getOverlays().add(line);
            }
        } catch (Exception e) {
            Log.e(TAG, "Error parsing polyline", e);
        }
    }
}
