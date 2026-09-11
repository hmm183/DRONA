package nisargpatel.deadreckoning.ui.screens

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import nisargpatel.deadreckoning.domain.state.LatLngPoint
import nisargpatel.deadreckoning.domain.state.NavigationSession

@Composable
fun SessionDetailScreen(
    session: NavigationSession,
    onBack: () -> Unit
) {
    var showGnssPath by remember { mutableStateOf(true) }
    var showDrEstimate by remember { mutableStateOf(true) }
    var showReferenceActual by remember { mutableStateOf(true) }

    val meetsTarget = session.meetsTarget && (session.displayDriftPct <= 10.0)

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFFF8FAFC))
            .padding(horizontal = 16.dp)
            .verticalScroll(rememberScrollState())
    ) {
        Spacer(modifier = Modifier.height(16.dp))

        // Navigation Top Bar
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically
        ) {
            IconButton(
                onClick = onBack,
                modifier = Modifier
                    .size(40.dp)
                    .clip(CircleShape)
                    .background(Color.White)
            ) {
                Icon(
                    imageVector = Icons.Default.ArrowBack,
                    contentDescription = "Back",
                    tint = Color(0xFF0F172A)
                )
            }
            Spacer(modifier = Modifier.width(12.dp))
            Column {
                Text(
                    text = session.displayTitle,
                    fontSize = 19.sp,
                    fontWeight = FontWeight.Black,
                    color = Color(0xFF0F172A),
                    letterSpacing = (-0.5).sp
                )
                Text(
                    text = session.sessionSource,
                    fontSize = 12.sp,
                    color = Color(0xFF64748B),
                    fontWeight = FontWeight.Medium
                )
            }
        }

        Spacer(modifier = Modifier.height(16.dp))

        // Target Compliance Hero Card
        Surface(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(3.dp, RoundedCornerShape(20.dp)),
            shape = RoundedCornerShape(20.dp),
            color = Color.White,
            border = BorderStroke(1.dp, Color(0xFFE2E8F0))
        ) {
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(16.dp),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Column(modifier = Modifier.weight(1f)) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Box(
                            modifier = Modifier
                                .size(10.dp)
                                .clip(CircleShape)
                                .background(if (meetsTarget) Color(0xFF10B981) else Color(0xFFEF4444))
                        )
                        Spacer(modifier = Modifier.width(6.dp))
                        Text(
                            text = if (meetsTarget) "PASS — MEETS <10% DRIFT TARGET" else "TARGET EXCEEDED",
                            fontSize = 11.sp,
                            fontWeight = FontWeight.Black,
                            color = if (meetsTarget) Color(0xFF059669) else Color(0xFFDC2626),
                            letterSpacing = 0.5.sp
                        )
                    }
                    Spacer(modifier = Modifier.height(6.dp))
                    Text(
                        text = "Final Outage Drift: ${String.format("%.1f", session.displayDriftMeters)} m (${String.format("%.1f", session.displayDriftPct)}%)",
                        fontSize = 15.sp,
                        fontWeight = FontWeight.Bold,
                        color = Color(0xFF0F172A)
                    )
                    Text(
                        text = "PS26168 Benchmark: < 10.0% of blackout distance",
                        fontSize = 11.sp,
                        color = Color(0xFF64748B)
                    )
                }

                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(12.dp))
                        .background(if (meetsTarget) Color(0xFFECFDF5) else Color(0xFFFEF2F2))
                        .padding(horizontal = 12.dp, vertical = 8.dp),
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = "${String.format("%.1f", session.displayDriftPct)}%",
                        fontSize = 18.sp,
                        fontWeight = FontWeight.Black,
                        color = if (meetsTarget) Color(0xFF059669) else Color(0xFFDC2626)
                    )
                }
            }
        }

        Spacer(modifier = Modifier.height(14.dp))

        // 2x2 Telemetry Metric Grid
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            DetailMetricCard(
                title = "DISTANCE",
                value = String.format("%.2f km", session.displayDistanceKm),
                subtitle = "Recorded GPS Track",
                icon = Icons.Default.Route,
                iconColor = Color(0xFF2563EB),
                modifier = Modifier.weight(1f)
            )
            DetailMetricCard(
                title = "DURATION",
                value = session.displayDurationString,
                subtitle = "Total Driving Time",
                icon = Icons.Default.Timer,
                iconColor = Color(0xFF0F172A),
                modifier = Modifier.weight(1f)
            )
        }

        Spacer(modifier = Modifier.height(12.dp))

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            DetailMetricCard(
                title = "SIMULATED OUTAGE",
                value = "${if (session.outageDurationSeconds > 0) session.outageDurationSeconds else session.drDurationSeconds}s",
                subtitle = if (session.outageStartSeconds > 0) "Started @ T=${session.outageStartSeconds}s" else "GNSS-Denied Window",
                icon = Icons.Default.SatelliteAlt,
                iconColor = Color(0xFFF59E0B),
                modifier = Modifier.weight(1f)
            )
            DetailMetricCard(
                title = "SPEED RANGE",
                value = if (session.maxSpeedMps > 0.0) String.format("%.1f km/h", session.maxSpeedMps * 3.6) else "45.0 km/h",
                subtitle = if (session.avgSpeedMps > 0.0) "Avg: ${String.format("%.1f", session.avgSpeedMps * 3.6)} km/h" else "Vehicle Dynamics",
                icon = Icons.Default.Speed,
                iconColor = Color(0xFF6366F1),
                modifier = Modifier.weight(1f)
            )
        }

        Spacer(modifier = Modifier.height(16.dp))

        // Phase 5 Trajectory Visualization Canvas Card
        Surface(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(2.dp, RoundedCornerShape(20.dp)),
            shape = RoundedCornerShape(20.dp),
            color = Color.White,
            border = BorderStroke(1.dp, Color(0xFFE2E8F0))
        ) {
            Column(modifier = Modifier.padding(18.dp)) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Column {
                        Text(
                            text = "TRAJECTORY MULTI-LAYER RECONSTRUCTION",
                            fontSize = 11.sp,
                            fontWeight = FontWeight.Bold,
                            color = Color(0xFF64748B),
                            letterSpacing = 0.5.sp
                        )
                        Text(
                            text = "GNSS Fix vs DR Estimate vs Real Ground Truth",
                            fontSize = 12.sp,
                            color = Color(0xFF0F172A),
                            fontWeight = FontWeight.SemiBold
                        )
                    }
                }

                Spacer(modifier = Modifier.height(12.dp))

                // Checkable Layer Legend
                Surface(
                    shape = RoundedCornerShape(12.dp),
                    color = Color(0xFFF8FAFC),
                    border = BorderStroke(1.dp, Color(0xFFE2E8F0)),
                    modifier = Modifier.fillMaxWidth()
                ) {
                    Row(
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(horizontal = 8.dp, vertical = 6.dp),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Checkbox(
                                checked = showGnssPath,
                                onCheckedChange = { showGnssPath = it },
                                colors = CheckboxDefaults.colors(checkedColor = Color(0xFF2563EB))
                            )
                            Text(text = "GNSS Path", fontSize = 11.sp, fontWeight = FontWeight.Bold, color = Color(0xFF2563EB))
                        }
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Checkbox(
                                checked = showDrEstimate,
                                onCheckedChange = { showDrEstimate = it },
                                colors = CheckboxDefaults.colors(checkedColor = Color(0xFFEF4444))
                            )
                            Text(text = "DR Estimate", fontSize = 11.sp, fontWeight = FontWeight.Bold, color = Color(0xFFEF4444))
                        }
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Checkbox(
                                checked = showReferenceActual,
                                onCheckedChange = { showReferenceActual = it },
                                colors = CheckboxDefaults.colors(checkedColor = Color(0xFF64748B))
                            )
                            Text(text = "Actual (Dotted)", fontSize = 11.sp, fontWeight = FontWeight.Bold, color = Color(0xFF475569))
                        }
                    }
                }

                Spacer(modifier = Modifier.height(14.dp))

                // The Map Canvas
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(280.dp)
                        .clip(RoundedCornerShape(14.dp))
                        .background(Color(0xFF0F172A))
                ) {
                    TrajectoryMultiLayerCanvas(
                        pathGnss = session.pathGnss,
                        pathDrEstimate = session.pathDrEstimate,
                        pathReferenceActual = session.pathReferenceActual,
                        showGnss = showGnssPath,
                        showDr = showDrEstimate,
                        showActual = showReferenceActual
                    )
                }

                Spacer(modifier = Modifier.height(12.dp))

                // Explanatory legend footnote
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(6.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Icon(imageVector = Icons.Default.Info, contentDescription = null, tint = Color(0xFF64748B), modifier = Modifier.size(14.dp))
                    Text(
                        text = "Dotted line is the real GPS ground truth retained strictly for evaluation during the simulated outage.",
                        fontSize = 11.sp,
                        color = Color(0xFF64748B),
                        lineHeight = 15.sp
                    )
                }
            }
        }

        Spacer(modifier = Modifier.height(16.dp))

        // Error & Performance Precision Audit Card
        Surface(
            modifier = Modifier
                .fillMaxWidth()
                .shadow(2.dp, RoundedCornerShape(20.dp)),
            shape = RoundedCornerShape(20.dp),
            color = Color.White,
            border = BorderStroke(1.dp, Color(0xFFE2E8F0))
        ) {
            Column(modifier = Modifier.padding(18.dp)) {
                Text(
                    text = "PRECISION & DRIFT AUDIT",
                    fontSize = 11.sp,
                    fontWeight = FontWeight.Bold,
                    color = Color(0xFF64748B),
                    letterSpacing = 0.5.sp
                )
                Spacer(modifier = Modifier.height(14.dp))

                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Text(text = "Final Drift at Outage End", color = Color(0xFF64748B), fontSize = 13.sp)
                    Text(text = "${String.format("%.2f", session.displayDriftMeters)} m", color = Color(0xFF0F172A), fontWeight = FontWeight.Bold, fontSize = 13.sp)
                }
                Spacer(modifier = Modifier.height(10.dp))
                HorizontalDivider(color = Color(0xFFF1F5F9))
                Spacer(modifier = Modifier.height(10.dp))

                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Text(text = "Drift (% of Outage Distance)", color = Color(0xFF64748B), fontSize = 13.sp)
                    Text(
                        text = "${String.format("%.2f", session.displayDriftPct)}%",
                        color = if (meetsTarget) Color(0xFF059669) else Color(0xFFDC2626),
                        fontWeight = FontWeight.Bold,
                        fontSize = 13.sp
                    )
                }
                Spacer(modifier = Modifier.height(10.dp))
                HorizontalDivider(color = Color(0xFFF1F5F9))
                Spacer(modifier = Modifier.height(10.dp))

                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Text(text = "PS26168 Target Threshold", color = Color(0xFF64748B), fontSize = 13.sp)
                    Text(text = "≤ 10.0%", color = Color(0xFF0F172A), fontWeight = FontWeight.Bold, fontSize = 13.sp)
                }
                Spacer(modifier = Modifier.height(10.dp))
                HorizontalDivider(color = Color(0xFFF1F5F9))
                Spacer(modifier = Modifier.height(10.dp))

                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Text(text = "Target Evaluation Verdict", color = Color(0xFF64748B), fontSize = 13.sp)
                    Text(
                        text = if (meetsTarget) "MEETS TARGET" else "EXCEEDS TARGET",
                        color = if (meetsTarget) Color(0xFF059669) else Color(0xFFDC2626),
                        fontWeight = FontWeight.Black,
                        fontSize = 13.sp
                    )
                }
            }
        }

        Spacer(modifier = Modifier.height(20.dp))

        // Export Actions
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            Button(
                onClick = { },
                modifier = Modifier
                    .weight(1f)
                    .height(48.dp),
                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF2563EB)),
                shape = RoundedCornerShape(14.dp)
            ) {
                Icon(imageVector = Icons.Default.Download, contentDescription = null, modifier = Modifier.size(18.dp))
                Spacer(modifier = Modifier.width(8.dp))
                Text(text = "Export GPX", fontWeight = FontWeight.Bold, fontSize = 13.sp)
            }

            OutlinedButton(
                onClick = { },
                modifier = Modifier
                    .weight(1f)
                    .height(48.dp),
                shape = RoundedCornerShape(14.dp),
                border = BorderStroke(1.dp, Color(0xFFCBD5E1))
            ) {
                Icon(imageVector = Icons.Default.Download, contentDescription = null, tint = Color(0xFF0F172A), modifier = Modifier.size(18.dp))
                Spacer(modifier = Modifier.width(8.dp))
                Text(text = "Export CSV", fontWeight = FontWeight.Bold, fontSize = 13.sp, color = Color(0xFF0F172A))
            }
        }

        Spacer(modifier = Modifier.height(28.dp))
    }
}

@Composable
private fun TrajectoryMultiLayerCanvas(
    pathGnss: List<LatLngPoint>,
    pathDrEstimate: List<LatLngPoint>,
    pathReferenceActual: List<LatLngPoint>,
    showGnss: Boolean,
    showDr: Boolean,
    showActual: Boolean
) {
    val allPoints = remember(pathGnss, pathDrEstimate, pathReferenceActual) {
        (pathGnss + pathDrEstimate + pathReferenceActual).filter { it.latitude != 0.0 && it.longitude != 0.0 }
    }

    if (allPoints.size < 2) {
        Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                Icon(imageVector = Icons.Default.Map, contentDescription = null, tint = Color(0xFF64748B), modifier = Modifier.size(36.dp))
                Spacer(modifier = Modifier.height(8.dp))
                Text("No GPS Track coordinates recorded for this trip.", color = Color(0xFF94A3B8), fontSize = 12.sp)
            }
        }
        return
    }

    val minLat = remember(allPoints) { allPoints.minOf { it.latitude } }
    val maxLat = remember(allPoints) { allPoints.maxOf { it.latitude } }
    val minLon = remember(allPoints) { allPoints.minOf { it.longitude } }
    val maxLon = remember(allPoints) { allPoints.maxOf { it.longitude } }

    Canvas(
        modifier = Modifier
            .fillMaxSize()
            .padding(16.dp)
    ) {
        val latSpan = (maxLat - minLat).coerceAtLeast(0.0001)
        val lonSpan = (maxLon - minLon).coerceAtLeast(0.0001)

        fun pointFor(p: LatLngPoint): Offset {
            val x = ((p.longitude - minLon) / lonSpan * size.width).toFloat()
            val y = (size.height - (p.latitude - minLat) / latSpan * size.height).toFloat()
            return Offset(x, y)
        }

        fun drawPolyline(points: List<LatLngPoint>, color: Color, strokeWidth: Float, pathEffect: PathEffect? = null) {
            if (points.size < 2) return
            val path = Path().apply {
                val start = pointFor(points.first())
                moveTo(start.x, start.y)
                points.drop(1).forEach {
                    val pt = pointFor(it)
                    lineTo(pt.x, pt.y)
                }
            }
            drawPath(path, color = color, style = Stroke(width = strokeWidth, pathEffect = pathEffect))
        }

        // 1. Blue solid polyline: path_gnss (GNSS-tracked segments)
        if (showGnss && pathGnss.isNotEmpty()) {
            drawPolyline(pathGnss, color = Color(0xFF3B82F6), strokeWidth = 5f)
        }

        // 2. Dotted overlay (neutral color): path_reference_actual (real recorded path)
        if (showActual && pathReferenceActual.isNotEmpty()) {
            val dash = PathEffect.dashPathEffect(floatArrayOf(10f, 10f), 0f)
            drawPolyline(pathReferenceActual, color = Color(0xFF94A3B8), strokeWidth = 4f, pathEffect = dash)
        }

        // 3. Red solid polyline: path_dr_estimate (during simulated outage)
        if (showDr && pathDrEstimate.isNotEmpty()) {
            drawPolyline(pathDrEstimate, color = Color(0xFFEF4444), strokeWidth = 5.5f)
        }

        // Draw Start Pin
        if (allPoints.isNotEmpty()) {
            val startPt = pointFor(allPoints.first())
            drawCircle(color = Color(0xFF10B981), radius = 8f, center = startPt)
            drawCircle(color = Color.White, radius = 4f, center = startPt)
        }

        // Draw Outage Entry Pin
        if (pathDrEstimate.isNotEmpty()) {
            val drStart = pointFor(pathDrEstimate.first())
            drawCircle(color = Color(0xFFF59E0B), radius = 7f, center = drStart)
            drawCircle(color = Color.White, radius = 3.5f, center = drStart)
        }

        // Draw Destination Pin
        if (allPoints.isNotEmpty()) {
            val endPt = pointFor(allPoints.last())
            drawCircle(color = Color(0xFFEF4444), radius = 8f, center = endPt)
            drawCircle(color = Color.White, radius = 4f, center = endPt)
        }
    }
}

@Composable
private fun DetailMetricCard(
    title: String,
    value: String,
    subtitle: String,
    icon: ImageVector,
    iconColor: Color,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier.shadow(2.dp, RoundedCornerShape(20.dp)),
        shape = RoundedCornerShape(20.dp),
        color = Color.White,
        border = BorderStroke(1.dp, Color(0xFFE2E8F0))
    ) {
        Column(modifier = Modifier.padding(14.dp)) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = title,
                    fontSize = 10.sp,
                    fontWeight = FontWeight.Bold,
                    color = Color(0xFF64748B),
                    letterSpacing = 0.5.sp
                )
                Icon(imageVector = icon, contentDescription = null, tint = iconColor, modifier = Modifier.size(16.dp))
            }
            Spacer(modifier = Modifier.height(8.dp))
            Text(
                text = value,
                fontSize = 17.sp,
                fontWeight = FontWeight.Black,
                color = Color(0xFF0F172A)
            )
            Spacer(modifier = Modifier.height(2.dp))
            Text(
                text = subtitle,
                fontSize = 11.sp,
                color = Color(0xFF94A3B8)
            )
        }
    }
}
