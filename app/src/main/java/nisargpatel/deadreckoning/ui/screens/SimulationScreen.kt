package nisargpatel.deadreckoning.ui.screens

import android.graphics.DashPathEffect
import androidx.compose.animation.*
import androidx.compose.foundation.*
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.compose.ui.window.Dialog
import kotlinx.coroutines.launch
import nisargpatel.deadreckoning.simulation.*
import nisargpatel.deadreckoning.ui.components.UberVehicleMarker
import nisargpatel.deadreckoning.ui.theme.*
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.Marker
import org.osmdroid.views.overlay.Polyline
import java.util.Locale

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SimulationScreen(
    onBack: () -> Unit = {}
) {
    val context = LocalContext.current
    val scope = rememberCoroutineScope()
    val controller = remember { SimulationController(context, scope) }
    val simState by controller.state.collectAsState()

    var showConfigDialog by remember { mutableStateOf(false) }
    var showReportDialog by remember { mutableStateOf(false) }
    var showMathSection by remember { mutableStateOf(true) }
    var showRouteMenu by remember { mutableStateOf(false) }
    var isFollowVehicleEnabled by remember { mutableStateOf(true) }
    var mapRef by remember { mutableStateOf<MapView?>(null) }

    val logListState = rememberLazyListState()

    // Auto-scroll logs as new entries arrive
    LaunchedEffect(simState.logs.size) {
        if (simState.logs.isNotEmpty()) {
            logListState.animateScrollToItem(simState.logs.size - 1)
        }
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = {
                    Column {
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Text(
                                text = "GNSS-Denied IDR Simulation",
                                fontSize = 16.sp,
                                fontWeight = FontWeight.Bold,
                                color = Color(0xFF0F172A)
                            )
                            Spacer(modifier = Modifier.width(8.dp))
                            // Status Badge
                            val (badgeBg, badgeFg, badgeText) = when {
                                simState.status == SimulationStatus.COMPLETED ->
                                    Triple(Color(0xFFDCFCE7), Color(0xFF16A34A), "COMPLETED")
                                simState.isOutageActive ->
                                    Triple(Color(0xFFFEE2E2), Color(0xFFDC2626), "GNSS OUTAGE")
                                simState.status == SimulationStatus.RUNNING ->
                                    Triple(Color(0xFFDBEAFE), Color(0xFF2563EB), "GNSS ACTIVE")
                                else ->
                                    Triple(Color(0xFFF1F5F9), Color(0xFF64748B), "READY")
                            }
                            Surface(
                                color = badgeBg,
                                shape = RoundedCornerShape(6.dp)
                            ) {
                                Text(
                                    text = badgeText,
                                    color = badgeFg,
                                    fontSize = 10.sp,
                                    fontWeight = FontWeight.Black,
                                    modifier = Modifier.padding(horizontal = 6.dp, vertical = 2.dp)
                                )
                            }
                        }
                        Text(
                            text = "Seed: ${simState.config.randomSeed} • ${simState.config.routeName}",
                            fontSize = 11.sp,
                            color = Color(0xFF64748B)
                        )
                    }
                },
                navigationIcon = {
                    IconButton(onClick = onBack) {
                        Icon(Icons.Default.ArrowBack, contentDescription = "Back")
                    }
                },
                actions = {
                    IconButton(onClick = { showRouteMenu = true }) {
                        Icon(Icons.Default.AltRoute, contentDescription = "Select Route", tint = Color(0xFF2563EB))
                    }
                    IconButton(onClick = { showConfigDialog = true }) {
                        Icon(Icons.Default.Tune, contentDescription = "Configuration", tint = Color(0xFF475569))
                    }
                    if (simState.completedReport != null || simState.status == SimulationStatus.COMPLETED) {
                        IconButton(onClick = { showReportDialog = true }) {
                            Icon(Icons.Default.Assessment, contentDescription = "Audit Report", tint = Color(0xFF16A34A))
                        }
                    }
                },
                colors = TopAppBarDefaults.topAppBarColors(containerColor = Color.White)
            )
        }
    ) { paddingValues ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .background(Color(0xFFF8FAFC))
                .padding(paddingValues)
                .verticalScroll(rememberScrollState())
        ) {
            // 1. Interactive OSM Map View
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(310.dp)
                    .background(Color(0xFF0F172A))
            ) {
                AndroidView(
                    factory = { ctx ->
                        MapView(ctx).apply {
                            setTileSource(TileSourceFactory.MAPNIK)
                            setMultiTouchControls(true)
                            isTilesScaledToDpi = true
                            this.controller.setZoom(15.5)
                            mapRef = this
                        }
                    },
                    update = { map ->
                        mapRef = map
                        map.overlays.clear()

                        // A. Ground Truth Reference Path (Dotted Slate)
                        if (simState.groundTruthPath.size >= 2) {
                            val gtLine = Polyline().apply {
                                outlinePaint.color = android.graphics.Color.parseColor("#94A3B8")
                                outlinePaint.strokeWidth = 3.5f
                                outlinePaint.pathEffect = DashPathEffect(floatArrayOf(12f, 8f), 0f)
                                setPoints(simState.groundTruthPath)
                            }
                            map.overlays.add(gtLine)
                        }

                        // B. Active GNSS Fix Path (Solid Blue) - Before & After Outage
                        if (simState.gnssActivePath.size >= 2) {
                            val gnssLine = Polyline().apply {
                                outlinePaint.color = android.graphics.Color.parseColor("#2563EB")
                                outlinePaint.strokeWidth = 6.0f
                                setPoints(simState.gnssActivePath)
                            }
                            map.overlays.add(gnssLine)
                        }

                        // C. Active DR Outage Path (Solid Red) - DYNAMIC SWITCH PER USER SPEC!
                        if (simState.drOutagePath.size >= 2) {
                            val outageLine = Polyline().apply {
                                outlinePaint.color = android.graphics.Color.parseColor("#DC2626")
                                outlinePaint.strokeWidth = 6.5f
                                setPoints(simState.drOutagePath)
                            }
                            map.overlays.add(outageLine)
                        }

                        // D. Naive DR Drifting Path (Dashed Amber)
                        if (simState.naiveDrPath.size >= 2) {
                            val naiveLine = Polyline().apply {
                                outlinePaint.color = android.graphics.Color.parseColor("#F59E0B")
                                outlinePaint.strokeWidth = 4.0f
                                outlinePaint.pathEffect = DashPathEffect(floatArrayOf(8f, 6f), 0f)
                                setPoints(simState.naiveDrPath)
                            }
                            map.overlays.add(naiveLine)
                        }

                        // E. Outage Start & End Markers
                        simState.outageStartPoint?.let { pt ->
                            val m = Marker(map).apply {
                                position = pt
                                setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM)
                                title = "Outage Begins (35%)"
                            }
                            map.overlays.add(m)
                        }
                        simState.outageEndPoint?.let { pt ->
                            val m = Marker(map).apply {
                                position = pt
                                setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM)
                                title = "GNSS Recovers (70%)"
                            }
                            map.overlays.add(m)
                        }

                        // F. Vehicle Marker
                        simState.groundTruthPosition?.let { pos ->
                            UberVehicleMarker.updateVehicleMarker(
                                mapView = map,
                                position = pos,
                                headingDegrees = simState.vehicleHeadingDegrees
                            )
                            if (isFollowVehicleEnabled) {
                                map.controller.setCenter(pos)
                            }
                        }
                    },
                    modifier = Modifier.fillMaxSize()
                )

                // Live Outage Banner Overlay on Map
                if (simState.isOutageActive) {
                    Surface(
                        color = Color(0xFFDC2626).copy(alpha = 0.92f),
                        shape = RoundedCornerShape(8.dp),
                        modifier = Modifier
                            .align(Alignment.TopCenter)
                            .padding(top = 10.dp)
                    ) {
                        Row(
                            verticalAlignment = Alignment.CenterVertically,
                            modifier = Modifier.padding(horizontal = 12.dp, vertical = 6.dp)
                        ) {
                            Box(
                                modifier = Modifier
                                    .size(8.dp)
                                    .clip(CircleShape)
                                    .background(Color.White)
                            )
                            Spacer(modifier = Modifier.width(6.dp))
                            Text(
                                text = "GNSS BLACKOUT ACTIVE — TRAJECTORY RED (DR MODE)",
                                color = Color.White,
                                fontSize = 11.sp,
                                fontWeight = FontWeight.Black
                            )
                        }
                    }
                }

                // Top-Right Floating Controls: Recenter, Follow Mode, Zoom Controls
                Column(
                    horizontalAlignment = Alignment.End,
                    modifier = Modifier
                        .align(Alignment.TopEnd)
                        .padding(top = 10.dp, end = 10.dp)
                ) {
                    // Recenter Button
                    Surface(
                        onClick = {
                            isFollowVehicleEnabled = true
                            simState.groundTruthPosition?.let { pos ->
                                mapRef?.controller?.setZoom(16.5)
                                mapRef?.controller?.setCenter(pos)
                            }
                        },
                        color = Color(0xFF2563EB),
                        shape = RoundedCornerShape(20.dp),
                        shadowElevation = 4.dp
                    ) {
                        Row(
                            verticalAlignment = Alignment.CenterVertically,
                            modifier = Modifier.padding(horizontal = 10.dp, vertical = 6.dp)
                        ) {
                            Icon(
                                imageVector = Icons.Default.MyLocation,
                                contentDescription = "Recenter on Car",
                                tint = Color.White,
                                modifier = Modifier.size(15.dp)
                            )
                            Spacer(modifier = Modifier.width(5.dp))
                            Text(
                                text = "Recenter",
                                color = Color.White,
                                fontSize = 11.sp,
                                fontWeight = FontWeight.Bold
                            )
                        }
                    }

                    Spacer(modifier = Modifier.height(6.dp))

                    // Follow Mode Toggle Pill
                    Surface(
                        onClick = {
                            isFollowVehicleEnabled = !isFollowVehicleEnabled
                            if (isFollowVehicleEnabled) {
                                simState.groundTruthPosition?.let { pos ->
                                    mapRef?.controller?.setCenter(pos)
                                }
                            }
                        },
                        color = if (isFollowVehicleEnabled) Color(0xFF10B981) else Color(0xFF1E293B).copy(alpha = 0.85f),
                        shape = RoundedCornerShape(20.dp),
                        shadowElevation = 3.dp
                    ) {
                        Row(
                            verticalAlignment = Alignment.CenterVertically,
                            modifier = Modifier.padding(horizontal = 8.dp, vertical = 5.dp)
                        ) {
                            Icon(
                                imageVector = if (isFollowVehicleEnabled) Icons.Default.Lock else Icons.Default.LockOpen,
                                contentDescription = "Follow Vehicle Mode",
                                tint = Color.White,
                                modifier = Modifier.size(13.dp)
                            )
                            Spacer(modifier = Modifier.width(4.dp))
                            Text(
                                text = if (isFollowVehicleEnabled) "Follow ON" else "Follow OFF",
                                color = Color.White,
                                fontSize = 10.sp,
                                fontWeight = FontWeight.Bold
                            )
                        }
                    }

                    Spacer(modifier = Modifier.height(8.dp))

                    // Zoom In / Out Buttons
                    Surface(
                        modifier = Modifier.width(36.dp),
                        color = Color.White.copy(alpha = 0.95f),
                        shape = RoundedCornerShape(8.dp),
                        shadowElevation = 3.dp,
                        border = BorderStroke(1.dp, Color(0xFFCBD5E1))
                    ) {
                        Column {
                            IconButton(
                                onClick = { mapRef?.controller?.zoomIn() },
                                modifier = Modifier.size(32.dp)
                            ) {
                                Icon(
                                    imageVector = Icons.Default.Add,
                                    contentDescription = "Zoom In",
                                    tint = Color(0xFF1E293B),
                                    modifier = Modifier.size(16.dp)
                                )
                            }
                            HorizontalDivider(color = Color(0xFFE2E8F0), thickness = 1.dp)
                            IconButton(
                                onClick = { mapRef?.controller?.zoomOut() },
                                modifier = Modifier.size(32.dp)
                            ) {
                                Icon(
                                    imageVector = Icons.Default.Remove,
                                    contentDescription = "Zoom Out",
                                    tint = Color(0xFF1E293B),
                                    modifier = Modifier.size(16.dp)
                                )
                            }
                        }
                    }
                }

                // Map Legend Card
                Surface(
                    color = Color.White.copy(alpha = 0.92f),
                    shape = RoundedCornerShape(8.dp),
                    border = BorderStroke(1.dp, Color(0xFFCBD5E1)),
                    modifier = Modifier
                        .align(Alignment.BottomStart)
                        .padding(8.dp)
                ) {
                    Column(modifier = Modifier.padding(8.dp)) {
                        LegendRow(color = Color(0xFF2563EB), label = "GNSS Active Path (Blue)")
                        LegendRow(color = Color(0xFFDC2626), label = "DR Outage Path (Red)")
                        LegendRow(color = Color(0xFFF59E0B), label = "Naive IMU DR (Dashed)")
                        LegendRow(color = Color(0xFF94A3B8), label = "True Road Geometry (Dotted)")
                    }
                }
            }

            // 2. Playback Control Bar
            Surface(
                color = Color.White,
                shadowElevation = 2.dp,
                modifier = Modifier.fillMaxWidth()
            ) {
                Column(modifier = Modifier.padding(horizontal = 16.dp, vertical = 10.dp)) {
                    // Progress Slider with Outage Zone Indicator
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(
                            text = "${(simState.progressFraction * 100).toInt()}% Journey",
                            fontSize = 12.sp,
                            fontWeight = FontWeight.Bold,
                            color = Color(0xFF334155)
                        )
                        Text(
                            text = "Blackout: ${(simState.config.blackoutStartPct * 100).toInt()}% → ${(simState.config.blackoutEndPct * 100).toInt()}%",
                            fontSize = 11.sp,
                            fontWeight = FontWeight.SemiBold,
                            color = if (simState.isOutageActive) Color(0xFFDC2626) else Color(0xFF64748B)
                        )
                    }

                    LinearProgressIndicator(
                        progress = { simState.progressFraction },
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(6.dp)
                            .clip(RoundedCornerShape(3.dp)),
                        color = if (simState.isOutageActive) Color(0xFFDC2626) else Color(0xFF2563EB),
                        trackColor = Color(0xFFE2E8F0)
                    )

                    Spacer(modifier = Modifier.height(8.dp))

                    // Row 1: Playback & Navigation Controls
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        // Play/Pause
                        FilledTonalButton(
                            onClick = {
                                if (simState.status == SimulationStatus.RUNNING) {
                                    controller.pause()
                                } else {
                                    controller.play()
                                }
                            },
                            colors = ButtonDefaults.filledTonalButtonColors(
                                containerColor = if (simState.status == SimulationStatus.RUNNING) Color(0xFFFEF2F2) else Color(0xFFEFF6FF),
                                contentColor = if (simState.status == SimulationStatus.RUNNING) Color(0xFFDC2626) else Color(0xFF2563EB)
                            ),
                            contentPadding = PaddingValues(horizontal = 12.dp, vertical = 6.dp)
                        ) {
                            Icon(
                                if (simState.status == SimulationStatus.RUNNING) Icons.Default.Pause else Icons.Default.PlayArrow,
                                contentDescription = "Play/Pause",
                                modifier = Modifier.size(16.dp)
                            )
                            Spacer(modifier = Modifier.width(4.dp))
                            Text(
                                text = if (simState.status == SimulationStatus.RUNNING) "Pause" else "Play",
                                fontWeight = FontWeight.Bold,
                                fontSize = 12.sp
                            )
                        }

                        // Step Forward
                        OutlinedButton(
                            onClick = { controller.stepForward() },
                            contentPadding = PaddingValues(horizontal = 9.dp, vertical = 6.dp)
                        ) {
                            Icon(Icons.Default.SkipNext, contentDescription = "Step", modifier = Modifier.size(16.dp))
                            Spacer(modifier = Modifier.width(2.dp))
                            Text("Step", fontSize = 11.sp, fontWeight = FontWeight.SemiBold)
                        }

                        // Skip -5%
                        OutlinedButton(
                            onClick = { controller.skipBackward5Percent() },
                            contentPadding = PaddingValues(horizontal = 9.dp, vertical = 6.dp)
                        ) {
                            Text("-5%", fontSize = 11.sp, fontWeight = FontWeight.Bold, color = Color(0xFF334155))
                        }

                        // Skip +5%
                        OutlinedButton(
                            onClick = { controller.skipForward5Percent() },
                            contentPadding = PaddingValues(horizontal = 9.dp, vertical = 6.dp)
                        ) {
                            Text("+5%", fontSize = 11.sp, fontWeight = FontWeight.Bold, color = Color(0xFF2563EB))
                        }

                        // Reset
                        OutlinedButton(
                            onClick = { controller.restart() },
                            contentPadding = PaddingValues(horizontal = 9.dp, vertical = 6.dp)
                        ) {
                            Icon(Icons.Default.Refresh, contentDescription = "Reset", modifier = Modifier.size(16.dp))
                        }
                    }

                    Spacer(modifier = Modifier.height(10.dp))

                    // Row 2: Speed Multipliers (100% visible, prominent, and easy to tap)
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Icon(
                                imageVector = Icons.Default.Speed,
                                contentDescription = "Speed",
                                tint = Color(0xFF64748B),
                                modifier = Modifier.size(16.dp)
                            )
                            Spacer(modifier = Modifier.width(5.dp))
                            Text(
                                text = "Simulation Speed",
                                fontSize = 12.sp,
                                fontWeight = FontWeight.SemiBold,
                                color = Color(0xFF475569)
                            )
                        }

                        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                            listOf(1.0f, 2.0f, 5.0f, 10.0f).forEach { mult ->
                                val active = simState.currentSpeedMultiplier == mult
                                Surface(
                                    color = if (active) Color(0xFF2563EB) else Color(0xFFF1F5F9),
                                    shape = RoundedCornerShape(8.dp),
                                    border = if (active) null else BorderStroke(1.dp, Color(0xFFE2E8F0)),
                                    modifier = Modifier
                                        .clickable { controller.setSpeedMultiplier(mult) }
                                ) {
                                    Text(
                                        text = "${mult.toInt()}x",
                                        color = if (active) Color.White else Color(0xFF334155),
                                        fontSize = 12.sp,
                                        fontWeight = FontWeight.Bold,
                                        modifier = Modifier.padding(horizontal = 14.dp, vertical = 5.dp)
                                    )
                                }
                            }
                        }
                    }
                }
            }

            Spacer(modifier = Modifier.height(10.dp))

            // 3. User-Requested Precision Spotlight: 5-Second Rolling Window Max Drift
            simState.metrics?.let { metrics ->
                Card(
                    colors = CardDefaults.cardColors(containerColor = Color.White),
                    border = BorderStroke(1.dp, Color(0xFFE2E8F0)),
                    shape = RoundedCornerShape(12.dp),
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(horizontal = 16.dp)
                ) {
                    Column(modifier = Modifier.padding(14.dp)) {
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text(
                                text = "LOCAL DRIFT (5-SEC ROLLING WINDOW)",
                                fontSize = 11.sp,
                                fontWeight = FontWeight.Black,
                                color = Color(0xFF2563EB),
                                letterSpacing = 0.5.sp
                            )
                            Surface(
                                color = if (metrics.meetsTarget) Color(0xFFDCFCE7) else Color(0xFFFEE2E2),
                                shape = RoundedCornerShape(4.dp)
                            ) {
                                Text(
                                    text = if (metrics.meetsTarget) "TARGET PASS (≤10%)" else "DRIFTING",
                                    color = if (metrics.meetsTarget) Color(0xFF16A34A) else Color(0xFFDC2626),
                                    fontSize = 10.sp,
                                    fontWeight = FontWeight.Bold,
                                    modifier = Modifier.padding(horizontal = 6.dp, vertical = 2.dp)
                                )
                            }
                        }

                        Spacer(modifier = Modifier.height(8.dp))

                        Row(modifier = Modifier.fillMaxWidth()) {
                            MetricCell(
                                title = "Hybrid Max 5s Drift",
                                value = "${String.format(Locale.US, "%.2f", metrics.hybridMaxDrift5s)} m",
                                subtitle = "Constrained by RBPF + FGO",
                                valueColor = Color(0xFF16A34A),
                                modifier = Modifier.weight(1f)
                            )
                            MetricCell(
                                title = "Naive DR 5s Drift",
                                value = "${String.format(Locale.US, "%.2f", metrics.naiveMaxDrift5s)} m",
                                subtitle = "Unchecked sensor bias",
                                valueColor = Color(0xFFEA580C),
                                modifier = Modifier.weight(1f)
                            )
                        }

                        HorizontalDivider(modifier = Modifier.padding(vertical = 8.dp), color = Color(0xFFF1F5F9))

                        Row(modifier = Modifier.fillMaxWidth()) {
                            MetricCell(
                                title = "Total Outage Drift",
                                value = "${String.format(Locale.US, "%.1f", metrics.finalDriftMeters)} m (${String.format(Locale.US, "%.2f", metrics.driftPctOfOutageDistance)}%)",
                                subtitle = "PS26168 Target: < 10.0%",
                                valueColor = if (metrics.meetsTarget) Color(0xFF16A34A) else Color(0xFFDC2626),
                                modifier = Modifier.weight(1f)
                            )
                            MetricCell(
                                title = "Road Corridor Lock",
                                value = "${String.format(Locale.US, "%.1f", metrics.roadConsistencyPct)}%",
                                subtitle = "Within 18m road corridor",
                                valueColor = Color(0xFF0F172A),
                                modifier = Modifier.weight(1f)
                            )
                        }
                    }
                }
            }

            Spacer(modifier = Modifier.height(10.dp))

            // 4. Mathematical Foundation & Fusion Equations (User-Requested Cool Section)
            Card(
                colors = CardDefaults.cardColors(containerColor = Color(0xFF0F172A)),
                shape = RoundedCornerShape(12.dp),
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(horizontal = 16.dp)
            ) {
                Column(modifier = Modifier.padding(14.dp)) {
                    Row(
                        modifier = Modifier
                            .fillMaxWidth()
                            .clickable { showMathSection = !showMathSection },
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Icon(Icons.Default.Functions, contentDescription = "Math", tint = Color(0xFF38BDF8), modifier = Modifier.size(18.dp))
                            Spacer(modifier = Modifier.width(6.dp))
                            Text(
                                text = "MATHEMATICAL FORMULATION & FUSION EQUATIONS",
                                color = Color.White,
                                fontSize = 11.sp,
                                fontWeight = FontWeight.Bold,
                                letterSpacing = 0.5.sp
                            )
                        }
                        Icon(
                            if (showMathSection) Icons.Default.ExpandLess else Icons.Default.ExpandMore,
                            contentDescription = "Toggle",
                            tint = Color(0xFF94A3B8)
                        )
                    }

                    if (showMathSection) {
                        Spacer(modifier = Modifier.height(10.dp))
                        HorizontalDivider(color = Color(0xFF1E293B))
                        Spacer(modifier = Modifier.height(10.dp))

                        // Tier 1: IMM-UKF
                        MathEquationBlock(
                            tier = "TIER 1: INTERACTING MULTIPLE MODEL (IMM-UKF)",
                            equation = "μ_j(k) = [ Λ_j(k) · ∑_i π_ij · μ_i(k-1) ] / c",
                            description = "Dynamically switches vehicle kinematics across CV, CTRV, and CA regimes.",
                            liveValue = "Active Mode: ${simState.dominantMotionMode} | μ = [${simState.modeProbabilities.joinToString(", ") { String.format(Locale.US, "%.2f", it) }}]"
                        )

                        Spacer(modifier = Modifier.height(10.dp))

                        // Tier 2: RBPF
                        MathEquationBlock(
                            tier = "TIER 2: RAO-BLACKWELLIZED PARTICLE FILTER (RBPF)",
                            equation = "w_k^(i) ∝ w_(k-1)^(i) · N(d_⊥^(i); 0, σ_⊥²) · cos(ψ_veh - θ_road)",
                            description = "Projects 30 particles onto candidate road segments, bounding cross-track drift.",
                            liveValue = "Particles: 30 | Best Hypothesis: ${simState.rbpfTopHypothesis?.roadName ?: "Centerline"} (${String.format(Locale.US, "%.1f", simState.rbpfTopHypothesis?.bearingDeg ?: 0.0)}°)"
                        )

                        Spacer(modifier = Modifier.height(10.dp))

                        // Tier 3: FGO
                        MathEquationBlock(
                            tier = "TIER 3: SLIDING-WINDOW FACTOR GRAPH OPTIMIZER (FGO)",
                            equation = "X* = argmin [ ||r_prior||²_Σ0 + ∑ ||r_odom||²_ΣΔ + ∑ ||r_road||²_Σmap ]",
                            description = "Levenberg-Marquardt nonlinear least-squares smoothing over 15 keyframe poses.",
                            liveValue = "Window Size: ${simState.fgoKeyframeCount} nodes | Damping λ: 2.0 | Convergence: 4 iters"
                        )
                    }
                }
            }

            Spacer(modifier = Modifier.height(10.dp))

            // 5. Live Engineering Event Logs
            Card(
                colors = CardDefaults.cardColors(containerColor = Color.White),
                border = BorderStroke(1.dp, Color(0xFFE2E8F0)),
                shape = RoundedCornerShape(12.dp),
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(horizontal = 16.dp)
            ) {
                Column(modifier = Modifier.padding(14.dp)) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Box(
                                modifier = Modifier
                                    .size(8.dp)
                                    .clip(CircleShape)
                                    .background(Color(0xFF16A34A))
                            )
                            Spacer(modifier = Modifier.width(6.dp))
                            Text(
                                text = "LIVE ENGINEERING TELEMETRY LOG",
                                fontSize = 11.sp,
                                fontWeight = FontWeight.Bold,
                                color = Color(0xFF0F172A),
                                letterSpacing = 0.5.sp
                            )
                        }

                        TextButton(
                            onClick = { showReportDialog = true },
                            contentPadding = PaddingValues(horizontal = 8.dp, vertical = 2.dp)
                        ) {
                            Icon(Icons.Default.Description, contentDescription = "Report", modifier = Modifier.size(14.dp))
                            Spacer(modifier = Modifier.width(4.dp))
                            Text("Full Audit", fontSize = 11.sp)
                        }
                    }

                    Spacer(modifier = Modifier.height(6.dp))

                    Box(
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(140.dp)
                            .clip(RoundedCornerShape(8.dp))
                            .background(Color(0xFF0F172A))
                            .padding(8.dp)
                    ) {
                        LazyColumn(state = logListState) {
                            items(simState.logs) { log ->
                                Text(
                                    text = log,
                                    color = when {
                                        log.contains("OUTAGE COMMENCED") || log.contains("RED") -> Color(0xFFF87171)
                                        log.contains("RECOVERED") || log.contains("PASS") -> Color(0xFF4ADE80)
                                        log.contains("TIER") -> Color(0xFF38BDF8)
                                        else -> Color(0xFFE2E8F0)
                                    },
                                    fontSize = 11.sp,
                                    fontFamily = FontFamily.Monospace,
                                    lineHeight = 15.sp
                                )
                            }
                        }
                    }
                }
            }

            Spacer(modifier = Modifier.height(20.dp))
        }
    }

    // Preset Route Selection Dropdown
    if (showRouteMenu) {
        Dialog(onDismissRequest = { showRouteMenu = false }) {
            Surface(
                shape = RoundedCornerShape(12.dp),
                color = Color.White,
                modifier = Modifier.fillMaxWidth()
            ) {
                Column(modifier = Modifier.padding(16.dp)) {
                    Text("Select Simulation Route", fontWeight = FontWeight.Bold, fontSize = 15.sp)
                    Spacer(modifier = Modifier.height(10.dp))
                    SimulationController.CANONICAL_ROUTES.forEach { opt ->
                        Row(
                            modifier = Modifier
                                .fillMaxWidth()
                                .clickable {
                                    controller.loadPresetRoute(opt)
                                    showRouteMenu = false
                                }
                                .padding(vertical = 10.dp),
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Icon(Icons.Default.Place, contentDescription = null, tint = Color(0xFF2563EB))
                            Spacer(modifier = Modifier.width(10.dp))
                            Text(opt.name, fontSize = 13.sp, fontWeight = FontWeight.Medium)
                        }
                    }
                }
            }
        }
    }

    // Configuration Dialog
    if (showConfigDialog) {
        SimulationConfigDialog(
            currentConfig = simState.config,
            onDismiss = { showConfigDialog = false },
            onApply = { newCfg ->
                controller.updateConfig(newCfg)
                showConfigDialog = false
            }
        )
    }

    // Comprehensive Post-Simulation Audit Report Dialog
    if (showReportDialog) {
        val reportText = remember { controller.exportReportText() }
        Dialog(onDismissRequest = { showReportDialog = false }) {
            Surface(
                shape = RoundedCornerShape(12.dp),
                color = Color(0xFF0F172A),
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(8.dp)
            ) {
                Column(modifier = Modifier.padding(16.dp)) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(
                            text = "ISRO PS26168 AUDIT REPORT",
                            color = Color.White,
                            fontWeight = FontWeight.Bold,
                            fontSize = 14.sp
                        )
                        IconButton(onClick = { showReportDialog = false }) {
                            Icon(Icons.Default.Close, contentDescription = "Close", tint = Color.White)
                        }
                    }

                    Spacer(modifier = Modifier.height(8.dp))

                    Box(
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(340.dp)
                            .verticalScroll(rememberScrollState())
                    ) {
                        Text(
                            text = reportText,
                            color = Color(0xFFE2E8F0),
                            fontSize = 11.sp,
                            fontFamily = FontFamily.Monospace,
                            lineHeight = 16.sp
                        )
                    }

                    Spacer(modifier = Modifier.height(10.dp))

                    Button(
                        onClick = { showReportDialog = false },
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF2563EB)),
                        modifier = Modifier.fillMaxWidth()
                    ) {
                        Text("Done")
                    }
                }
            }
        }
    }
}

@Composable
private fun LegendRow(color: Color, label: String) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = Modifier.padding(vertical = 2.dp)
    ) {
        Box(
            modifier = Modifier
                .size(14.dp, 4.dp)
                .background(color, RoundedCornerShape(2.dp))
        )
        Spacer(modifier = Modifier.width(6.dp))
        Text(text = label, fontSize = 10.sp, color = Color(0xFF334155), fontWeight = FontWeight.SemiBold)
    }
}

@Composable
private fun MetricCell(
    title: String,
    value: String,
    subtitle: String,
    valueColor: Color,
    modifier: Modifier = Modifier
) {
    Column(modifier = modifier.padding(horizontal = 4.dp)) {
        Text(text = title, fontSize = 11.sp, color = Color(0xFF64748B), fontWeight = FontWeight.Medium)
        Text(text = value, fontSize = 16.sp, color = valueColor, fontWeight = FontWeight.Bold)
        Text(text = subtitle, fontSize = 10.sp, color = Color(0xFF94A3B8))
    }
}

@Composable
private fun MathEquationBlock(
    tier: String,
    equation: String,
    description: String,
    liveValue: String
) {
    Column {
        Text(text = tier, fontSize = 10.sp, fontWeight = FontWeight.Bold, color = Color(0xFF38BDF8))
        Spacer(modifier = Modifier.height(3.dp))
        Surface(
            color = Color(0xFF1E293B),
            shape = RoundedCornerShape(6.dp),
            modifier = Modifier.fillMaxWidth()
        ) {
            Text(
                text = equation,
                color = Color(0xFFF1F5F9),
                fontSize = 12.sp,
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                modifier = Modifier.padding(horizontal = 8.dp, vertical = 6.dp)
            )
        }
        Spacer(modifier = Modifier.height(2.dp))
        Text(text = description, fontSize = 10.sp, color = Color(0xFF94A3B8))
        Text(text = "▶ $liveValue", fontSize = 10.sp, color = Color(0xFF4ADE80), fontWeight = FontWeight.SemiBold)
    }
}

@Composable
private fun SimulationConfigDialog(
    currentConfig: SimulationConfig,
    onDismiss: () -> Unit,
    onApply: (SimulationConfig) -> Unit
) {
    var seed by remember { mutableStateOf(currentConfig.randomSeed.toString()) }
    var blackoutStart by remember { mutableStateOf((currentConfig.blackoutStartPct * 100).toInt().toString()) }
    var blackoutEnd by remember { mutableStateOf((currentConfig.blackoutEndPct * 100).toInt().toString()) }
    var gyroBias by remember { mutableStateOf(currentConfig.gyroBiasDps.toString()) }

    Dialog(onDismissRequest = onDismiss) {
        Surface(
            shape = RoundedCornerShape(12.dp),
            color = Color.White,
            modifier = Modifier.fillMaxWidth()
        ) {
            Column(modifier = Modifier.padding(16.dp)) {
                Text("Simulation Configuration", fontWeight = FontWeight.Bold, fontSize = 15.sp)
                Spacer(modifier = Modifier.height(12.dp))

                OutlinedTextField(
                    value = seed,
                    onValueChange = { seed = it },
                    label = { Text("Random Seed") },
                    modifier = Modifier.fillMaxWidth()
                )
                Spacer(modifier = Modifier.height(8.dp))

                OutlinedTextField(
                    value = blackoutStart,
                    onValueChange = { blackoutStart = it },
                    label = { Text("Outage Start (%)") },
                    modifier = Modifier.fillMaxWidth()
                )
                Spacer(modifier = Modifier.height(8.dp))

                OutlinedTextField(
                    value = blackoutEnd,
                    onValueChange = { blackoutEnd = it },
                    label = { Text("Outage End (%)") },
                    modifier = Modifier.fillMaxWidth()
                )
                Spacer(modifier = Modifier.height(8.dp))

                OutlinedTextField(
                    value = gyroBias,
                    onValueChange = { gyroBias = it },
                    label = { Text("Gyro Bias (°/s)") },
                    modifier = Modifier.fillMaxWidth()
                )

                Spacer(modifier = Modifier.height(16.dp))

                Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.End) {
                    TextButton(onClick = onDismiss) { Text("Cancel") }
                    Spacer(modifier = Modifier.width(8.dp))
                    Button(
                        onClick = {
                            val newCfg = currentConfig.copy(
                                randomSeed = seed.toLongOrNull() ?: currentConfig.randomSeed,
                                blackoutStartPct = (blackoutStart.toFloatOrNull() ?: 35f) / 100f,
                                blackoutEndPct = (blackoutEnd.toFloatOrNull() ?: 70f) / 100f,
                                gyroBiasDps = gyroBias.toDoubleOrNull() ?: currentConfig.gyroBiasDps
                            )
                            onApply(newCfg)
                        }
                    ) {
                        Text("Apply")
                    }
                }
            }
        }
    }
}
