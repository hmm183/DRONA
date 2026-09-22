package drona.deadreckoning.ui.components

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.Undo
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import drona.deadreckoning.domain.model.ManeuverIconType
import drona.deadreckoning.domain.model.RouteInfo
import drona.deadreckoning.ui.theme.*

@Composable
fun UberNavigationHUD(
    routeInfo: RouteInfo,
    speedKmh: Double,
    headingDegrees: Double,
    accuracyMeters: Double,
    isNavigating: Boolean,
    onToggleNavigation: () -> Unit,
    onRecenterMap: () -> Unit,
    modifier: Modifier = Modifier
) {
    val hasRoute = routeInfo.routePoints.size > 1

    Surface(
        modifier = modifier
            .fillMaxWidth()
            .shadow(18.dp, shape = RoundedCornerShape(24.dp), ambientColor = Color(0x220F2445), spotColor = Color(0x220F2445)),
        shape = RoundedCornerShape(24.dp),
        color = AutomotiveCardBg.copy(alpha = 0.98f),
        border = androidx.compose.foundation.BorderStroke(1.dp, AutomotiveCardBorder)
    ) {
        Column(modifier = Modifier.padding(16.dp)) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Surface(
                    shape = CircleShape,
                    color = PrimaryBlue.copy(alpha = 0.13f),
                    modifier = Modifier.size(44.dp)
                ) {
                    Box(contentAlignment = Alignment.Center) {
                        val icon = when (routeInfo.maneuverIconType) {
                            ManeuverIconType.LEFT -> Icons.Default.TurnLeft
                            ManeuverIconType.RIGHT -> Icons.Default.TurnRight
                            ManeuverIconType.SLIGHT_LEFT -> Icons.Default.TurnSlightLeft
                            ManeuverIconType.SLIGHT_RIGHT -> Icons.Default.TurnSlightRight
                            ManeuverIconType.UTURN -> Icons.AutoMirrored.Filled.Undo
                            ManeuverIconType.ARRIVED -> Icons.Default.CheckCircle
                            ManeuverIconType.STRAIGHT -> Icons.Default.ArrowUpward
                        }
                        Icon(imageVector = icon, contentDescription = "Maneuver", tint = PrimaryBlue, modifier = Modifier.size(26.dp))
                    }
                }

                Spacer(modifier = Modifier.width(14.dp))

                Column(modifier = Modifier.weight(1f)) {
                    Text(text = routeInfo.nextManeuver, color = TextPrimary, fontWeight = FontWeight.ExtraBold, fontSize = 15.sp)
                    Text(text = "Destination: ${routeInfo.destinationName}", color = TextSecondary, fontSize = 12.sp, fontWeight = FontWeight.Medium)
                }
            }

            Spacer(modifier = Modifier.height(14.dp))
            Divider(color = DividerSoft, thickness = 1.dp)
            Spacer(modifier = Modifier.height(12.dp))

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                HudMetric("EST. ARRIVAL", if (hasRoute) "${routeInfo.estimatedTimeMinutes} min" else "--", PurpleAI)
                HudMetric("DISTANCE", if (hasRoute) String.format("%.1f km", routeInfo.totalDistanceKm) else "--", TextPrimary)
                HudMetric("SPEED", String.format("%.1f", speedKmh), PrimaryBlue)
                HudMetric("HEADING", String.format("%.0f°", headingDegrees), TextPrimary)
            }

            Spacer(modifier = Modifier.height(14.dp))

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(10.dp)
            ) {
                OutlinedButton(
                    onClick = onRecenterMap,
                    modifier = Modifier.weight(1f).height(48.dp),
                    shape = CircleShape,
                    border = androidx.compose.foundation.BorderStroke(1.dp, AutomotiveCardBorder),
                    contentPadding = PaddingValues(0.dp)
                ) {
                    Icon(imageVector = Icons.Default.MyLocation, contentDescription = "Recenter", tint = TextPrimary, modifier = Modifier.size(18.dp))
                    Spacer(modifier = Modifier.width(6.dp))
                    Text(text = "RECENTER", color = TextPrimary, fontSize = 12.sp, fontWeight = FontWeight.ExtraBold)
                }

                Button(
                    onClick = onToggleNavigation,
                    modifier = Modifier.weight(1.4f).height(48.dp).shadow(6.dp, shape = CircleShape),
                    colors = ButtonDefaults.buttonColors(containerColor = if (isNavigating) ErrorRed else PurpleAI),
                    shape = CircleShape,
                    contentPadding = PaddingValues(0.dp)
                ) {
                    Icon(
                        imageVector = if (isNavigating) Icons.Default.Stop else Icons.Default.Navigation,
                        contentDescription = "Navigate",
                        tint = Color.White,
                        modifier = Modifier.size(20.dp)
                    )
                    Spacer(modifier = Modifier.width(6.dp))
                    Text(
                        text = if (isNavigating) "END JOURNEY" else "START JOURNEY",
                        color = Color.White,
                        fontSize = 13.sp,
                        fontWeight = FontWeight.ExtraBold,
                        letterSpacing = 0.3.sp
                    )
                }
            }
        }
    }
}

@Composable
private fun HudMetric(label: String, value: String, color: Color) {
    Column {
        Text(text = label, color = TextMuted, fontSize = 9.sp, fontWeight = FontWeight.ExtraBold, letterSpacing = 0.4.sp)
        Text(text = value, color = color, fontSize = 19.sp, fontWeight = FontWeight.ExtraBold)
    }
}
