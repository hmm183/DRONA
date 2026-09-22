package drona.deadreckoning.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import drona.deadreckoning.domain.model.NavigationMode
import drona.deadreckoning.ui.theme.*

@Composable
fun ModeIndicator(
    mode: NavigationMode,
    modifier: Modifier = Modifier
) {
    val (color, icon, text) = when (mode) {
        NavigationMode.GNSS_INS -> Triple(SuccessGreen, Icons.Default.GpsFixed, "GNSS + INS")
        NavigationMode.AI_DEAD_RECKONING -> Triple(WarningAmber, Icons.Default.Psychology, "AI DEAD RECKONING")
        NavigationMode.GNSS_RECOVERY -> Triple(PrimaryBlue, Icons.Default.Sync, "GNSS RECOVERY")
        NavigationMode.OFFLINE -> Triple(Color(0xFFCBD5E1), Icons.Default.SignalCellularOff, "OFFLINE NAV")
        NavigationMode.CALIBRATION -> Triple(PurpleAI, Icons.Default.Tune, "CALIBRATION")
        NavigationMode.ERROR -> Triple(ErrorRed, Icons.Default.Warning, "SYSTEM ERROR")
    }

    Surface(
        modifier = modifier.shadow(1.dp, shape = CircleShape),
        shape = CircleShape,
        color = color.copy(alpha = 0.15f),
        border = androidx.compose.foundation.BorderStroke(1.5.dp, color)
    ) {
        Row(
            modifier = Modifier.padding(horizontal = 14.dp, vertical = 7.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Icon(
                imageVector = icon,
                contentDescription = text,
                tint = color,
                modifier = Modifier.size(18.dp)
            )
            Spacer(modifier = Modifier.width(6.dp))
            Text(
                text = text,
                color = color,
                fontWeight = FontWeight.ExtraBold,
                fontSize = 12.sp
            )
        }
    }
}

@Composable
fun DemoModeIndicator(
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier.shadow(1.dp, shape = CircleShape),
        shape = CircleShape,
        color = WarningAmber.copy(alpha = 0.18f),
        border = androidx.compose.foundation.BorderStroke(1.5.dp, WarningAmber)
    ) {
        Row(
            modifier = Modifier.padding(horizontal = 10.dp, vertical = 5.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Box(
                modifier = Modifier
                    .size(8.dp)
                    .background(WarningAmber, shape = CircleShape)
            )
            Spacer(modifier = Modifier.width(6.dp))
            Text(
                text = "DEMO MODE",
                color = WarningAmber,
                fontWeight = FontWeight.Bold,
                fontSize = 11.sp
            )
        }
    }
}

@Composable
fun OutageBanner(
    outageSeconds: Long,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier
            .fillMaxWidth()
            .shadow(2.dp, shape = RoundedCornerShape(8.dp)),
        shape = RoundedCornerShape(8.dp),
        color = Color.Unspecified,
        border = androidx.compose.foundation.BorderStroke(1.5.dp, ErrorRed)
    ) {
        Box(
            modifier = Modifier
                .background(
                    brush = Brush.horizontalGradient(
                        colors = listOf(
                            ErrorRed.copy(alpha = 0.92f),
                            Color(0xFF7F1D1D)
                        )
                    )
                )
                .padding(14.dp)
        ) {
            Row(
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(
                    imageVector = Icons.Default.GpsOff,
                    contentDescription = "Outage Warning",
                    tint = Color.White,
                    modifier = Modifier.size(26.dp)
                )
                Spacer(modifier = Modifier.width(12.dp))
                Column(modifier = Modifier.weight(1f)) {
                    Text(
                        text = "GNSS SIGNAL LOST",
                        color = Color.White,
                        fontWeight = FontWeight.Black,
                        fontSize = 15.sp
                    )
                    Text(
                        text = "AI Dead Reckoning Active • Outage Duration: ${outageSeconds}s",
                        color = Color.White.copy(alpha = 0.95f),
                        fontSize = 13.sp,
                        fontWeight = FontWeight.Medium
                    )
                }
            }
        }
    }
}

@Composable
fun ConfidenceIndicator(
    percentage: Int,
    label: String = "Confidence",
    modifier: Modifier = Modifier
) {
    val color = when {
        percentage >= 90 -> SuccessGreen
        percentage >= 70 -> WarningAmber
        else -> ErrorRed
    }
    val levelText = when {
        percentage >= 90 -> "HIGH"
        percentage >= 70 -> "MEDIUM"
        else -> "LOW"
    }

    Row(
        modifier = modifier,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(text = "$label: ", color = TextSecondary, fontSize = 12.sp, fontWeight = FontWeight.Medium)
        Surface(
            shape = CircleShape,
            color = color.copy(alpha = 0.2f),
            border = androidx.compose.foundation.BorderStroke(1.dp, color)
        ) {
            Text(
                text = "$levelText ($percentage%)",
                color = color,
                fontWeight = FontWeight.ExtraBold,
                fontSize = 12.sp,
                modifier = Modifier.padding(horizontal = 10.dp, vertical = 3.dp)
            )
        }
    }
}

@Composable
fun HealthIndicator(
    healthPercentage: Int,
    label: String = "System Health",
    modifier: Modifier = Modifier
) {
    val color = when {
        healthPercentage >= 90 -> SuccessGreen
        healthPercentage >= 70 -> WarningAmber
        else -> ErrorRed
    }

    Column(modifier = modifier) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            Text(text = label, color = TextSecondary, fontSize = 13.sp, fontWeight = FontWeight.SemiBold)
            Text(text = "$healthPercentage%", color = color, fontWeight = FontWeight.Black, fontSize = 13.sp)
        }
        Spacer(modifier = Modifier.height(6.dp))
        LinearProgressIndicator(
            progress = { healthPercentage / 100f },
            modifier = Modifier
                .fillMaxWidth()
                .height(8.dp)
                .clip(CircleShape),
            color = color,
            trackColor = AutomotiveCardBorder
        )
    }
}
