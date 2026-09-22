package drona.deadreckoning.ui.components

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.CompassCalibration
import androidx.compose.material.icons.filled.Psychology
import androidx.compose.material.icons.filled.Speed
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import drona.deadreckoning.ui.theme.*

@Composable
fun SpeedIndicator(
    speedKmh: Double,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier.shadow(2.dp, shape = RoundedCornerShape(8.dp)),
        shape = RoundedCornerShape(8.dp),
        color = PanelRaised,
        border = androidx.compose.foundation.BorderStroke(1.dp, DividerSoft)
    ) {
        Column(
            modifier = Modifier.padding(14.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(
                    imageVector = Icons.Default.Speed,
                    contentDescription = "Speed",
                    tint = PrimaryBlue,
                    modifier = Modifier.size(18.dp)
                )
                Spacer(modifier = Modifier.width(6.dp))
                Text(text = "SPEED", color = TextSecondary, fontSize = 11.sp, fontWeight = FontWeight.Bold)
            }
            Spacer(modifier = Modifier.height(4.dp))
            Text(
                text = String.format("%.1f", speedKmh),
                color = TextPrimary,
                fontSize = 30.sp,
                fontWeight = FontWeight.Black
            )
            Text(text = "km/h", color = PrimaryBlue, fontSize = 12.sp, fontWeight = FontWeight.Bold)
        }
    }
}

@Composable
fun HeadingIndicator(
    headingDegrees: Double,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier.shadow(2.dp, shape = RoundedCornerShape(8.dp)),
        shape = RoundedCornerShape(8.dp),
        color = PanelRaised,
        border = androidx.compose.foundation.BorderStroke(1.dp, DividerSoft)
    ) {
        Column(
            modifier = Modifier.padding(14.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(
                    imageVector = Icons.Default.CompassCalibration,
                    contentDescription = "Heading",
                    tint = PrimaryBlue,
                    modifier = Modifier.size(18.dp)
                )
                Spacer(modifier = Modifier.width(6.dp))
                Text(text = "HEADING", color = TextSecondary, fontSize = 11.sp, fontWeight = FontWeight.Bold)
            }
            Spacer(modifier = Modifier.height(4.dp))
            Text(
                text = String.format("%.0f°", headingDegrees),
                color = TextPrimary,
                fontSize = 30.sp,
                fontWeight = FontWeight.Black
            )
            Text(text = "Degrees", color = PrimaryBlue, fontSize = 12.sp, fontWeight = FontWeight.Bold)
        }
    }
}

@Composable
fun MetricCard(
    title: String,
    value: String,
    subtitle: String? = null,
    icon: ImageVector? = null,
    valueColor: Color = TextPrimary,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier.shadow(2.dp, shape = RoundedCornerShape(8.dp)),
        shape = RoundedCornerShape(8.dp),
        color = AutomotiveCardBg,
        border = androidx.compose.foundation.BorderStroke(1.dp, AutomotiveCardBorder)
    ) {
        Column(modifier = Modifier.padding(14.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                if (icon != null) {
                    Icon(imageVector = icon, contentDescription = title, tint = PrimaryBlue, modifier = Modifier.size(16.dp))
                    Spacer(modifier = Modifier.width(6.dp))
                }
                Text(text = title.uppercase(), color = TextSecondary, fontSize = 11.sp, fontWeight = FontWeight.Bold)
            }
            Spacer(modifier = Modifier.height(6.dp))
            Text(text = value, color = valueColor, fontSize = 21.sp, fontWeight = FontWeight.Black)
            if (subtitle != null) {
                Text(text = subtitle, color = TextSecondary, fontSize = 11.sp, fontWeight = FontWeight.Medium)
            }
        }
    }
}

@Composable
fun AIStatusCard(
    predictedSpeedKmh: Double,
    motionClassification: String,
    inferenceTimeMs: Long,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier
            .fillMaxWidth()
            .shadow(2.dp, shape = RoundedCornerShape(8.dp)),
        shape = RoundedCornerShape(8.dp),
        color = PanelRaised,
        border = androidx.compose.foundation.BorderStroke(1.dp, PurpleAI.copy(alpha = 0.55f))
    ) {
        Column(modifier = Modifier.padding(16.dp)) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Icon(imageVector = Icons.Default.Psychology, contentDescription = "AI", tint = PurpleAI, modifier = Modifier.size(24.dp))
                    Spacer(modifier = Modifier.width(8.dp))
                    Text(text = "ON-DEVICE AI ENGINE", color = PurpleAI, fontWeight = FontWeight.Black, fontSize = 14.sp)
                }
                Surface(
                    shape = CircleShape,
                    color = PurpleAI.copy(alpha = 0.2f),
                    border = androidx.compose.foundation.BorderStroke(1.dp, PurpleAI)
                ) {
                    Text(
                        text = "${inferenceTimeMs} ms",
                        color = PurpleAI,
                        fontSize = 11.sp,
                        fontWeight = FontWeight.ExtraBold,
                        modifier = Modifier.padding(horizontal = 10.dp, vertical = 3.dp)
                    )
                }
            }
            Spacer(modifier = Modifier.height(12.dp))
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                Column {
                    Text(text = "AI Predicted Speed", color = TextSecondary, fontSize = 11.sp, fontWeight = FontWeight.Medium)
                    Text(text = String.format("%.1f km/h", predictedSpeedKmh), color = TextPrimary, fontWeight = FontWeight.Black, fontSize = 18.sp)
                }
                Column(horizontalAlignment = Alignment.End) {
                    Text(text = "Motion Classification", color = TextSecondary, fontSize = 11.sp, fontWeight = FontWeight.Medium)
                    Text(text = motionClassification, color = SuccessGreen, fontWeight = FontWeight.Black, fontSize = 16.sp)
                }
            }
        }
    }
}
