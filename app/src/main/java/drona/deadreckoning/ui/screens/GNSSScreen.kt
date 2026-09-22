package drona.deadreckoning.ui.screens

import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.GpsFixed
import androidx.compose.material.icons.filled.GpsOff
import androidx.compose.material.icons.filled.SatelliteAlt
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import drona.deadreckoning.ui.components.CommandPanel
import drona.deadreckoning.ui.components.CommandScreen
import drona.deadreckoning.ui.components.DataRow
import drona.deadreckoning.ui.components.DividerLine
import drona.deadreckoning.ui.components.MetricCard
import drona.deadreckoning.ui.components.PageHeader
import drona.deadreckoning.ui.components.SectionLabel
import drona.deadreckoning.ui.components.StatusPill
import drona.deadreckoning.ui.theme.*
import drona.deadreckoning.ui.viewmodel.GNSSViewModel

@Composable
fun GNSSScreen(
    viewModel: GNSSViewModel
) {
    val gnssState by viewModel.gnssState.collectAsState()

    CommandScreen {
        PageHeader(
            title = "GNSS Monitor",
            subtitle = "Satellite fix, precision, bearing, and outage state",
            icon = if (gnssState.isAvailable) Icons.Default.GpsFixed else Icons.Default.GpsOff,
            tint = if (gnssState.isAvailable) SuccessGreen else ErrorRed,
            trailing = {
                StatusPill(text = if (gnssState.isAvailable) "Active" else "No fix", color = if (gnssState.isAvailable) SuccessGreen else ErrorRed)
            }
        )

        CommandPanel {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Icon(imageVector = Icons.Default.SatelliteAlt, contentDescription = "Sats", tint = PrimaryBlue)
                        Spacer(modifier = Modifier.width(8.dp))
                        Text(text = "${gnssState.satelliteCount} Satellites In View", color = TextPrimary, fontWeight = FontWeight.Bold, fontSize = 16.sp)
                    }
                Text(text = gnssState.fixStatus, color = if (gnssState.isAvailable) SuccessGreen else WarningAmber, fontWeight = FontWeight.Bold, fontSize = 13.sp)
            }
            DividerLine()
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                    Column {
                        Text(text = "Accuracy", color = TextSecondary, fontSize = 11.sp)
                        Text(text = "${gnssState.accuracyMeters} m", color = TextPrimary, fontWeight = FontWeight.Bold, fontSize = 15.sp)
                    }
                    Column {
                        Text(text = "Signal Quality", color = TextSecondary, fontSize = 11.sp)
                        Text(text = "${gnssState.signalQualityPercentage}%", color = SuccessGreen, fontWeight = FontWeight.Bold, fontSize = 15.sp)
                    }
                    Column(horizontalAlignment = Alignment.End) {
                        Text(text = "Outage Duration", color = TextSecondary, fontSize = 11.sp)
                        Text(text = "${gnssState.outageDurationSeconds}s", color = if (gnssState.outageDurationSeconds > 0) ErrorRed else TextPrimary, fontWeight = FontWeight.Bold, fontSize = 15.sp)
                    }
            }
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            MetricCard(
                title = "Latitude",
                value = String.format("%.4f", gnssState.latitude),
                subtitle = "WGS84 Datum",
                valueColor = PrimaryBlue,
                modifier = Modifier.weight(1f)
            )
            MetricCard(
                title = "Longitude",
                value = String.format("%.4f", gnssState.longitude),
                subtitle = "WGS84 Datum",
                valueColor = PrimaryBlue,
                modifier = Modifier.weight(1f)
            )
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            MetricCard(
                title = "GNSS Speed",
                value = String.format("%.1f km/h", gnssState.speedKmh),
                valueColor = SuccessGreen,
                modifier = Modifier.weight(1f)
            )
            MetricCard(
                title = "GNSS Bearing",
                value = String.format("%.0f°", gnssState.bearingDegrees),
                valueColor = PrimaryBlue,
                modifier = Modifier.weight(1f)
            )
        }

        CommandPanel(color = RoadInk, borderColor = DividerSoft) {
            SectionLabel("Precision")
            DataRow("Horizontal DOP", String.format("%.1f", gnssState.hdop), SuccessGreen)
            DataRow("Vertical DOP", String.format("%.1f", gnssState.vdop), SuccessGreen)
            DataRow("Bearing", String.format("%.0f deg", gnssState.bearingDegrees), PrimaryBlue)
        }
    }
}
