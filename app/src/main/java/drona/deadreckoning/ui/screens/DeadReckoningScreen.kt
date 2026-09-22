package drona.deadreckoning.ui.screens

import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.CompassCalibration
import androidx.compose.material.icons.filled.DirectionsCar
import androidx.compose.material.icons.filled.Psychology
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import drona.deadreckoning.ui.components.ConfidenceIndicator
import drona.deadreckoning.ui.components.CommandPanel
import drona.deadreckoning.ui.components.CommandScreen
import drona.deadreckoning.ui.components.MetricCard
import drona.deadreckoning.ui.components.PageHeader
import drona.deadreckoning.ui.components.SectionLabel
import drona.deadreckoning.ui.theme.*
import drona.deadreckoning.ui.viewmodel.NavigationViewModel

@Composable
fun DeadReckoningScreen(
    viewModel: NavigationViewModel
) {
    val navState by viewModel.navigationState.collectAsState()

    CommandScreen {
        PageHeader(
            title = "Dead Reckoning",
            subtitle = "Vehicle INS position estimate during GNSS stress",
            icon = Icons.Default.DirectionsCar,
            tint = WarningAmber,
            trailing = { ConfidenceIndicator(percentage = navState.confidencePercentage) }
        )

        CommandPanel {
            SectionLabel("Current estimated position")
            Text(
                text = if (navState.latitude == 0.0 && navState.longitude == 0.0) "Awaiting location" else "${String.format("%.4f", navState.latitude)}, ${String.format("%.4f", navState.longitude)}",
                color = PrimaryBlue,
                fontWeight = FontWeight.Black,
                fontSize = 20.sp
            )
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            MetricCard(
                title = "DR Vehicle Speed",
                value = String.format("%.1f km/h", navState.speedKmh),
                subtitle = "IMU + AI Fused",
                valueColor = SuccessGreen,
                modifier = Modifier.weight(1f)
            )
            MetricCard(
                title = "DR Heading",
                value = String.format("%.0f°", navState.headingDegrees),
                subtitle = "Gyro + Mag Fusion",
                valueColor = PrimaryBlue,
                modifier = Modifier.weight(1f)
            )
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            MetricCard(
                title = "DR Active Duration",
                value = "${navState.outageDurationSeconds} sec",
                valueColor = WarningAmber,
                modifier = Modifier.weight(1f)
            )
            MetricCard(
                title = "Estimated Position Drift",
                value = "${navState.accuracyMeters} m",
                subtitle = "Drift Accumulation",
                valueColor = ErrorRed,
                modifier = Modifier.weight(1f)
            )
        }

        CommandPanel(color = RoadInk, borderColor = DividerSoft) {
            SectionLabel("Estimator")
            Text(
                text = "EKF state is advanced by IMU samples, V8 speed inference, heading correction, and map-matched road continuity.",
                color = TextSecondary,
                fontSize = 12.sp,
                fontWeight = FontWeight.Medium
            )
        }
    }
}
