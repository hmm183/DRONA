package nisargpatel.deadreckoning.ui.components

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.RowScope
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.clickable
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material3.Icon
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import nisargpatel.deadreckoning.ui.theme.AutomotiveCardBg
import nisargpatel.deadreckoning.ui.theme.AutomotiveCardBorder
import nisargpatel.deadreckoning.ui.theme.AutomotiveDarkBg
import nisargpatel.deadreckoning.ui.theme.DividerSoft
import nisargpatel.deadreckoning.ui.theme.PanelRaised
import nisargpatel.deadreckoning.ui.theme.PrimaryBlue
import nisargpatel.deadreckoning.ui.theme.TextMuted
import nisargpatel.deadreckoning.ui.theme.TextPrimary
import nisargpatel.deadreckoning.ui.theme.TextSecondary

private val DashboardCardShape = RoundedCornerShape(20.dp)

@Composable
fun CommandScreen(
    modifier: Modifier = Modifier,
    content: @Composable ColumnScope.() -> Unit
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .background(AutomotiveDarkBg)
            .verticalScroll(rememberScrollState())
            .padding(horizontal = 16.dp, vertical = 18.dp),
        verticalArrangement = Arrangement.spacedBy(14.dp),
        content = content
    )
}

@Composable
fun PageHeader(
    title: String,
    subtitle: String,
    icon: ImageVector,
    tint: Color = PrimaryBlue,
    onBack: (() -> Unit)? = null,
    trailing: (@Composable () -> Unit)? = null
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Row(verticalAlignment = Alignment.CenterVertically, modifier = Modifier.weight(1f)) {
            if (onBack != null) {
                Surface(
                    shape = CircleShape,
                    color = Color.White.copy(alpha = 0.08f),
                    border = BorderStroke(1.dp, Color.White.copy(alpha = 0.12f)),
                    modifier = Modifier.clickable { onBack() }
                ) {
                    Icon(
                        imageVector = Icons.Default.ArrowBack,
                        contentDescription = "Back",
                        tint = TextPrimary,
                        modifier = Modifier
                            .padding(8.dp)
                            .size(20.dp)
                    )
                }
                Spacer(modifier = Modifier.width(10.dp))
            }
            Surface(
                shape = RoundedCornerShape(15.dp),
                color = tint.copy(alpha = 0.12f),
                border = BorderStroke(1.dp, tint.copy(alpha = 0.18f))
            ) {
                Icon(
                    imageVector = icon,
                    contentDescription = title,
                    tint = tint,
                    modifier = Modifier
                        .padding(10.dp)
                        .size(22.dp)
                )
            }
            Spacer(modifier = Modifier.width(12.dp))
            Column {
                Text(text = title, color = TextPrimary, fontWeight = FontWeight.ExtraBold, fontSize = 22.sp)
                Text(text = subtitle, color = TextSecondary, fontSize = 12.sp, fontWeight = FontWeight.Medium)
            }
        }
        if (trailing != null) {
            Spacer(modifier = Modifier.width(10.dp))
            trailing()
        }
    }
}

@Composable
fun SectionLabel(text: String, color: Color = TextMuted) {
    Text(
        text = text.uppercase(),
        color = color,
        fontSize = 10.sp,
        fontWeight = FontWeight.ExtraBold,
        letterSpacing = 0.7.sp
    )
}

@Composable
fun CommandPanel(
    modifier: Modifier = Modifier,
    color: Color = AutomotiveCardBg,
    borderColor: Color = AutomotiveCardBorder,
    content: @Composable ColumnScope.() -> Unit
) {
    Surface(
        modifier = modifier
            .fillMaxWidth()
            .shadow(8.dp, shape = DashboardCardShape, ambientColor = Color(0x120F2445), spotColor = Color(0x120F2445)),
        shape = DashboardCardShape,
        color = color,
        border = BorderStroke(1.dp, borderColor)
    ) {
        Column(
            modifier = Modifier.padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp),
            content = content
        )
    }
}

@Composable
fun StatusPill(
    text: String,
    color: Color,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier,
        shape = CircleShape,
        color = color.copy(alpha = 0.11f),
        border = BorderStroke(1.dp, color.copy(alpha = 0.24f))
    ) {
        Text(
            text = text,
            color = color,
            fontWeight = FontWeight.ExtraBold,
            fontSize = 10.sp,
            modifier = Modifier.padding(horizontal = 10.dp, vertical = 5.dp)
        )
    }
}

@Composable
fun DataRow(label: String, value: String, valueColor: Color = TextPrimary) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(text = label, color = TextSecondary, fontSize = 12.sp, fontWeight = FontWeight.Medium)
        Text(text = value, color = valueColor, fontSize = 13.sp, fontWeight = FontWeight.Bold)
    }
}

@Composable
fun DividerLine() {
    Spacer(
        modifier = Modifier
            .fillMaxWidth()
            .height(1.dp)
            .background(DividerSoft)
    )
}

@Composable
fun RouteStrip(content: @Composable RowScope.() -> Unit) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .background(PanelRaised, RoundedCornerShape(15.dp))
            .padding(14.dp),
        horizontalArrangement = Arrangement.spacedBy(10.dp),
        verticalAlignment = Alignment.CenterVertically,
        content = content
    )
}
