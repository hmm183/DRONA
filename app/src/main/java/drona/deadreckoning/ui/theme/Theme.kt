package drona.deadreckoning.ui.theme

import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Typography
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

/**
 * Light automotive dashboard tokens.
 * Kept under the established names so visual changes never affect app behavior.
 */
val UberBlack = Color(0xFF172033)
val UberDarkCard = Color(0xFFFFFFFF)
val UberCardSurface = Color(0xFFFFFFFF)
val UberCardBorder = Color(0xFFE3E9F3)
val UberBlue = Color(0xFF3478F6)
val UberMintGreen = Color(0xFF1EAF78)
val UberAmber = Color(0xFFF4A340)
val ErrorRed = Color(0xFFE8525E)

val AutomotiveDarkBg = Color(0xFFF3F6FB)
val AutomotiveSurfaceBg = Color(0xFFFFFFFF)
val AutomotiveCardBg = Color(0xFFFFFFFF)
val AutomotiveCardBorder = Color(0xFFE3E9F3)
val PrimaryBlue = UberBlue
val SuccessGreen = UberMintGreen
val WarningAmber = UberAmber
val PurpleAI = Color(0xFF7654E8)
val RoadInk = Color(0xFFF8FAFE)
val PanelRaised = Color(0xFFF7F9FD)
val DividerSoft = Color(0xFFE6EBF3)

val TextPrimary = Color(0xFF172033)
val TextSecondary = Color(0xFF69778D)
val TextMuted = Color(0xFF9AA7BA)

val UberPillShape = CircleShape
val UberCardShape = RoundedCornerShape(20.dp)

private val DashboardColorScheme = lightColorScheme(
    primary = PurpleAI,
    onPrimary = Color.White,
    secondary = PrimaryBlue,
    onSecondary = Color.White,
    background = AutomotiveDarkBg,
    onBackground = TextPrimary,
    surface = AutomotiveCardBg,
    onSurface = TextPrimary,
    surfaceVariant = PanelRaised,
    onSurfaceVariant = TextSecondary,
    outline = AutomotiveCardBorder,
    error = ErrorRed,
    onError = Color.White
)

private val IDRTypography = Typography(
    headlineLarge = TextStyle(
        fontFamily = FontFamily.SansSerif,
        fontWeight = FontWeight.ExtraBold,
        fontSize = 30.sp,
        lineHeight = 35.sp
    ),
    headlineMedium = TextStyle(
        fontFamily = FontFamily.SansSerif,
        fontWeight = FontWeight.ExtraBold,
        fontSize = 22.sp,
        lineHeight = 27.sp
    ),
    titleMedium = TextStyle(
        fontFamily = FontFamily.SansSerif,
        fontWeight = FontWeight.Bold,
        fontSize = 15.sp,
        lineHeight = 21.sp
    ),
    bodyMedium = TextStyle(
        fontFamily = FontFamily.SansSerif,
        fontWeight = FontWeight.Medium,
        fontSize = 13.sp,
        lineHeight = 19.sp
    ),
    labelSmall = TextStyle(
        fontFamily = FontFamily.SansSerif,
        fontWeight = FontWeight.Bold,
        fontSize = 11.sp,
        lineHeight = 14.sp
    )
)

@Composable
fun IDRTheme(
    darkTheme: Boolean = false,
    content: @Composable () -> Unit
) {
    MaterialTheme(
        colorScheme = DashboardColorScheme,
        typography = IDRTypography,
        content = content
    )
}
