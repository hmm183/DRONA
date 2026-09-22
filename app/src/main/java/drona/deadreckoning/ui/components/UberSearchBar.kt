package drona.deadreckoning.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.LocationOn
import androidx.compose.material.icons.filled.Search
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import drona.deadreckoning.domain.model.RouteInfo
import drona.deadreckoning.ui.theme.*
import org.osmdroid.util.GeoPoint

@Composable
fun UberSearchBar(
    currentRoute: RouteInfo,
    onDestinationSelected: (String, GeoPoint) -> Unit,
    modifier: Modifier = Modifier
) {
    var showSearchModal by remember { mutableStateOf(false) }

    Surface(
        modifier = modifier.fillMaxWidth().shadow(10.dp, shape = RoundedCornerShape(20.dp)),
        shape = RoundedCornerShape(20.dp),
        color = AutomotiveCardBg.copy(alpha = 0.98f),
        border = androidx.compose.foundation.BorderStroke(1.dp, AutomotiveCardBorder)
    ) {
        Column(modifier = Modifier.clickable { showSearchModal = true }.padding(15.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Box(modifier = Modifier.size(10.dp).background(PrimaryBlue, shape = CircleShape))
                Spacer(modifier = Modifier.width(12.dp))
                Text(text = currentRoute.sourceName, color = TextPrimary, fontWeight = FontWeight.Bold, fontSize = 13.sp)
            }

            Box(
                modifier = Modifier.padding(start = 4.dp, top = 3.dp, bottom = 3.dp)
                    .width(2.dp).height(12.dp).background(DividerSoft)
            )

            Row(verticalAlignment = Alignment.CenterVertically) {
                Box(modifier = Modifier.size(10.dp).background(PurpleAI, shape = CircleShape))
                Spacer(modifier = Modifier.width(12.dp))
                Row(
                    modifier = Modifier.weight(1f),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(text = currentRoute.destinationName, color = TextPrimary, fontWeight = FontWeight.ExtraBold, fontSize = 14.sp)
                    Surface(shape = CircleShape, color = PurpleAI.copy(alpha = 0.12f)) {
                        Row(
                            modifier = Modifier.padding(horizontal = 9.dp, vertical = 5.dp),
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Icon(imageVector = Icons.Default.Search, contentDescription = "Search", tint = PurpleAI, modifier = Modifier.size(15.dp))
                            Spacer(modifier = Modifier.width(4.dp))
                            Text(text = "Search", color = PurpleAI, fontSize = 11.sp, fontWeight = FontWeight.ExtraBold)
                        }
                    }
                }
            }
        }
    }

    if (showSearchModal) {
        UberDestinationSearchModal(
            onDismiss = { showSearchModal = false },
            onSelect = { name, geoPoint ->
                onDestinationSelected(name, geoPoint)
                showSearchModal = false
            }
        )
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun UberDestinationSearchModal(
    onDismiss: () -> Unit,
    onSelect: (String, GeoPoint) -> Unit
) {
    var destinationName by remember { mutableStateOf("") }
    var coordinates by remember { mutableStateOf("") }
    val coordinateParts = coordinates.split(',').map(String::trim)
    val latitude = coordinateParts.getOrNull(0)?.toDoubleOrNull()
    val longitude = coordinateParts.getOrNull(1)?.toDoubleOrNull()
    val canSelectDestination = latitude != null && longitude != null && latitude in -90.0..90.0 && longitude in -180.0..180.0

    ModalBottomSheet(onDismissRequest = onDismiss, containerColor = AutomotiveCardBg, contentColor = TextPrimary) {
        Column(modifier = Modifier.fillMaxWidth().padding(20.dp)) {
            Text(text = "SET DESTINATION", color = TextPrimary, fontWeight = FontWeight.ExtraBold, fontSize = 20.sp, letterSpacing = 0.4.sp)
            Spacer(modifier = Modifier.height(14.dp))

            OutlinedTextField(
                value = destinationName,
                onValueChange = { destinationName = it },
                label = { Text("Destination name") },
                placeholder = { Text("Optional label", color = TextSecondary) },
                leadingIcon = { Icon(Icons.Default.LocationOn, contentDescription = null, tint = PrimaryBlue) },
                modifier = Modifier.fillMaxWidth(),
                colors = OutlinedTextFieldDefaults.colors(
                    focusedBorderColor = PurpleAI,
                    unfocusedBorderColor = AutomotiveCardBorder,
                    focusedContainerColor = RoadInk,
                    unfocusedContainerColor = RoadInk
                ),
                shape = RoundedCornerShape(14.dp)
            )
            Spacer(modifier = Modifier.height(10.dp))

            OutlinedTextField(
                value = coordinates,
                onValueChange = { coordinates = it },
                label = { Text("Destination coordinates") },
                placeholder = { Text("Latitude, longitude", color = TextSecondary) },
                leadingIcon = { Icon(Icons.Default.Search, contentDescription = null, tint = PrimaryBlue) },
                isError = coordinates.isNotBlank() && !canSelectDestination,
                supportingText = { if (coordinates.isNotBlank() && !canSelectDestination) Text("Enter valid latitude and longitude values") },
                modifier = Modifier.fillMaxWidth(),
                colors = OutlinedTextFieldDefaults.colors(
                    focusedBorderColor = PurpleAI,
                    unfocusedBorderColor = AutomotiveCardBorder,
                    focusedContainerColor = RoadInk,
                    unfocusedContainerColor = RoadInk
                ),
                shape = RoundedCornerShape(14.dp)
            )
            Spacer(modifier = Modifier.height(16.dp))

            Button(
                onClick = { onSelect(destinationName.ifBlank { "Selected destination" }, GeoPoint(requireNotNull(latitude), requireNotNull(longitude))) },
                enabled = canSelectDestination,
                modifier = Modifier.fillMaxWidth().height(52.dp),
                shape = RoundedCornerShape(16.dp),
                colors = ButtonDefaults.buttonColors(containerColor = PurpleAI, disabledContainerColor = AutomotiveCardBorder, contentColor = Color.White)
            ) {
                Text("START JOURNEY", fontWeight = FontWeight.ExtraBold)
            }
        }
    }
}
