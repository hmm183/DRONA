package drona.deadreckoning.ui.viewmodel

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.StateFlow
import drona.deadreckoning.domain.repository.NavigationRepository
import drona.deadreckoning.domain.state.AIState
import drona.deadreckoning.domain.state.GNSSState
import drona.deadreckoning.domain.state.NavigationState
import drona.deadreckoning.domain.state.SensorState

class DiagnosticsViewModel(
    private val repository: NavigationRepository
) : ViewModel() {

    val navigationState: StateFlow<NavigationState> = repository.navigationState
    val sensorState: StateFlow<SensorState> = repository.sensorState
    val gnssState: StateFlow<GNSSState> = repository.gnssState
    val aiState: StateFlow<AIState> = repository.aiState
}
