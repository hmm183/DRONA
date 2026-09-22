package drona.deadreckoning.ui.viewmodel

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.StateFlow
import drona.deadreckoning.domain.repository.NavigationRepository
import drona.deadreckoning.domain.state.NavigationState
import drona.deadreckoning.domain.state.GNSSState
import drona.deadreckoning.domain.state.AIState
import drona.deadreckoning.domain.state.SensorState
import drona.deadreckoning.domain.state.SessionState

class HomeViewModel(
    private val repository: NavigationRepository
) : ViewModel() {

    val navigationState: StateFlow<NavigationState> = repository.navigationState
    val sessionState: StateFlow<SessionState> = repository.sessionState
    val gnssState: StateFlow<GNSSState> = repository.gnssState
    val aiState: StateFlow<AIState> = repository.aiState
    val sensorState: StateFlow<SensorState> = repository.sensorState

    fun startNavigation() {
        repository.startNavigation()
    }

    fun stopNavigation() {
        repository.stopNavigation()
    }
}
