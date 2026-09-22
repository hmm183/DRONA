package drona.deadreckoning.ui.viewmodel

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.StateFlow
import drona.deadreckoning.domain.repository.NavigationRepository
import drona.deadreckoning.domain.state.SensorState

class SensorsViewModel(
    private val repository: NavigationRepository
) : ViewModel() {

    val sensorState: StateFlow<SensorState> = repository.sensorState
}
