package drona.deadreckoning.ui.viewmodel

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.StateFlow
import drona.deadreckoning.domain.repository.NavigationRepository
import drona.deadreckoning.domain.state.GNSSState

class GNSSViewModel(
    private val repository: NavigationRepository
) : ViewModel() {

    val gnssState: StateFlow<GNSSState> = repository.gnssState
}
