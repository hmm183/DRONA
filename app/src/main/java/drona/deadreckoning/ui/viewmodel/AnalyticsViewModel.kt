package drona.deadreckoning.ui.viewmodel

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.StateFlow
import drona.deadreckoning.domain.repository.NavigationRepository
import drona.deadreckoning.domain.state.AnalyticsState

class AnalyticsViewModel(
    private val repository: NavigationRepository
) : ViewModel() {

    val analyticsState: StateFlow<AnalyticsState> = repository.analyticsState
    val navigationState: StateFlow<drona.deadreckoning.domain.state.NavigationState> = repository.navigationState
    val gnssState: StateFlow<drona.deadreckoning.domain.state.GNSSState> = repository.gnssState
}
