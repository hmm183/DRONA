package drona.deadreckoning.ui.viewmodel

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.StateFlow
import drona.deadreckoning.domain.repository.NavigationRepository
import drona.deadreckoning.domain.state.SessionState

class SessionsViewModel(
    private val repository: NavigationRepository
) : ViewModel() {

    val sessionState: StateFlow<SessionState> = repository.sessionState
}
