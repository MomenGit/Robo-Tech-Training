from typing import List
from cv2.typing import MatLike


class HistoryManager:
    """Class to manage the history of canvas states for undo/redo functionality."""

    def __init__(self):
        self.history: List[MatLike] = []
        self.redo_stack: List[MatLike] = []
        self.current_index = -1

    def add_state(self, canvas_state: MatLike):
        """Add a new state to the history and clear the redo stack."""
        self.history.append(canvas_state.copy())
        self.redo_stack.clear()

    def undo(self):
        """Revert to the previous state in history."""
        if len(self.history) > 1:  # Need at least 2 states to undo
            current_state = self.history.pop()
            self.redo_stack.append(current_state)
            # Return a copy of the previous state
            return self.history[-1].copy()
        return None

    def redo(self):
        """Reapply a state from the redo stack."""
        if self.redo_stack:
            state = self.redo_stack.pop()
            self.history.append(state.copy())
            return state.copy()
        return None
