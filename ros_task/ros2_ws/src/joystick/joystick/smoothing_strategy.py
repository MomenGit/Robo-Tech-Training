from abc import ABC, abstractmethod
from typing import List
import math


class SmoothingStrategy(ABC):
    """Abstract base class for smoothing strategies"""

    @abstractmethod
    def smooth(self, current_value: float, target_value: float, steps: int) -> List[float]:
        """
        Generate smoothed values between current and target

        Args:
            current_value: Starting value
            target_value: Target value to reach
            steps: Number of steps to take between values

        Returns:
            List of smoothed values including target but not current
        """
        pass


class LinearSmoothing(SmoothingStrategy):
    """Linear interpolation smoothing strategy"""

    def smooth(self, current_value: float, target_value: float, steps: int) -> List[float]:
        if steps <= 0:
            return [target_value]

        step_size = (target_value - current_value) / steps
        return [current_value + (step_size * (i + 1)) for i in range(steps)]


class ExponentialSmoothing(SmoothingStrategy):
    """Exponential smoothing strategy"""

    def __init__(self, smoothing_factor: float = 0.3):
        """
        Args:
            smoothing_factor: Higher values mean faster convergence (0 < factor < 1)
        """
        self.smoothing_factor = max(0.01, min(0.99, smoothing_factor))

    def smooth(self, current_value: float, target_value: float, steps: int) -> List[float]:
        if steps <= 0:
            return [target_value]

        smoothed_values = []
        current = current_value

        for _ in range(steps):
            current = current + self.smoothing_factor * \
                (target_value - current)
            smoothed_values.append(current)

        return smoothed_values


class CubicSmoothing(SmoothingStrategy):
    """Cubic easing smoothing for smoother transitions"""

    def smooth(self, current_value: float, target_value: float, steps: int) -> List[float]:
        if steps <= 0:
            return [target_value]

        def ease_in_out_cubic(t: float) -> float:
            """Cubic easing function"""
            t = max(0, min(1, t))
            if t < 0.5:
                return 4 * t * t * t
            else:
                return 1 - math.pow(-2 * t + 2, 3) / 2

        total_change = target_value - current_value
        return [current_value + (ease_in_out_cubic((i + 1) / steps) * total_change)
                for i in range(steps)]


class SmoothingStrategyFactory:
    """Factory class for creating smoothing strategies"""

    @staticmethod
    def create_strategy(strategy_type: str, **kwargs) -> SmoothingStrategy:
        """
        Create a smoothing strategy based on type

        Args:
            strategy_type: 'linear', 'exponential', or 'cubic'
            **kwargs: Additional parameters for the strategy

        Returns:
            SmoothingStrategy instance
        """
        strategies = {
            'linear': LinearSmoothing,
            'exponential': ExponentialSmoothing,
            'cubic': CubicSmoothing
        }

        if strategy_type not in strategies:
            raise ValueError(f"Unknown strategy type: {strategy_type}")

        return strategies[strategy_type](**kwargs)
