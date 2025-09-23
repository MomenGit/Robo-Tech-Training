class PWMMapper:
    min_pwm = 1100.0
    max_pwm = 1900.0
    mid_pwm = (max_pwm + min_pwm) / 2

    @staticmethod
    def map_to_pwm(value: float) -> float:
        """
        Convert value (-1 to 1) into PWM signal.
        value = -1 → pwm_min
        value = 0  → pwm_mid
        value = 1  → pwm_max
        """

        # Clamp value between -1 and 1
        clamped_value = max(-1.0, min(1.0, value))

        mid_pwm = (PWMMapper.min_pwm + PWMMapper.max_pwm) // 2
        pwm_range = (PWMMapper.max_pwm - PWMMapper.min_pwm)

        return float(mid_pwm + clamped_value * (pwm_range/2))
