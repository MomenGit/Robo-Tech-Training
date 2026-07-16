class IMUModel:
    def __init__(self) -> None:
        self.current_angle = 0
        self.set_ref_angle()

    def set_ref_angle(self):
        self.ref_angle = self.current_angle

    def calculate_angle_deviation(self, target, deviation):
        while self.current_angle < target:
            self.current_angle += deviation
            yield self.current_angle
