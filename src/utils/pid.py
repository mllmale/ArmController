from typing import Tuple


class PIDController:
    """
    Controlador PID Discreto
    Ideal para malhas de controle em sistemas robóticos físicos.
    """

    def __init__(self, kp: float, ki: float, kd: float, min_out: float = -1.0, max_out: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.min_out = min_out
        self.max_out = max_out

        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, setpoint: float, pv: float, dt: float) -> float:
        """
        Calcula o sinal de controle com base no erro e no delta tempo.
        """
        if dt <= 0.0:
            return 0.0

        error = setpoint - pv

        # proporcional
        proportional = self.kp * error

        # integral
        self.integral += error * dt

        if self.ki > 0:
            max_integral = max(abs(self.max_out), abs(self.min_out)) / self.ki
            self.integral = max(-max_integral, min(self.integral, max_integral))

        integral_term = self.ki * self.integral

        # derivativo
        derivative = self.kd * ((error - self.previous_error) / dt)

        self.previous_error = error

        control = proportional + integral_term + derivative

        return max(self.min_out, min(control, self.max_out))

    def reset(self) -> None:
        """
        Zera a memória do PID. 
        """
        self.integral = 0.0
        self.previous_error = 0.0