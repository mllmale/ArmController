def pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt):
    error = setpoint - pv
    integral += error * dt
    derivative = (error - previous_error) / dt
    control = kp * error + ki * integral + kd * derivative
    return control, error, integral