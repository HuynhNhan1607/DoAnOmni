import time

def pid_manual(feedback_rpm, setpoint, kp, ki, kd, previous_error=0, integral=0, dt=0.05):
    error = setpoint - feedback_rpm
    integral += error * dt
    derivative = (error - previous_error) / dt
    output = kp * error + ki * integral + kd * derivative
    return output, error, integral

def pid_controller(feedback_rpm, setpoint, kp, ki, kd, sample_time=0.05, use_simple_pid=True):
    if use_simple_pid:
        from simple_pid import PID
        pid = PID(kp, ki, kd, setpoint)
        pid.sample_time = sample_time  # Chu kỳ cập nhật PID
        pid.output_limits = (0, None)  # Giới hạn giá trị đầu ra RPM (nếu cần)
        return pid(feedback_rpm)
    else:
        return pid_manual(feedback_rpm, setpoint, kp, ki, kd)

# Thông số PID
Kp = 5
Ki = 0
Kd = 0

while True:
    setpoint_rpm = float(input("Nhập Setpoint RPM: "))
    feedback_rpm = float(input("Nhập Feedback RPM: "))

    use_simple_pid = 1
    if use_simple_pid:
        output_rpm = pid_controller(feedback_rpm, setpoint_rpm, Kp, Ki, Kd, use_simple_pid=True)
    else:
        output_rpm, _, _ = pid_manual(feedback_rpm, setpoint_rpm, Kp, Ki, Kd)

    print(f"Output RPM after PID control: {output_rpm:.2f}")
