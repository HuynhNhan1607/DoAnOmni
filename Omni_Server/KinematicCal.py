import math
import numpy as np

# Constants (update these based on your robot's actual parameters)
WHEEL_RADIUS = 0.03  # meters
ROBOT_RADIUS = 0.1543  # meters

# L values represent the distance from center to wheel (perpendicular to wheel)
L1 = ROBOT_RADIUS
L2 = ROBOT_RADIUS
L3 = ROBOT_RADIUS

def calculate_wheel_speeds(vx, vy, omega=0, theta=0):
    """
    Calculate wheel angular velocities based on desired robot velocities
    vx: velocity in x direction (m/s)
    vy: velocity in y direction (m/s)
    omega: rotational velocity (rad/s), default 0
    theta: robot orientation (radians), default 0
    """
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    
    # Using ideal kinematics model
    H_inv = np.array([
        [-sin_theta, cos_theta, L1],
        [0.5 * sin_theta - 0.866025 * cos_theta, -0.866025 * sin_theta - 0.5 * cos_theta, L2],
        [0.5 * sin_theta + 0.866025 * cos_theta, 0.866025 * sin_theta - 0.5 * cos_theta, L3]
    ])
    
    # Calculate angular velocities (rad/s)
    omega1 = (H_inv[0, 0] * vx + H_inv[0, 1] * vy + H_inv[0, 2] * omega) / WHEEL_RADIUS
    omega2 = (H_inv[1, 0] * vx + H_inv[1, 1] * vy + H_inv[1, 2] * omega) / WHEEL_RADIUS
    omega3 = (H_inv[2, 0] * vx + H_inv[2, 1] * vy + H_inv[2, 2] * omega) / WHEEL_RADIUS
    
    return omega1, omega2, omega3

def rad_s_to_rpm(rad_s):
    """Convert radians per second to RPM"""
    return (rad_s * 60) / (2 * math.pi)

def main():
    print("Omni Robot Wheel RPM Calculator")
    print("===============================")
    
    try:
        vx = float(input("Enter desired velocity in x direction (m/s): "))
        vy = float(input("Enter desired velocity in y direction (m/s): "))
        
        # Optional rotation input
        use_rotation = input("Include rotation? (y/n): ").lower() == 'y'
        omega = 0
        if use_rotation:
            omega = float(input("Enter desired rotational velocity (rad/s): "))
        
        # Calculate angular velocities
        omega1, omega2, omega3 = calculate_wheel_speeds(vx, vy, omega)
        
        # Convert to RPM
        rpm1 = rad_s_to_rpm(omega1)
        rpm2 = rad_s_to_rpm(omega2)
        rpm3 = rad_s_to_rpm(omega3)
        
        print("\nResults:")
        print(f"Wheel 1: {omega1:.2f} rad/s = {rpm1:.2f} RPM")
        print(f"Wheel 2: {omega2:.2f} rad/s = {rpm2:.2f} RPM")
        print(f"Wheel 3: {omega3:.2f} rad/s = {rpm3:.2f} RPM")
        
    except ValueError:
        print("Error: Please enter valid numeric values.")

if __name__ == "__main__":
    main()