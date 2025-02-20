#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "omni_control.h"
#include "motor_handler.h"

#define WHEEL_RADIUS 0.03   // Bán kính bánh xe (m)
#define ROBOT_RADIUS 0.1528 // Khoảng cách từ tâm robot đến bánh xe (m)
#define WEIGHT 2.0          // Trọng lượng robot (kg)
#define PI 3.14159265359

// Quy đổi từ rad/s sang RPM
int m_s_to_rpm(float m_s)
{
    return (m_s * 1000) / PI; // 1 round = 3PI/50 m
}

float rad_s_to_rpm(float rad_s)
{
    return (rad_s * 60) / (2 * PI);
}
int rpm_to_pulse(float rpm)
{
    return rpm * 5.115; // 1023/200 = 5.115
}

void calculate_wheel_speeds(const RobotParams *params, float *omega1, float *omega2, float *omega3)
{
    // Ma trận H^-1
    float H_inv[3][3] = {
        {-sin(params->theta), cos(params->theta), params->robot_radius},
        {-sin(M_PI / 3 - params->theta), -cos(M_PI / 3 - params->theta), params->robot_radius},
        {sin(M_PI / 3 + params->theta), -cos(M_PI / 3 + params->theta), params->robot_radius}};

    // Tính toán vận tốc góc
    *omega1 = (H_inv[0][0] * params->dot_x + H_inv[0][1] * params->dot_y + H_inv[0][2] * params->dot_theta) / params->wheel_radius;
    *omega2 = (H_inv[1][0] * params->dot_x + H_inv[1][1] * params->dot_y + H_inv[1][2] * params->dot_theta) / params->wheel_radius;
    *omega3 = (H_inv[2][0] * params->dot_x + H_inv[2][1] * params->dot_y + H_inv[2][2] * params->dot_theta) / params->wheel_radius;

    printf("Omega1: %f, Omega2: %f, Omega3: %f\n", *omega1, *omega2, *omega3);
}
// Task chính để điều khiển robot
void omni_control(float dot_x, float dot_y, float dot_theta)
{
    float rpm1, rpm2, rpm3;
    int pulse1, pulse2, pulse3;

    float omega1, omega2, omega3;
    RobotParams robot = {
        .dot_x = dot_x,
        .dot_y = dot_y,
        .dot_theta = dot_theta,
        .theta = 0,
        .wheel_radius = WHEEL_RADIUS,
        .robot_radius = ROBOT_RADIUS};
    // Tính vận tốc bánh xe theo RPM
    calculate_wheel_speeds(&robot, &omega1, &omega2, &omega3);

    rpm1 = rad_s_to_rpm(omega1);
    rpm2 = rad_s_to_rpm(omega2);
    rpm3 = rad_s_to_rpm(omega3);

    pulse1 = rpm_to_pulse(rpm1);
    pulse2 = rpm_to_pulse(rpm2);
    pulse3 = rpm_to_pulse(rpm3);

    // Điều khiển động cơ theo tốc độ RPM
    // printf("RPM1: %.2f, RPM2: %.2f, RPM3: %.2f \n", rpm1, rpm2, rpm3);
    // printf("Pulse1: %d, Pulse2: %d, Pulse3: %d \n", pulse1, pulse2, pulse3);
    // Must be update same time
    if (pulse1 < 0)
    {
        pulse1 = -pulse1;
        set_motor_speed(1, 0, pulse1);
    }
    else
    {
        set_motor_speed(1, 1, pulse1);
    }
    if (pulse2 < 0)
    {
        pulse2 = -pulse2;
        set_motor_speed(2, 0, pulse2);
    }
    else
    {
        set_motor_speed(2, 1, pulse2);
    }
    if (pulse3 < 0)
    {
        pulse3 = -pulse3;
        set_motor_speed(3, 0, pulse3);
    }
    else
    {
        set_motor_speed(3, 1, pulse3);
    }
}
