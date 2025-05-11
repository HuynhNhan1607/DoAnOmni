#include <stdio.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "omni_control.h"
#include "motor_handler.h"
#include "sys_config.h"
#include "pid_handler.h"
#include "LPF.h"
#include "bno055_handler.h"

extern PID_t pid_motor[NUM_MOTORS];
extern LPF encoder_lpf[NUM_MOTORS];

TaskHandle_t wheel_speed_task_handle = NULL;
static float omega[NUM_MOTORS] = {0};

static const char *TAG = "OMNI_CONTROL";

RobotParams robot;

int m_s_to_rpm(float m_s)
{
    return (m_s * 1000) / M_PI; // 1 round = 3PI/50 m
}

float rad_s_to_rpm(float rad_s)
{
    return (rad_s * 60) / (2 * M_PI);
}

/*

*/
void calculate_wheel_speeds(RobotParams *params, float *omega1, float *omega2, float *omega3)
{
    float sin_theta = sin(params->theta);
    float cos_theta = cos(params->theta);

#if IDEAL == 1
    float H_inv[3][3] = {
        {-sin_theta, cos_theta, L1},
        {0.5 * sin_theta - 0.866025 * cos_theta, -0.866025 * sin_theta - 0.5 * cos_theta, L2},
        {0.5 * sin_theta + 0.866025 * cos_theta, 0.866025 * sin_theta - 0.5 * cos_theta, L3}};
#else
    float H_inv[3][3] = {
        {-sin_theta - 0.0032 * cos_theta, -0.0032 * sin_theta + cos_theta, L1},
        {0.465769 * sin_theta - 0.884906 * cos_theta, -0.884906 * sin_theta - 0.465769 * cos_theta, L2},
        {0.58224 * sin_theta + 0.813012 * cos_theta, 0.813012 * sin_theta - 0.58224 * cos_theta, L3}};
#endif

    // Tính toán vận tốc góc
    *omega1 = (H_inv[0][0] * params->dot_x + H_inv[0][1] * params->dot_y + H_inv[0][2] * params->dot_theta) / r1;
    *omega2 = (H_inv[1][0] * params->dot_x + H_inv[1][1] * params->dot_y + H_inv[1][2] * params->dot_theta) / r2;
    *omega3 = (H_inv[2][0] * params->dot_x + H_inv[2][1] * params->dot_y + H_inv[2][2] * params->dot_theta) / r3;

    ESP_LOGD(TAG, "Omega: %.2f, %.2f, %.2f rad/s", *omega1, *omega2, *omega3);
}

void apply_wheel_speeds(void)
{
    float rpm[NUM_MOTORS];
    int pulse[NUM_MOTORS];
    int direction[NUM_MOTORS];

#if NON_PID == 1
    // Chuyển đổi sang RPM và Pulse
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        rpm[i] = rad_s_to_rpm(omega[i]);

        LPF_Clear(&encoder_lpf[i], rpm[i]);
        pulse[i] = rpm_to_pulse(rpm[i]);

        // Xác định hướng động cơ
        if (pulse[i] < 0)
        {
            direction[i] = 0; // Quay ngược
            pulse[i] = -pulse[i];
        }
        else
        {
            direction[i] = 1; // Quay xuôi
        }
    }

    // Gửi lệnh đồng thời
    set_motor_speed(1, direction[0], pulse[0]);
    set_motor_speed(2, direction[1], pulse[1]);
    set_motor_speed(3, direction[2], pulse[2]);
#else
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        rpm[i] = rad_s_to_rpm(omega[i]);
        LPF_Clear(&encoder_lpf[i], rpm[i]);
        // LPF_Clear(&encoder_lpf[i], 0);
        pid_set_setpoint(&pid_motor[i], rpm[i]);
    }
#endif

    ESP_LOGI(TAG, "Applied speeds: %.2f, %.2f, %.2f RPM", rpm[0], rpm[1], rpm[2]);
}

void wheel_speed_calculation_task(void *pvParameters)
{

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
#if USE_THETA == 1
        float current_heading = get_heading();
        robot.theta = (current_heading * M_PI) / 180.0f;
        ESP_LOGD(TAG, "Recalculating with heading: %.2f", current_heading);
#endif
        // Tính toán vận tốc góc mới
        calculate_wheel_speeds(&robot, &omega[0], &omega[1], &omega[2]);

        apply_wheel_speeds();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(RECALCULATION_PERIOD_MS));
    }
}

void set_control(float dot_x, float dot_y, float dot_theta)
{
    // Only update the velocity commands
    robot.dot_x = dot_x;
    robot.dot_y = dot_y;
    robot.dot_theta = dot_theta;

#if USE_THETA == 1
    // Update current heading
    float current_heading = get_heading();
    robot.theta = (current_heading * M_PI) / 180.0f;
#endif
    ESP_LOGD(TAG, "Set control: dot_x=%.4f, dot_y=%.4f, dot_theta=%.4f",
             dot_x, dot_y, dot_theta);
    // Calculate and apply wheel speeds immediately for responsive control
    calculate_wheel_speeds(&robot, &omega[0], &omega[1], &omega[2]);
    apply_wheel_speeds();
}
void omni_init()
{
    robot.dot_x = 0;
    robot.dot_y = 0;
    robot.dot_theta = 0;
    robot.theta = 0;
    robot.wheel_radius = WHEEL_RADIUS;
    robot.robot_radius = ROBOT_RADIUS;
    // Dùng để đáp ứng điều khiển ngay lập tức, không phải đợi task chạy
    // apply_wheel_speeds();
#if USE_THETA == 1
    if (wheel_speed_task_handle == NULL)
    {
        xTaskCreate(wheel_speed_calculation_task,
                    "wheel_speed_task",
                    4096, // Stack size
                    NULL, // Parameter
                    8,    // Priority
                    &wheel_speed_task_handle);

        ESP_LOGD(TAG, "Wheel speed calculation task started");
    }
#endif
}