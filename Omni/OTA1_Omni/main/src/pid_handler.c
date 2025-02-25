#include "pid_handler.h"
#include "encoder_handler.h"
#include "motor_handler.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define TIME_STEP 0.04   // 10ms
#define TIME_INTERVAL 40 // 10ms

extern PID_t pid_motor[NUM_MOTORS];

extern float encoder_rpm[NUM_MOTORS];

void pid_init(PID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0;
    pid->integral = 0;
}

void pid_set_setpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
    // Clear Previous PID values
    pid->prev_error = 0;
    pid->integral = 0;
}

float pid_compute(PID_t *pid, float feedback)
{
    float error = pid->setpoint - feedback;
    pid->integral += error * TIME_STEP;
    float derivative = (error - pid->prev_error) / TIME_STEP;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;
    return output + feedback;
}

void update_rpm(float *encoder_rpm, float *pid_rpm)
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        pid_rpm[i] = pid_compute(&pid_motor[i], encoder_rpm[i]);
    }
}
void pid_task(void *pvParameters)
{
    ESP_LOGI("PID", "PID Task Started");

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_print_time = last_wake_time;

    float pid_rpm[NUM_MOTORS] = {0};

    int pulse[NUM_MOTORS];
    int direction[NUM_MOTORS];

    while (1)
    {
        read_rpm(TIME_INTERVAL);
        update_rpm(encoder_rpm, pid_rpm);

        if (xTaskGetTickCount() - last_print_time >= pdMS_TO_TICKS(1000))
        {
            ESP_LOGW("PID", "ENC: %.2f %.2f %.2f || PID RPM: %.2f %.2f %.2f ", encoder_rpm[0], encoder_rpm[1], encoder_rpm[2], pid_rpm[0], pid_rpm[1], pid_rpm[2]);

            // ESP_LOGW("PID", "PID Pulse: %d %d %d", abs((int)(pid_rpm[0] * 5.11)), abs((int)(pid_rpm[1] * 5.11)), abs((int)(pid_rpm[2] * 5.11)));
            last_print_time = xTaskGetTickCount();
        }
        for (int i = 0; i < NUM_MOTORS; i++)
        {

            pulse[i] = rpm_to_pulse(pid_rpm[i]);

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

        // Sau khi tính toán xong, gửi lệnh đồng thời
        set_motor_speed(1, direction[0], pulse[0]);
        set_motor_speed(2, direction[1], pulse[1]);
        set_motor_speed(3, direction[2], pulse[2]);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TIME_INTERVAL));
    }
}