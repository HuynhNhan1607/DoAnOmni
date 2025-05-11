#include "pid_handler.h"
#include "encoder_handler.h"
#include "motor_handler.h"
#include "LPF.h"
#include "sys_config.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#define TIME_STEP 0.05   // s
#define TIME_INTERVAL 50 // ms

extern PID_t pid_motor[NUM_MOTORS];

extern float encoder_rpm[NUM_MOTORS];

extern LPF encoder_lpf[NUM_MOTORS];

#if USE_FUZZY_PID == 1
#include "fuzzy_control.h"

FuzzyPID fuzzy_pid_controllers[NUM_MOTORS];

void fuzzy_pid_handler_init(void)
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        // Khởi tạo với các khoảng giá trị mặc định - điều chỉnh dựa trên hệ thống của bạn
        fuzzy_pid_init(&fuzzy_pid_controllers[i],
                       0.1f, 10.0f,     // Kp range
                       0.01f, 1.0f,     // Ki range
                       0.0f, 1.0f,      // Kd range
                       -100.0f, 100.0f, // Error range (RPM)
                       -50.0f, 50.0f);  // Delta error range (RPM/s)
    }
    ESP_LOGI("Fuzzy", "Fuzzy PID initialized");
}

#endif

void pid_init(PID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0.0;
    pid->integral = 0.0;
    pid->last_derivative = 0.0;
    pid->beta_coeff = 0.7;

#if USE_FUZZY_PID == 1
    fuzzy_pid_handler_init();
    ESP_LOGW("Fuzzy", "Fuzzy PID control");
#else
    ESP_LOGW("PID", "Standard PID control");
#endif
}

void pid_set_setpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
    // Clear Previous PID values
    pid->prev_error = 0.0;
    pid->integral = 0.0;
}

float pid_compute(PID_t *pid, float feedback, int index)
{
    int64_t start_time = esp_timer_get_time(); // Thêm đo thời gian
    float error = pid->setpoint - feedback;

#if USE_FUZZY_PID == 1
    // Tính toán thay đổi lỗi (delta_error)
    float delta_error = (error - pid->prev_error) / TIME_STEP;

    // Lấy thông số PID điều chỉnh từ bộ điều khiển fuzzy
    float kp_adj, ki_adj, kd_adj;
    fuzzy_pid_compute(&fuzzy_pid_controllers[index], error, delta_error,
                      &kp_adj, &ki_adj, &kd_adj);

    // Sử dụng các tham số đã được điều chỉnh fuzzy
    pid->integral += error * TIME_STEP;
    float derivative = delta_error;
    derivative = pid->beta_coeff * pid->last_derivative + (1 - pid->beta_coeff) * derivative;

    float output = kp_adj * error + ki_adj * pid->integral + kd_adj * derivative;

    // Đo thời gian thực thi và log
    int64_t end_time = esp_timer_get_time();
    int64_t execution_time = end_time - start_time;

    // Ghi log thông số đã điều chỉnh theo định kỳ
    static int log_counter = 0;
    if (++log_counter >= 4)
    {
        ESP_LOGW("Fuzzy", "Motor %d - Fuzzy PID: Kp: %.3f, Ki: %.3f, Kd: %.3f, Error: %.2f, Execution time: %lld us",
                 index + 1, kp_adj, ki_adj, kd_adj, error, execution_time);
        log_counter = 0;
    }
#else
    pid->integral += error * TIME_STEP;
    float derivative = (error - pid->prev_error) / TIME_STEP;
    derivative = pid->beta_coeff * pid->last_derivative + (1 - pid->beta_coeff) * derivative;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // Đo thời gian cho PID thông thường
    int64_t end_time = esp_timer_get_time();
    int64_t execution_time = end_time - start_time;

    static int std_log_counter = 0;
    if (index == 0 && ++std_log_counter >= 100)
    {
        ESP_LOGI("PID", "Standard PID execution time: %lld us", execution_time);
        std_log_counter = 0;
    }
#endif
    pid->prev_error = error;
    pid->last_derivative = derivative;
    return output + feedback;
}

void update_rpm(float *encoder_rpm, float *pid_rpm)
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        pid_rpm[i] = pid_compute(&pid_motor[i], encoder_rpm[i], i);
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
            ESP_LOGI("PID", "ENC: %.2f %.2f %.2f || PID RPM: %.2f %.2f %.2f ", encoder_rpm[0], encoder_rpm[1], encoder_rpm[2], pid_rpm[0], pid_rpm[1], pid_rpm[2]);
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