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

#include "string.h"
#include "lwip/sockets.h"

#define TIME_STEP 0.05   // s
#define TIME_INTERVAL 50 // ms

extern PID_t pid_motor[NUM_MOTORS];

extern float encoder_rpm[NUM_MOTORS];

extern LPF encoder_lpf[NUM_MOTORS];

static int socket_fd = -1;

#if USE_FUZZY_PID == 1
#include "fuzzy_control.h"

FuzzyPID fuzzy_pid_controllers[NUM_MOTORS];

float kp_adj[NUM_MOTORS] = {0};
float ki_adj[NUM_MOTORS] = {0};
float kd_adj[NUM_MOTORS] = {0};

float error_adj[NUM_MOTORS] = {0};
float delta_error_adj[NUM_MOTORS] = {0};

void fuzzy_pid_handler_init(void)
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        // Khởi tạo với các khoảng giá trị mặc định - điều chỉnh dựa trên hệ thống của bạn
        fuzzy_pid_init(&fuzzy_pid_controllers[i],
                       0.1f, 5.0f,       // Kp range
                       0.01f, 1.0f,      // Ki range
                       0.0f, 0.6f,       // Kd range
                       -30.0f, 30.0f,    // Error range (RPM)
                       -300.0f, 300.0f); // Delta error range (RPM/s)
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

#if USE_FUZZY_PID == 1
    error_adj[index] = pid->setpoint - feedback;
    // Tính toán thay đổi lỗi (delta_error)
    delta_error_adj[index] = (error_adj[index] - pid->prev_error) / TIME_STEP;

    // Lấy thông số PID điều chỉnh từ bộ điều khiển fuzzy
    fuzzy_pid_compute(&fuzzy_pid_controllers[index], error_adj[index], delta_error_adj[index],
                      &kp_adj[index], &ki_adj[index], &kd_adj[index]);

    // Sử dụng các tham số đã được điều chỉnh fuzzy
    pid->integral += error_adj[index] * TIME_STEP;
    float derivative = delta_error_adj[index];
    derivative = pid->beta_coeff * pid->last_derivative + (1 - pid->beta_coeff) * derivative;

    float output = kp_adj[index] * error_adj[index] + ki_adj[index] * pid->integral + kd_adj[index] * derivative;

    pid->prev_error = error_adj[index];

#else
    float error = pid->setpoint - feedback;
    pid->integral += error * TIME_STEP;
    float derivative = (error - pid->prev_error) / TIME_STEP;
    derivative = pid->beta_coeff * pid->last_derivative + (1 - pid->beta_coeff) * derivative;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    pid->prev_error = error;
#endif

    pid->last_derivative = derivative;
    return output + feedback;
}

void update_rpm(float *encoder_rpm, float *pid_rpm)
{
    int64_t start_time = esp_timer_get_time();
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        pid_rpm[i] = pid_compute(&pid_motor[i], encoder_rpm[i], i);
    }
    int64_t end_time = esp_timer_get_time();
    int64_t execution_time = end_time - start_time;

#if USE_FUZZY_PID == 1

    // Tạo chuỗi JSON theo định dạng yêu cầu
    char json_string[512];
    snprintf(json_string, sizeof(json_string),
             "{\"type\":\"fuzzy\",\"data\":{"
             "\"Motor1\":[%.4f,%.4f,%.4f],"
             "\"Motor2\":[%.4f,%.4f,%.4f],"
             "\"Motor3\":[%.4f,%.4f,%.4f],"
             "\"Error\":[%.2f,%.2f,%.2f],"
             "\"Delta\":[%.2f,%.2f,%.2f],"
             "\"Exc Time\":%lld"
             "}}\n",
             kp_adj[0], ki_adj[0], kd_adj[0],
             kp_adj[1], ki_adj[1], kd_adj[1],
             kp_adj[2], ki_adj[2], kd_adj[2],
             error_adj[0], error_adj[1], error_adj[2],
             delta_error_adj[0], delta_error_adj[1], delta_error_adj[2],
             execution_time);

    // Gửi dữ liệu lên server
    if (send(socket_fd, json_string, strlen(json_string), 0) < 0)
    {
        ESP_LOGW("Fuzzy", "Cannot send data: Invalid socket");
    }
    // printf("D1: %.4f -  D2: %.4f -  D3: %.4f \n", delta_error_adj[0], delta_error_adj[1], delta_error_adj[2]);
#endif
}
void pid_task(void *pvParameters)
{
    ESP_LOGI("PID", "PID Task Started");

    int socket = *(int *)pvParameters;

    socket_fd = socket;

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