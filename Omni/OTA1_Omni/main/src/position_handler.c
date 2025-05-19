#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "position_handler.h"
#include "encoder_handler.h"
#include "bno055_handler.h"
#include "sys_config.h"

static const char *TAG_POS = "Position_Handler";

// Vị trí robot
static RobotPos_t robot_position = {0.0f, 0.0f, 0.0f};
static RobotVel_t robot_velocity = {0.0f, 0.0f, 0.0f}; // Vận tốc robot

// Mutex cho việc truy cập vị trí
static SemaphoreHandle_t position_mutex = NULL;

extern float encoder_rpm[NUM_MOTORS];

// Task handle
static TaskHandle_t fk_task_handle = NULL;

static float control_vel_x = 0.0f; // Vận tốc điều khiển theo trục x
static float control_vel_y = 0.0f; // Vận tốc điều khiển theo trục y

#define DT 0.1f // chu kỳ lấy mẫu (100ms)

float rpm_to_rad_s(float rpm)
{
    return (rpm * 2.0f * M_PI) / 60.0f;
}

void set_control_velocity(float vel_x, float vel_y)
{
    control_vel_x = vel_x;
    control_vel_y = vel_y;
}

// Hàm lấy vị trí hiện tại của robot
void get_robot_position(RobotPos_t *pos)
{
    if (position_mutex != NULL && xSemaphoreTake(position_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        pos->pos_x = robot_position.pos_x;
        pos->pos_y = robot_position.pos_y;
        pos->pos_theta = robot_position.pos_theta;
        xSemaphoreGive(position_mutex);
    }
    else
    {
        ESP_LOGW(TAG_POS, "get_robot_position() - Failed mutex");
    }
}

// Hàm set vị trí robot
void set_robot_position(RobotPos_t *pos)
{
    if (position_mutex != NULL && xSemaphoreTake(position_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        robot_position.pos_x = pos->pos_x;
        robot_position.pos_y = pos->pos_y;
        robot_position.pos_theta = pos->pos_theta;
        xSemaphoreGive(position_mutex);
        ESP_LOGW(TAG_POS, "Robot position reset to (%.3f, %.3f, %.3f)",
                 pos->pos_x, pos->pos_y, pos->pos_theta);
    }
    else
    {
        ESP_LOGW(TAG_POS, "set_robot_position() - Failed mutex");
    }
}

void forward_kinematics_task(void *pvParameters)
{
    int sock = *(int *)pvParameters;

    char json_buffer[256];

    // Ticks cho chu kỳ cố định
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    float omega_wheels[3];
    float theta = 0.0f;
    float accel_x, accel_y, accel_z = 0.0f;

    int64_t previous_time = 0;
    int64_t current_time;
    float dt_actual;

    previous_time = esp_timer_get_time();

    bool simpson_initialized = 0;
    while (1)
    {
#if USE_THETA == 1
        robot_position.pos_theta = get_heading() * M_PI / 180.0f; // Chuyển độ sang radian
        theta = robot_position.pos_theta;
#else
        robot_position.pos_theta = 0.0f; // Không sử dụng cảm biến BNO055
        theta = 0.0f;
#endif

        for (int i = 0; i < NUM_MOTORS; i++)
        {
            omega_wheels[i] = rpm_to_rad_s(encoder_rpm[i]);
        }

        float H[3][3] = {
            {-2.0f / 3.0f * sinf(theta), -2.0f / 3.0f * cosf(theta + M_PI / 6.0f), 2.0f / 3.0f * sinf(theta + M_PI / 3.0f)},
            {2.0f / 3.0f * cosf(theta), -2.0f / 3.0f * sinf(theta + M_PI / 6.0f), -2.0f / 3.0f * cosf(theta + M_PI / 3.0f)},
            {1.0f / (3.0f * ROBOT_RADIUS), 1.0f / (3.0f * ROBOT_RADIUS), 1.0f / (3.0f * ROBOT_RADIUS)}};

        // Tính vận tốc trong hệ tọa độ của robot (body frame)
        float vx = H[0][0] * omega_wheels[0] * WHEEL_RADIUS +
                   H[0][1] * omega_wheels[1] * WHEEL_RADIUS +
                   H[0][2] * omega_wheels[2] * WHEEL_RADIUS;

        float vy = H[1][0] * omega_wheels[0] * WHEEL_RADIUS +
                   H[1][1] * omega_wheels[1] * WHEEL_RADIUS +
                   H[1][2] * omega_wheels[2] * WHEEL_RADIUS;

        // float omega = H[2][0] * omega_wheels[0] * WHEEL_RADIUS +
        //               H[2][1] * omega_wheels[1] * WHEEL_RADIUS +
        //               H[2][2] * omega_wheels[2] * WHEEL_RADIUS;

        current_time = esp_timer_get_time();
        // dt_actual = (current_time - previous_time) / 1000000.0f;
        dt_actual = DT; // Sử dụng chu kỳ cố định 100ms

        if (position_mutex != NULL && xSemaphoreTake(position_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            get_accel(&accel_x, &accel_y, &accel_z);
            // Lưu giá trị vận tốc mới nhất

            robot_position.pos_x += vx * dt_actual;
            robot_position.pos_y += vy * dt_actual;

            robot_velocity.vel_x = vx;
            robot_velocity.vel_y = vy;

            xSemaphoreGive(position_mutex);
        }
        else
        {
            ESP_LOGW(TAG_POS, "forward_kinematics_task() - Failed mutex");
        }

        previous_time = current_time;

        ESP_LOGD(TAG_POS, "Velocities: vx=%.4f, vy=%.4f", vx, vy);
        ESP_LOGD(TAG_POS, "Position: x=%.4f, y=%.4f, theta=%.4f",
                 robot_position.pos_x, robot_position.pos_y, robot_position.pos_theta);

        if (sock >= 0)
        {
            snprintf(json_buffer, sizeof(json_buffer),
                     "{"
                     "\"id\":\"%s\","
                     "\"type\":\"position\","
                     "\"data\":{"
                     "\"position\":[%.4f,%.4f,%.4f],"
                     "\"velocity\":[%.4f,%.4f],"
                     "\"acceleration\":[%.4f,%.4f,%.4f],"
                     "\"vel_control\":[%.4f,%.4f]"
                     "}"
                     "}\n",
                     ID_ROBOT,
                     robot_position.pos_x, robot_position.pos_y,
                     robot_position.pos_theta * 180.0f / M_PI,
                     robot_velocity.vel_x,
                     robot_velocity.vel_y,
                     accel_x, accel_y, accel_z,
                     control_vel_x,
                     control_vel_y);

            if (send(sock, json_buffer, strlen(json_buffer), 0) < 0)
            {
                ESP_LOGE(TAG_POS, "Failed to send position data");
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

void start_forward_kinematics(int *socket)
{
    if (position_mutex == NULL)
    {
        position_mutex = xSemaphoreCreateMutex();
    }

    RobotPos_t init_pos = {1.8f, 1.2f, 0.0f}; // mét
    set_robot_position(&init_pos);

    if (fk_task_handle == NULL)
    {
        xTaskCreatePinnedToCore(
            forward_kinematics_task,
            "forward_kinematics",
            4096,
            socket,
            9,
            &fk_task_handle,
            0);
        ESP_LOGI(TAG_POS, "Forward Kinematics task created");
    }
    else
    {
        ESP_LOGW(TAG_POS, "Forward Kinematics task already running");
    }
}

void stop_forward_kinematics()
{
    if (fk_task_handle != NULL)
    {
        vTaskDelete(fk_task_handle);
        fk_task_handle = NULL;
        ESP_LOGI(TAG_POS, "Forward Kinematics task stopped");
    }
}