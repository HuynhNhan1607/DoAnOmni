#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "position_handler.h"
#include "position_controller.h"
#include "omni_control.h"
#include "sys_config.h"

static const char *TAG = "POS_CTRL";

// Target position (x, y only)
static float target_x = 0.0f;
static float target_y = 0.0f;
static bool position_reached = true;

// Control parameters
static float gain_xy = 0.5f;        // Proportional gain for position
static float max_velocity = 0.15f;  // Max linear velocity (m/s)
static float pos_tolerance = 0.02f; // Position tolerance (5cm)

// Task handle
static TaskHandle_t controller_task_handle = NULL;

// Control period
#define CONTROL_PERIOD_MS 100

// Limit value between min and max
float limit_value(float value, float min_val, float max_val)
{
    if (value > max_val)
        return max_val;
    if (value < min_val)
        return min_val;
    return value;
}

// Set target position (x, y only)
void set_target_position(float x, float y)
{
    target_x = x;
    target_y = y;
    position_reached = false;

    ESP_LOGI(TAG, "New target set: (%.3f, %.3f)", x, y);
}

// Check if target position has been reached
bool is_target_reached(void)
{
    return position_reached;
}

// Position control task
void position_control_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    RobotPos_t current_pos;

    float error_x, error_y;
    float vel_x, vel_y;
    float distance;

    while (1)
    {
        // Get current position
        get_robot_position(&current_pos);

        // Calculate errors
        error_x = target_x - current_pos.pos_x;
        error_y = target_y - current_pos.pos_y;

        // Calculate distance to target
        distance = sqrtf(error_x * error_x + error_y * error_y);

        // Check if target is reached
        if (distance <= pos_tolerance)
        {
            position_reached = true;
            set_control(0.0f, 0.0f, 0.0f); // Stop robot
            set_control_velocity(0.0f, 0.0f);
            ESP_LOGI(TAG, "Target position reached");
        }
        else
        {
            position_reached = false;

            // Calculate velocities (simple proportional control)
            vel_x = gain_xy * error_x;
            vel_y = gain_xy * error_y;

            // Limit linear velocity
            float vel_magnitude = sqrtf(vel_x * vel_x + vel_y * vel_y);
            if (vel_magnitude > max_velocity)
            {
                float scale = max_velocity / vel_magnitude;
                vel_x *= scale;
                vel_y *= scale;
            }
            set_control_velocity(vel_x, vel_y);
            // Send velocity commands to robot (no angular velocity)
            set_control(vel_x, vel_y, 0.0f);

            ESP_LOGI(TAG, "Pos: (%.3f, %.3f), Error: (%.3f, %.3f), Vel: (%.3f, %.3f)",
                     current_pos.pos_x, current_pos.pos_y,
                     error_x, error_y,
                     vel_x, vel_y);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

// Start position controller
void start_position_controller(void)
{

    if (controller_task_handle == NULL)
    {
        xTaskCreatePinnedToCore(
            position_control_task,
            "pos_controller",
            4096,
            NULL,
            10,
            &controller_task_handle,
            1);
        ESP_LOGW(TAG, "Position controller started");
    }
    else
    {
        ESP_LOGW(TAG, "Position controller already running");
    }
}

// Stop position controller
void stop_position_controller(void)
{
    if (controller_task_handle != NULL)
    {
        vTaskDelete(controller_task_handle);
        controller_task_handle = NULL;
        ESP_LOGI(TAG, "Position controller stopped");

        // Stop the robot
        set_control(0.0f, 0.0f, 0.0f);
    }
}