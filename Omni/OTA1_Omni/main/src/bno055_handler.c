#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"

#include "lwip/sockets.h"
#include "bno055.h"
#include "bno055_handler.h"
#include "nvs_handler.h"
#include "sys_config.h"
#include "string.h"

// UART includes
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG_IMU = "BNO055_Handler";

// Data structures for storing the received values
bno055_euler_t euler = {0};
bno055_quaternion_t quat = {0};
bno055_vec3_t lin_accel = {0};
bno055_vec3_t gyro_raw = {0};
static bool is_moving = false;

// Socket for data forwarding (if needed)
static int global_socket = -1;

// Synchronization
static SemaphoreHandle_t imu_data_mutex = NULL;
EventGroupHandle_t bno055_event_group = NULL;

// Queue for UART data
#define BNO055_QUEUE_SIZE 10
#define BNO055_MAX_MSG_SIZE 512
typedef struct
{
    char data[BNO055_MAX_MSG_SIZE];
    size_t len;
} bno055_queue_item_t;

static QueueHandle_t bno055_queue = NULL;
static TaskHandle_t bno055_task_handle = NULL;
static TaskHandle_t uart_task_handle = NULL;

// UART configurations
#define BNO055_UART_PORT UART_NUM_1
#define BNO055_UART_RX_PIN GPIO_NUM_16
#define BNO055_UART_TX_PIN GPIO_NUM_17
#define BNO055_UART_BAUD_RATE 115200

// Parse the received JSON data
bool parse_bno055_json(const char *json_data)
{
    cJSON *root = cJSON_Parse(json_data);
    if (root == NULL)
    {
        ESP_LOGE(TAG_IMU, "Failed to parse JSON");
        return false;
    }

    bool success = false;
    cJSON *type = cJSON_GetObjectItem(root, "type");

    // Check if this is BNO055 data
    if (cJSON_IsString(type) && strcmp(type->valuestring, "bno055") == 0)
    {
        cJSON *data = cJSON_GetObjectItem(root, "data");
        if (data)
        {
            if (xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                // Parse euler data
                cJSON *euler_array = cJSON_GetObjectItem(data, "euler");
                if (euler_array && cJSON_GetArraySize(euler_array) >= 3)
                {
                    euler.heading = cJSON_GetArrayItem(euler_array, 0)->valuedouble;
                    euler.pitch = cJSON_GetArrayItem(euler_array, 1)->valuedouble;
                    euler.roll = cJSON_GetArrayItem(euler_array, 2)->valuedouble;
                }

                // Parse linear acceleration data
                cJSON *lin_accel_array = cJSON_GetObjectItem(data, "lin_accel");
                if (lin_accel_array && cJSON_GetArraySize(lin_accel_array) >= 3)
                {
                    lin_accel.x = cJSON_GetArrayItem(lin_accel_array, 0)->valuedouble;
                    lin_accel.y = cJSON_GetArrayItem(lin_accel_array, 1)->valuedouble;
                    lin_accel.z = cJSON_GetArrayItem(lin_accel_array, 2)->valuedouble;
                }

                // Parse gyro raw data
                cJSON *gyro_raw_array = cJSON_GetObjectItem(data, "gyro_raw");
                if (gyro_raw_array && cJSON_GetArraySize(gyro_raw_array) >= 3)
                {
                    gyro_raw.x = cJSON_GetArrayItem(gyro_raw_array, 0)->valuedouble;
                    gyro_raw.y = cJSON_GetArrayItem(gyro_raw_array, 1)->valuedouble;
                    gyro_raw.z = cJSON_GetArrayItem(gyro_raw_array, 2)->valuedouble;
                }

                // Parse status
                cJSON *status = cJSON_GetObjectItem(data, "status");
                if (status && cJSON_IsString(status))
                {
                    is_moving = (strcmp(status->valuestring, "moving") == 0);
                }

                xSemaphoreGive(imu_data_mutex);

                // Set event flag for data received
                if (bno055_event_group != NULL)
                {
                    xEventGroupSetBits(bno055_event_group, BNO055_DATA_RECEIVED_BIT);
                }

                success = true;
            }
        }
    }

    cJSON_Delete(root);
    return success;
}

// Hàm xử lý dữ liệu từ queue
void bno055_process_uart_data(const char *data, size_t len)
{
    // Parse the received JSON data
    if (parse_bno055_json(data))
    {
        ESP_LOGD(TAG_IMU, "Successfully processed BNO055 data");

        // Forward to socket if needed
        if (global_socket >= 0)
        {
            if (send(global_socket, data, len, 0))
            {
                ESP_LOGD(TAG_IMU, "Socket BNO055: %s", data);
            }
            else
            {
                ESP_LOGE(TAG_IMU, "Failed Socket BNO055");
            }
        }
    }
    else
    {
        ESP_LOGW(TAG_IMU, "Failed to parse BNO055 data");
    }
}

// Task xử lý dữ liệu BNO055 từ queue
void bno055_process_task(void *pvParameters)
{
    bno055_queue_item_t item;

    while (1)
    {
        // Đợi dữ liệu từ queue
        if (xQueueReceive(bno055_queue, &item, portMAX_DELAY) == pdTRUE)
        {
            // Xử lý dữ liệu
            bno055_process_uart_data(item.data, item.len);
        }
    }
}

// UART event task để đọc dữ liệu từ UART
void bno055_uart_task(void *pvParameters)
{
    uint8_t data[BNO055_MAX_MSG_SIZE];

    while (1)
    {
        int len = uart_read_bytes(BNO055_UART_PORT, data, sizeof(data) - 1, pdMS_TO_TICKS(20));

        if (len > 0)
        {
            data[len] = '\0'; // Kết thúc chuỗi

            // ESP_LOGI(TAG_IMU, "Raw UART data (%d bytes): %s", len, data);

            // Kiểm tra có đúng là gói từ BNO055 không
            if (strstr((char *)data, "\"type\":\"bno055\""))
            {
                bno055_queue_item_t item;

                size_t copy_len = len < BNO055_MAX_MSG_SIZE - 1 ? len : BNO055_MAX_MSG_SIZE - 1;
                memcpy(item.data, data, copy_len);
                item.data[copy_len] = '\0';
                item.len = copy_len;

                if (xQueueSend(bno055_queue, &item, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG_IMU, "Queue full, dropping BNO055 data packet");
                }
                else
                {
                    ESP_LOGD(TAG_IMU, "Queued: %s", item.data);
                }
            }
            else
            {
                ESP_LOGW(TAG_IMU, "Invalid BNO055 data: %s", data);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Get heading data
float get_heading()
{
    float result = 0.0f;
    if (imu_data_mutex != NULL && xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        result = -euler.heading; // Keep same sign convention
        xSemaphoreGive(imu_data_mutex);
    }
    return result;
}

// Get acceleration data
void get_accel(float *accel_x, float *accel_y, float *accel_z)
{
    if (imu_data_mutex != NULL && xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        *accel_x = -lin_accel.x;
        *accel_y = -lin_accel.y;
        *accel_z = lin_accel.z;
        xSemaphoreGive(imu_data_mutex);
    }
    else
    {
        *accel_x = *accel_y = *accel_z = 0.0f;
    }
}

// Get gyroscope data
void get_gyro_raw(float *gyro_x, float *gyro_y, float *gyro_z)
{
    if (imu_data_mutex != NULL && xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        *gyro_x = gyro_raw.x;
        *gyro_y = gyro_raw.y;
        *gyro_z = gyro_raw.z;
        xSemaphoreGive(imu_data_mutex);
    }
    else
    {
        *gyro_x = *gyro_y = *gyro_z = 0.0f;
    }
}

// Get motion status
bool get_motion_status(void)
{
    bool status = false;
    if (imu_data_mutex != NULL && xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        status = is_moving;
        xSemaphoreGive(imu_data_mutex);
    }
    return status;
}

// Get quaternion data
void get_quaternion(float *quat_w, float *quat_x, float *quat_y, float *quat_z)
{
    if (imu_data_mutex != NULL && xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        *quat_w = quat.w;
        *quat_x = quat.x;
        *quat_y = quat.y;
        *quat_z = quat.z;
        xSemaphoreGive(imu_data_mutex);
    }
    else
    {
        *quat_w = 1.0f;
        *quat_x = *quat_y = *quat_z = 0.0f;
    }
}

// Initialize the BNO055 handler with UART
void bno055_start(int *socket)
{
    global_socket = *socket;

    ESP_LOGI(TAG_IMU, "Initializing BNO055 UART mode...");

    // Cấu hình UART
    uart_config_t uart_config = {
        .baud_rate = BNO055_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    // Cấu hình và cài đặt UART
    ESP_ERROR_CHECK(uart_param_config(BNO055_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BNO055_UART_PORT, BNO055_UART_TX_PIN, BNO055_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(BNO055_UART_PORT, BNO055_MAX_MSG_SIZE * 2, 0, 0, NULL, 0));

    // Tạo queue cho dữ liệu BNO055
    bno055_queue = xQueueCreate(BNO055_QUEUE_SIZE, sizeof(bno055_queue_item_t));
    if (bno055_queue == NULL)
    {
        ESP_LOGE(TAG_IMU, "Failed to create BNO055 queue");
        return;
    }

    // Create event group
    bno055_event_group = xEventGroupCreate();
    if (bno055_event_group != NULL)
    {
        xEventGroupClearBits(bno055_event_group, BNO055_CALIBRATED_BIT);
        xEventGroupClearBits(bno055_event_group, BNO055_DATA_RECEIVED_BIT);
        // Set calibrated bit since we're receiving from another device
        xEventGroupSetBits(bno055_event_group, BNO055_CALIBRATED_BIT);
    }

    // Create mutex for data protection
    imu_data_mutex = xSemaphoreCreateMutex();
    if (imu_data_mutex == NULL)
    {
        ESP_LOGE(TAG_IMU, "Failed to create IMU data mutex");
        return;
    }

    // Tạo task xử lý UART
    BaseType_t uart_task_created = xTaskCreate(
        bno055_uart_task,
        "bno055_uart",
        4096,             // Kích thước stack
        NULL,             // Tham số
        6,                // Ưu tiên cao hơn task xử lý
        &uart_task_handle // Task handle
    );

    if (uart_task_created != pdPASS)
    {
        ESP_LOGE(TAG_IMU, "Failed to create BNO055 UART task");
        return;
    }

    // Tạo task xử lý dữ liệu BNO055
    BaseType_t process_task_created = xTaskCreate(
        bno055_process_task,
        "bno055_process",
        4096,               // Kích thước stack
        NULL,               // Tham số
        5,                  // Priority
        &bno055_task_handle // Task handle
    );

    if (process_task_created != pdPASS)
    {
        ESP_LOGE(TAG_IMU, "Failed to create BNO055 process task");
        return;
    }

    ESP_LOGI(TAG_IMU, "BNO055 UART mode initialized (RX: %d, Baud: %d). Waiting for data...",
             BNO055_UART_RX_PIN, BNO055_UART_BAUD_RATE);
}