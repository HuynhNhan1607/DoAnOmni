#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "lwip/sockets.h"
#include "bno055.h"
#include "bno055_handler.h"
#include "nvs_handler.h"
#include "sys_config.h"
#include "string.h"

#include <math.h>

#define BNO_MODE OPERATION_MODE_NDOF

static const char *TAG_IMU = "BNO055_Handler";

TaskHandle_t blink_led_task_handle = NULL;
TaskHandle_t ndof_task_handle = NULL;
TaskHandle_t calib_task_handle = NULL;

bno055_euler_t euler;
bno055_quaternion_t quat;
bno055_vec3_t lin_accel;
bno055_vec3_t gravity;

static bno055_config_t bno_conf;
static i2c_number_t i2c_num = 0;
static gpio_num_t led_gpio = GPIO_NUM_2;

static int global_socket = -1;

static bool calibration_complete = false;
static float yaw_offset = 0.0f;
static bool apply_yaw_offset = false;

float adjusted_heading = 0.0f;

float get_heading()
{
    return adjusted_heading;
}
void blink_led_task(void *pvParameters)
{
    gpio_num_t led_gpio = *(gpio_num_t *)pvParameters;
    gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(led_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(led_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void handle_sensor_error(i2c_number_t i2c_num, esp_err_t err_code)
{
    TaskHandle_t reinit_task_handle = NULL;

    ESP_LOGE(TAG_IMU, "BNO055 sensor error: %02x", err_code);

    // Đóng kết nối với cảm biến
    esp_err_t err = bno055_close(i2c_num);
    ESP_LOGW(TAG_IMU, "bno055_close() returned 0x%02X", err);

    // Tạo task reinit_sensor để khởi tạo lại cảm biến
    xTaskCreatePinnedToCore(
        reinit_sensor,
        "reinit_sensor",
        2048,
        NULL,
        10,
        &reinit_task_handle,
        1);

    if (ndof_task_handle != NULL)
    {
        vTaskSuspend(ndof_task_handle);
    }
}

void send_calibration_notification(int sock, calib_status_t status)
{
    char calib_json[256];
    snprintf(calib_json, sizeof(calib_json),
             "{"
             "\"id\":%d,"
             "\"type\":\"bno055\","
             "\"data\":{"
             "\"event\":\"calibration_complete\","
             "\"status\":{\"sys\":%d,\"gyro\":%d,\"accel\":%d,\"mag\":%d}"
             "}"
             "}\n",
             ID_ROBOT, status.sys, status.gyro, status.accel, status.mag);

    if (send(sock, calib_json, strlen(calib_json), 0) < 0)
    {
        ESP_LOGE(TAG_IMU, "Failed to send calibration notification");
    }
    ESP_LOGI(TAG_IMU, "Calibration notification sent successfully");
}

void bno055_set_yaw_reference(void)
{
    ESP_LOGI(TAG_IMU, "Setting yaw reference point...");

    esp_err_t err;
    float current_heading = 0.0f;
    float prev_heading = 0.0f;
    int stable_count = 0;
    const int REQUIRED_STABLE_COUNT = 20;    // Số lần đọc liên tiếp cần ổn định
    const float STABILITY_THRESHOLD = 0.01f; // Độ chênh lệch cho phép

    vTaskDelay(pdMS_TO_TICKS(1000));

    err = bno055_get_fusion_data(i2c_num, &quat, &lin_accel, &gravity);
    if (err == ESP_OK)
    {
        err = bno055_quaternion_to_euler(&quat, &euler);
        if (err == ESP_OK)
        {
            prev_heading = euler.heading;
        }
    }
    for (int i = 0; i < 50; i++)
    {
        err = bno055_get_fusion_data(i2c_num, &quat, &lin_accel, &gravity);
        if (err != ESP_OK)
        {
            stable_count = 0; // Reset nếu đọc lỗi
            continue;
        }
        err = bno055_quaternion_to_euler(&quat, &euler);
        if (err != ESP_OK)
        {
            stable_count = 0; // Reset nếu đọc lỗi
            continue;
        }
        current_heading = euler.heading;
        float diff = fabs(current_heading - prev_heading);

        ESP_LOGI(TAG_IMU, "Reading %d: %.2f, diff: %.2f", i, current_heading, diff);

        if (diff <= STABILITY_THRESHOLD)
        {
            stable_count++;

            ESP_LOGD(TAG_IMU, "Stable reading #%d: %.2f", stable_count, current_heading);

            if (stable_count >= REQUIRED_STABLE_COUNT)
            {
                // Sử dụng giá trị hiện tại làm offset
                yaw_offset = current_heading;
                apply_yaw_offset = true;

                ESP_LOGI(TAG_IMU, "Yaw reference set to %.2f after %d stable readings",
                         yaw_offset, stable_count);
                return;
            }
        }
        else
        {
            ESP_LOGD(TAG_IMU, "Unstable change detected: %.2f", diff);
            stable_count = 0;
        }
        prev_heading = current_heading;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGW(TAG_IMU, "Could not find stable heading after 50 attempts");
}

float get_adjusted_heading(float raw_heading)
{
    if (!apply_yaw_offset)
    {
        return raw_heading;
    }
    float adjusted = raw_heading - yaw_offset;
    while (adjusted > 180.0f)
        adjusted -= 360.0f;
    while (adjusted < -180.0f)
        adjusted += 360.0f;

    return adjusted;
}

void reinit_sensor(void *pvParameters)
{
    esp_err_t err;

    // Tạo task blink_led khi sensor lỗi
    if (blink_led_task_handle == NULL)
    {
        xTaskCreatePinnedToCore(blink_led_task,
                                "blink_led_task",
                                1024,
                                &led_gpio,
                                5,
                                &blink_led_task_handle,
                                0);
    }

    vTaskDelay(pdMS_TO_TICKS(REINIT_TIME));

    while (1)
    {
        err = bno055_open(i2c_num, &bno_conf, BNO_MODE);
        ESP_LOGI(TAG_IMU, "bno055_open() returned 0x%02X", err);
        if (err == ESP_OK)
        {

            if (blink_led_task_handle != NULL)
            {
                vTaskDelete(blink_led_task_handle);
                gpio_set_level(led_gpio, 0);
                blink_led_task_handle = NULL;
            }
            if (ndof_task_handle != NULL)
            {
                vTaskResume(ndof_task_handle);
            }
            else
            {
                // Tạo task nếu chưa tồn tại
                xTaskCreatePinnedToCore(ndof_task,
                                        "ndof_task",
                                        4096,
                                        NULL,
                                        10,
                                        &ndof_task_handle,
                                        1);
            }
            vTaskDelete(NULL);
            break;
        }
        else
        {
            // Khởi tạo thất bại, thử lại sau 20 giây
            ESP_LOGW(TAG_IMU, "Failed to open BNO055, retrying......");
            vTaskDelay(pdMS_TO_TICKS(REINIT_TIME));
        }
    }
}
// Thêm task mới cho calibration
void calibration_task(void *pvParameters)
{
    int sock = global_socket;
    esp_err_t err;
    calib_status_t calib_status;
    bno055_offsets_t offsets;
    bool was_calibrated = false;

    ESP_LOGI(TAG_IMU, "Calibration task started");

    // Khởi tạo LED và đảm bảo nó tắt
    gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(led_gpio, 0);

    // Lặp kiểm tra trạng thái hiệu chuẩn
    while (!was_calibrated)
    {
        // Kiểm tra xem cảm biến đã được hiệu chuẩn đầy đủ chưa
        if (bno055_is_fully_calibrated(i2c_num, &calib_status, BNO_MODE))
        {
            was_calibrated = true;

            ESP_LOGW(TAG_IMU, "Calib - Sys: %d, Gyro: %d, Accel: %d, Mag: %d",
                     calib_status.sys, calib_status.gyro,
                     calib_status.accel, calib_status.mag);

            // Đọc giá trị offset
            err = bno055_get_offsets(i2c_num, &offsets);
            if (err == ESP_OK)
            {
                ESP_LOGW(TAG_IMU, "Accel offset: %d %d %d    Magnet: %d %d %d    Gyro: %d %d %d Acc_Radius: %d    Mag_Radius: %d",
                         offsets.accel_offset_x, offsets.accel_offset_y, offsets.accel_offset_z,
                         offsets.mag_offset_x, offsets.mag_offset_y, offsets.mag_offset_z,
                         offsets.gyro_offset_x, offsets.gyro_offset_y, offsets.gyro_offset_z,
                         offsets.accel_radius, offsets.mag_radius);
            }

            // Lưu dữ liệu hiệu chuẩn vào NVS
            err = nvs_save_bno055_calibration(&offsets);
            if (err == ESP_OK)
            {
                ESP_LOGW(TAG_IMU, "Calibration data saved successfully");

                // Bật LED chỉ thị
                gpio_set_level(led_gpio, 1);

                // Gửi thông báo hiệu chuẩn hoàn tất
                if (sock >= 0)
                {
                    send_calibration_notification(sock, calib_status);
                }
            }
            else
            {
                ESP_LOGE(TAG_IMU, "Failed to save calibration data: %d", err);
            }
        }

        // Đợi một khoảng thời gian trước khi kiểm tra lại
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); // Chuan bi san sang truoc khi do

    // bno055_set_yaw_reference();

    xTaskCreatePinnedToCore(ndof_task,
                            "ndof_task",
                            4096,
                            NULL,
                            10,
                            &ndof_task_handle,
                            1);
    // Task hoàn thành và xóa chính nó
    ESP_LOGI(TAG_IMU, "Calibration task complete");
    calib_task_handle = NULL;
    vTaskDelete(NULL);
}
/* Gửi Fusion Data
void ndof_task(void *pvParameters)
{
    int sock = global_socket;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    esp_err_t err;
    int64_t time_mks, time_mks_after;
    int time_bno;

    char json_buffer[512];

    while (1)
    {

        time_mks = esp_timer_get_time();

        // Đọc dữ liệu quaternion, linear accel và gravity trong một lần gọi
        err = bno055_get_fusion_data(i2c_num, &quat, &lin_accel, &gravity);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_IMU, "bno055_get_fusion_data() returned error: %02x", err);
            handle_sensor_error(i2c_num, err);
            taskYIELD();
            continue;
        }

        err = bno055_quaternion_to_euler(&quat, &euler);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_IMU, "bno055_quaternion_to_euler() returned error: %02x", err);
        }
        adjusted_heading = get_adjusted_heading(euler.heading);

        time_mks_after = esp_timer_get_time();
        time_bno = time_mks_after - time_mks;

        snprintf(json_buffer, sizeof(json_buffer),
                 "{"
                 "\"id\":%d,"
                 "\"type\":\"bno055\","
                 "\"data\":{"
                 "\"time\":%10d,"
                 "\"euler\":[%.4f,%.4f,%.4f],"
                 "\"lin_accel\":[%.4f,%.4f,%.4f],"
                 "\"gravity\":[%.4f,%.4f,%.4f]"
                 "}"
                 "}\n",
                 ID_ROBOT, time_bno, adjusted_heading, euler.pitch, euler.roll,
                 lin_accel.x, lin_accel.y, lin_accel.z, gravity.x, gravity.y, gravity.z);

        if (send(sock, json_buffer, strlen(json_buffer), 0) < 0)
        {
            ESP_LOGE(TAG_IMU, "Failed to send IMU data");
            printf("Failed to send IMU data\n");
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BNO_POLLING_MS));
    }
}
*/
void ndof_task(void *pvParameters)
{
    int sock = global_socket;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    esp_err_t err;
    int64_t time_mks, time_mks_after;
    int time_bno;

    char json_buffer[512];

    while (1)
    {
        time_mks = esp_timer_get_time();

        err = bno055_get_orientation_data(i2c_num, &quat, &euler);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_IMU, "bno055_get_orientation_data() returned error: %02x", err);
            handle_sensor_error(i2c_num, err);
            taskYIELD();
            continue;
        }

        // adjusted_heading = get_adjusted_heading(euler.heading);

        time_mks_after = esp_timer_get_time();
        time_bno = time_mks_after - time_mks;

        snprintf(json_buffer, sizeof(json_buffer),
                 "{"
                 "\"id\":%d,"
                 "\"type\":\"bno055\","
                 "\"data\":{"
                 "\"time\":%10d,"
                 "\"euler\":[%.4f,%.4f,%.4f],"
                 "\"quaternion\":[%.4f,%.4f,%.4f,%.4f]"
                 "}"
                 "}\n",
                 ID_ROBOT, time_bno,
                 euler.heading, euler.pitch, euler.roll,
                 quat.w, quat.x, quat.y, quat.z);

        if (send(sock, json_buffer, strlen(json_buffer), 0) < 0)
        {
            ESP_LOGE(TAG_IMU, "Failed to send IMU data");
            printf("Failed to send IMU data\n");
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BNO_POLLING_MS));
    }
}

void bno055_start(int *socket)
{
    global_socket = *socket;
    printf("\n\n\n");
    printf("********************\n");
    printf("  BNO055 NDOF test\n");
    printf("********************\n");

    esp_err_t err;
    err = nvs_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_IMU, "Failed to initialize NVS");
    }

    err = bno055_set_default_conf(&bno_conf);
    err = bno055_open(i2c_num, &bno_conf, BNO_MODE);
    ESP_LOGI(TAG_IMU, "bno055_open() returned 0x%02X", err);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_IMU, "Program terminated! returned 0x%02X", err);
        err = bno055_close(i2c_num);
        ESP_LOGW(TAG_IMU, "bno055_close() returned 0x%02X", err);
        ESP_LOGW(TAG_IMU, "Failed to open BNO055, starting reinit process");
        xTaskCreatePinnedToCore(reinit_sensor,
                                "reinit_sensor",
                                2048,
                                NULL,
                                10,
                                NULL,
                                1);
    }
    else
    {
        // Reset calibration status
        calibration_complete = false;

        // Tạo task hiệu chuẩn với ưu tiên cao hơn
        xTaskCreatePinnedToCore(calibration_task,
                                "calib_task",
                                2048,
                                NULL,
                                11, // Ưu tiên cao hơn ndof_task
                                &calib_task_handle,
                                1);
    }
}