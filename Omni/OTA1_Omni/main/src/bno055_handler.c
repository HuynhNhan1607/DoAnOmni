#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bno055.h"
#include "bno055_handler.h"
#include "nvs_handler.h"

TaskHandle_t blink_led_task_handle = NULL;
TaskHandle_t ndof_task_handle = NULL;

bno055_euler_t euler;
bno055_quaternion_t quat;

static bno055_config_t bno_conf;
static i2c_number_t i2c_num = 0;
static gpio_num_t led_gpio = GPIO_NUM_2;

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

    printf("BNO055 sensor error: %02x\n", err_code);

    // Đóng kết nối với cảm biến
    esp_err_t err = bno055_close(i2c_num);
    printf("bno055_close() returned 0x%02X\n", err);

    // Tạo task reinit_sensor để khởi tạo lại cảm biến
    xTaskCreatePinnedToCore(
        reinit_sensor,
        "reinit_sensor",
        2048,
        NULL,
        10,
        &reinit_task_handle,
        1);
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
        err = bno055_open(i2c_num, &bno_conf, OPERATION_MODE_NDOF);
        printf("bno055_open() returned 0x%02X \n", err);

        if (err == ESP_OK)
        {
            // Đã khởi tạo thành công, xóa task blink_led
            if (blink_led_task_handle != NULL)
            {
                vTaskDelete(blink_led_task_handle);
                gpio_set_level(led_gpio, 0);
                blink_led_task_handle = NULL;
            }
            xTaskCreatePinnedToCore(ndof_task,
                                    "ndof_task",
                                    2048,
                                    &i2c_num,
                                    10,
                                    &ndof_task_handle,
                                    1);

            vTaskDelete(NULL);
            break;
        }
        else
        {
            // Khởi tạo thất bại, thử lại sau 20 giây
            printf("Failed to open BNO055, retrying......\n");
            vTaskDelay(pdMS_TO_TICKS(REINIT_TIME));
        }
    }
}

void ndof_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    esp_err_t err;
    int64_t time_mks, time_mks_after;
    int time_bno;

    calib_status_t calib_status;
    i2c_number_t i2c_num = *(i2c_number_t *)pvParameters;
    bool was_calibrated = false;
    bno055_offsets_t offsets;
    while (1)
    {
        time_mks = esp_timer_get_time();

        if (bno055_is_fully_calibrated(i2c_num, &calib_status) && !was_calibrated)
        {
            was_calibrated = true;
            printf("Calib - Sys: %d, Gyro: %d, Accel: %d, Mag: %d\n",
                   calib_status.sys, calib_status.gyro,
                   calib_status.accel, calib_status.mag);

            err = bno055_get_offsets(i2c_num, &offsets);
            if (err == ESP_OK)
            {
                printf("Accel offset: %d %d %d\t Magnet: %d %d %d\t Gyro: %d %d %d Acc_Radius: %d\t Mag_Radius: %d\n", offsets.accel_offset_x, offsets.accel_offset_y, offsets.accel_offset_z, offsets.mag_offset_x, offsets.mag_offset_y, offsets.mag_offset_z, offsets.gyro_offset_x, offsets.gyro_offset_y, offsets.gyro_offset_z, offsets.accel_radius, offsets.mag_radius);
            }
            err = nvs_save_bno055_calibration(&offsets);
            if (err == ESP_OK)
            {
                printf("Calibration data saved successfully\n");
            }
            else
            {
                printf("Failed to save calibration data: %d\n", err);
            }
        }
        err = bno055_get_quaternion(i2c_num, &quat);
        if (err != ESP_OK)
        {
            printf("bno055_get_quaternion() returned error: %02x\n", err);
            handle_sensor_error(i2c_num, err);
            vTaskDelete(NULL);
            break;
        }

        err = bno055_quaternion_to_euler(&quat, &euler);
        if (err != ESP_OK)
        {
            printf("bno055_quaternion_to_euler() returned error: %02x\n", err);
        }
        time_mks_after = esp_timer_get_time();

        // printf("Euler angles: Yaw: %.6f, Pitch: %.6f, Roll: %.6f\n",
        //        euler.heading, euler.pitch, euler.roll);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BNO_POLLING_MS));
    }
}

void bno055_start(void)
{
    printf("\n\n\n");
    printf("********************\n");
    printf("  BNO055 NDOF test\n");
    printf("********************\n");
    esp_err_t err;
    err = nvs_init();
    if (err != ESP_OK)
    {
        printf("Failed to initialize NVS\n");
    }

    //    esp_log_level_set("*", ESP_LOG_INFO);

    err = bno055_set_default_conf(&bno_conf);
    err = bno055_open(i2c_num, &bno_conf, OPERATION_MODE_NDOF);
    printf("bno055_open() returned 0x%02X \n", err);

    if (err != ESP_OK)
    {
        printf("Program terminated! returned 0x%02X\n", err);
        err = bno055_close(i2c_num);
        printf("bno055_close() returned 0x%02X \n", err);
        printf("Failed to open BNO055, starting reinit process\n");
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
        // create task on the APP CPU (CPU_1)
        err = xTaskCreatePinnedToCore(ndof_task,         // task function
                                      "ndof_task",       // task name for debugging
                                      2048,              // stack size bytes
                                      &i2c_num,          // pointer to params to pass
                                      10,                // task priority. configMAX_PRIORITIES == 25, so 24 is the highest. 0 is idle
                                      &ndof_task_handle, // returned task handle
                                      1);                // CPU to use 1 means APP_CPU
    }
}