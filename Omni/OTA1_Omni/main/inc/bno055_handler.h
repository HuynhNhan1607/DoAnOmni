#ifndef BNO055_HANDLER_H
#define BNO055_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// Event bits
#define BNO055_DATA_RECEIVED_BIT BIT1

// Event group handle
extern EventGroupHandle_t bno055_event_group;

// Initialize BNO055 handler
void bno055_start(int *socket);

// Data access functions
float get_heading(void);
void get_accel(float *accel_x, float *accel_y, float *accel_z);
void get_gyro_raw(float *gyro_x, float *gyro_y, float *gyro_z);
void get_quaternion(float *quat_w, float *quat_x, float *quat_y, float *quat_z);
bool get_motion_status(void);

#endif /* BNO055_HANDLER_H */