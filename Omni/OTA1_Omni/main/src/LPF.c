#include "LPF.h"
#include <stdlib.h>
#include <string.h>

// Hệ số của bộ lọc (dùng chung)

// Hàm khởi tạo bộ lọc
void LPF_Init(LPF *filter, float *a, float *b, float time_interval)
{
    memcpy(filter->a_coeffs, a, sizeof(filter->a_coeffs));
    memcpy(filter->b_coeffs, b, sizeof(filter->b_coeffs));
    filter->time_interval = time_interval;
}

#define Motor_Respone_In_50ms 1.0f // 30% so với Setpoint mỗi 50ms
void LPF_Clear(LPF *filter, float rpm)
{
    // memset(filter->x_prev, rpm, sizeof(filter->x_prev));
    // memset(filter->y_prev, rpm, sizeof(filter->y_prev));
    // rpm = filter->time_interval * rpm * 0.033; // 0.033 = Pulse per round / 60 * 1000

    /*-----------------*/

    // filter->x_prev += Motor_Respone_In_50ms * rpm;
    // filter->y_prev += Motor_Respone_In_50ms * rpm;

    return;
}

// Hàm áp dụng bộ lọc
float LPF_Apply(LPF *filter, float x)
{

    float y = filter->b_coeffs[0] * x +
              filter->b_coeffs[1] * filter->x_prev +
              filter->a_coeffs[0] * filter->y_prev;

    filter->x_prev = x;

    filter->y_prev = y;

    return y;
}
