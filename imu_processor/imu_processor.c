// imu_processor.c
#include "imu_processor.h"
#include <math.h>
/*
void imu_init(IMUProcessor *proc, float accel_scale, float gyro_scale, float alpha) {
    proc->alpha = alpha;
    proc->gyro_offset_x = 0.0f;
    proc->gyro_offset_y = 0.0f;
    proc->gyro_offset_z = 0.0f;
    proc->angle_pitch = 0.0f;
}
*/
void imu_init(IMUProcessor *proc, float alpha)
 {
    proc->alpha = alpha;
}

void imu_set_offsets(IMUProcessor *proc, float ox, float oy, float oz) {
    proc->gyro_offset_x = ox;
    proc->gyro_offset_y = oy;
    proc->gyro_offset_z = oz;
}
/*
void imu_update_pitch(IMUProcessor *proc, const uint8_t *accel_raw, const uint8_t *gyro_raw, float dt) {
    int16_t raw_ax = (accel_raw[1] << 8) | accel_raw[0];
    int16_t raw_ay = (accel_raw[3] << 8) | accel_raw[2];
    int16_t raw_az = (accel_raw[5] << 8) | accel_raw[4];

    float ax = raw_ax * proc->accel_scale;
    float ay = raw_ay * proc->accel_scale;
    float az = raw_az * proc->accel_scale;

    int16_t raw_gy = (gyro_raw[3] << 8) | gyro_raw[2];
    float gy_rate = (raw_gy * proc->gyro_scale) - proc->gyro_offset_y;

    float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.295f;   
    proc->angle_pitch = proc->alpha * (proc->angle_pitch + gy_rate * dt) + (1.0f - proc->alpha) * accel_pitch;
}

*/

/* Calculation block */
void imu_update_pitch(IMUProcessor *proc, float ax, float ay, float az, float gy, float dt) {
    // Apply the Exponential Moving Average filter 

    
    float smoothed_ax = (proc->accel_filter_factor * ax) + ((1.0f - proc->accel_filter_factor) * proc->last_accel_pitch);

    // Calculate pitch angle based on accelerometer data 
    float accel_pitch = atan2f(smoothed_ax, sqrtf(ay * ay + az * az)) * 57.295f;   
    
    // Remove the calibrated zero-bias drift offset 
    float gy_rate = gy - proc->gyro_offset_y;
    
    // Complementary filter formula 
    proc->angle_pitch = proc->alpha * (proc->angle_pitch + gy_rate * dt) + (1.0f - proc->alpha) * accel_pitch;
}

/* Convert Euler degree to half-angle radians for quaternion generation */
void imu_get_quaternion(float pitch, float *w, float *x, float *y, float *z) {
    float p = pitch * 0.5f * 0.01745329f;
    *w = cosf(p);
    *x = 0.0f;
    *y = sinf(p);
    *z = 0.0f;
}