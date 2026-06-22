// imu_processor.h
#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    //float accel_scale;
    //float gyro_scale;
    float alpha;
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
    float angle_pitch;
} IMUProcessor;

/* Initialize the complementary filter states */
void imu_init(IMUProcessor *proc, float alpha);

/* Configure calibration offsets dynamically */
void imu_set_offsets(IMUProcessor *proc, float ox, float oy, float oz);

/* <<< HIGHLIGHT: Processes pre-scaled floating point values instead of raw bytes >>> */
void imu_update_pitch(IMUProcessor *proc, float ax, float ay, float az, float gy, float dt);

/* Converts Euler pitch angles into standard ROS 2 geometry quaternions */
void imu_get_quaternion(float pitch, float *w, float *x, float *y, float *z);

#endif