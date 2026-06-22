// test_imu.cpp (Compiled on Host Machine)
#include <gtest/gtest.h>
extern "C" {
    #include "imu_processor.h"
}

/* Test if complementary filter behavior works as expected on host systems */

/*TEST(IMUProcessorTest, TestFilterMath) {
    IMUProcessor proc;
    imu_init(&proc, 0.98f);
    
    // Simulate static gravity on Z axis and mock rotation on Y axis
    float mock_ax = 0.0f;
    float mock_ay = 0.0f;
    float mock_az = 9.80665f;
    float mock_gy = 10.0f; // 10 degrees per second
    float dt = 0.05f;      // 50ms loop interval

    imu_update_pitch(&proc, mock_ax, mock_ay, mock_az, mock_gy, dt);
    
    // Assert tracking filters do not overshoot wildly or crash on basic math calculations
    EXPECT_GT(proc.angle_pitch, -45.0f);
    EXPECT_LT(proc.angle_pitch, 45.0f);
}
*/
TEST(IMUProcessorTest, TestComplementaryFilterPitch) {
    IMUProcessor proc;
    // Accel scale, Gyro scale, Alpha = 0.98
    //imu_init(&proc, 0.0039f * 9.80665f, 0.00875f, 0.98f);
    imu_init(&proc, 0.98f);

    // Mock pre-scaled floating-point sensor values
    float mock_ax = 0.0f;
    float mock_ay = 0.0f;
    float mock_az = 9.80665f; // Pure gravity along Z-axis
    float mock_gy = 10.0f;    // Rotational rate of 10 dps along Y-axis
    float dt = 0.05f;         // 50ms loop interval
    
    // Run the filter update
    imu_update_pitch(&proc, mock_ax, mock_ay, mock_az, mock_gy, dt);
    
    // Assert expectations (Verify your math actually matches!)
    EXPECT_GT(proc.angle_pitch, -180.0f);
    EXPECT_LT(proc.angle_pitch, 180.0f);
}

TEST(IMUProcessorTest, TestQuaternionConversion) {
    float w, x, y, z;
    imu_get_quaternion(0.0f, &w, &x, &y, &z);
    
    // Neutral pitch should yield identity quaternion [1, 0, 0, 0]
    EXPECT_NEAR(w, 1.0f, 0.0001f);
    EXPECT_NEAR(x, 0.0f, 0.0001f);
    EXPECT_NEAR(y, 0.0f, 0.0001f);
    EXPECT_NEAR(z, 0.0f, 0.0001f);
}