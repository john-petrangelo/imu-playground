#include <gtest/gtest.h>

#include "common.h"

float const MAX_DELTA = 0.01;

TEST(AttitudeTest, flat_N) {
 Vector accel(0.0, 0.0,  1.0);
 Vector mag  (0.0, 1.0, -0.5);

 Vector exp_euler(0.0, 1.0, 0.0);
 Vector exp_ihat(1.0, 0.0, 0.0);
 Vector exp_jhat(0.0, 1.0, 0.0);
 Vector exp_khat(0.0, 0.0, 1.0);

 Attitude attitude = get_attitude_from_accel_mag(accel, mag);
 EXPECT_EQ(exp_euler, attitude.euler);
 EXPECT_EQ(exp_ihat, attitude.ihat);
 EXPECT_EQ(exp_jhat, attitude.jhat);
 EXPECT_EQ(exp_khat, attitude.khat);
}


TEST(AttitudeTest, flat_N2) {
 // { [IN] ax, ay, az, mx, my, mz, [OUT] ix, iy, iz, jx, jy, jz, kx, ky, kz}
 float flat_data[][15] = {
  {0, 0, 1,  0.00, 1.00, -0.5,    1.00, 0.00, 0,   0.00, 1.00, 0,  0, 0, 1},
  {0, 0, 1,  0.50, 0.87, -0.5,    0.87, 0.50, 0,  -0.50, 0.87, 0,  0, 0, 1},
  {0, 0, 1,  0.87, 0.50, -0.5,    0.50, 0.87, 0,  -0.87, 0.50, 0,  0, 0, 1}
// 0 0 1 1.00 0.00 -0.5 0.00 1.00 0 -1.00 0.00 0 0 0 1
// 0 0 1 0.87 -0.50 -0.5 -0.50 0.87 0 -0.87 -0.50 0 0 0 1
// 0 0 1 0.50 -0.87 -0.5 -0.87 0.50 0 -0.50 -0.87 0 0 0 1
// 0 0 1 0.00 -1.00 -0.5 -1.00 0.00 0 0.00 -1.00 0 0 0 1
// 0 0 1 -0.50 -0.87 -0.5 -0.87 -0.50 0 0.50 -0.87 0 0 0 1
// 0 0 1 -0.87 -0.50 -0.5 -0.50 -0.87 0 0.87 -0.50 0 0 0 1
// 0 0 1 -1.00 0.00 -0.5 0.00 -1.00 0 1.00 0.00 0 0 0 1
// 0 0 1 -0.87 0.50 -0.5 0.50 -0.87 0 0.87 0.50 0 0 0 1
// 0 0 1 -0.50 0.87 -0.5 0.87 -0.50 0 0.50 0.87 0 0 0 1
 };

  for (int i = 0; i < 3; ++i) {
  	float (&d)[15] = flat_data[i];

    Vector accel(d[0], d[1], d[2]);
    Vector mag  (d[3], d[4], d[5]);

    Vector exp_ihat(d[6], d[7], d[8]);
    Vector exp_jhat(d[9], d[10], d[11]);
    Vector exp_khat(d[12], d[13], d[14]);
    Vector exp_euler(exp_ihat.y, exp_jhat.y, exp_khat.y);

    Attitude attitude = get_attitude_from_accel_mag(accel, mag);
    EXPECT_EQ(exp_euler, attitude.euler);
    EXPECT_EQ(exp_ihat, attitude.ihat);
    EXPECT_EQ(exp_jhat, attitude.jhat);
    EXPECT_EQ(exp_khat, attitude.khat);
  }
}
