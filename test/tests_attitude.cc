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

struct AttitudeTestData {
  Vector inAccel;
  Vector inMag;
  Attitude out;
};


TEST(AttitudeTest, flat_N2) {
 // { [IN] ax, ay, az, mx, my, mz, [OUT] ix, iy, iz, jx, jy, jz, kx, ky, kz}
  AttitudeTestData flat_data[] = {
    {
      Vector(0, 0, 1), Vector(0.00, 1.00, -0.5),
      {
        Vector(0, 0, 0),
        Vector(1.00, 0.00, 0), Vector(0.00, 1.00, 0), Vector(0, 0, 1),
        0, 0, 0,
      }
    },
    {
      Vector(0, 0, 1), Vector(0.50, 0.87, -0.5),
      {
        Vector(0, 0, 0),
        Vector(0.87, 0.50, 0), Vector(-0.50, 0.87, 0), Vector(0, 0, 1),
        0, 0, 0,
      }
    },
    {
      Vector(0, 0, 1), Vector(0.87, 0.50, -0.5),
      {
        Vector(0, 0, 0),
        Vector(0.50, 0.87, 0), Vector(-0.87, 0.50, 0), Vector(0, 0, 1),
        0, 0, 0,
      }
    },
  };
// 0 0 1 1.00 0.00 -0.5 0.00 1.00 0 -1.00 0.00 0 0 0 1
// 0 0 1 0.87 -0.50 -0.5 -0.50 0.87 0 -0.87 -0.50 0 0 0 1
// 0 0 1 0.50 -0.87 -0.5 -0.87 0.50 0 -0.50 -0.87 0 0 0 1
// 0 0 1 0.00 -1.00 -0.5 -1.00 0.00 0 0.00 -1.00 0 0 0 1
// 0 0 1 -0.50 -0.87 -0.5 -0.87 -0.50 0 0.50 -0.87 0 0 0 1
// 0 0 1 -0.87 -0.50 -0.5 -0.50 -0.87 0 0.87 -0.50 0 0 0 1
// 0 0 1 -1.00 0.00 -0.5 0.00 -1.00 0 1.00 0.00 0 0 0 1
// 0 0 1 -0.87 0.50 -0.5 0.50 -0.87 0 0.87 0.50 0 0 0 1
// 0 0 1 -0.50 0.87 -0.5 0.87 -0.50 0 0.50 0.87 0 0 0 1

  for (int i = 0; i < 3; ++i) {
    Vector const &inAccel =  flat_data[i].inAccel;
    Vector const &inMag =  flat_data[i].inMag;

    Vector const &exp_euler(flat_data[i].out.euler);
    Vector const &exp_ihat(flat_data[i].out.ihat);
    Vector const &exp_jhat(flat_data[i].out.jhat);
    Vector const &exp_khat(flat_data[i].out.khat);

    Attitude attitude = get_attitude_from_accel_mag(inAccel, inMag);
    EXPECT_EQ(exp_euler, attitude.euler);
    EXPECT_EQ(exp_ihat, attitude.ihat);
    EXPECT_EQ(exp_jhat, attitude.jhat);
    EXPECT_EQ(exp_khat, attitude.khat);
  }
}
