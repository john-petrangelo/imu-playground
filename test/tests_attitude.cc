#include <gtest/gtest.h>
#include <cmath>

#include "common.h"

struct AttitudeTestData {
  std::string name;
  Vector inAccel;
  Vector inMag;
  Attitude out;
};

std::ostream& operator<<(std::ostream& os, AttitudeTestData const &data)
{
  return os << data.name;
}

// Sweep heading 0-330 in 30˚ steps, no pitch, no roll.
AttitudeTestData const sweep_heading_data[] = {
  {
    "Flat, 0˚ (N)",
    Vector(0, 0, -1), Vector(0, 0.5, -0.87), // accel, magnetometer
    {
      Vector(0, 1, 0), // Euler
      Vector(1, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1), // ihat, jhat, khat
      0, 0, 0, // heading, pitch, roll
    }
  },
  {
    "Flat, 30˚ (NNE)",
    Vector(0, 0, -1), Vector(-0.25, 0.43, -0.87),
    {
      Vector(0.5, 0.87, 0),
      Vector(0.87, 0.5, 0), Vector(-0.5, 0.87, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 60˚ (ENE)",
    Vector(0, 0, -1), Vector(-0.43, 0.25, -0.87),
    {
      Vector(0.87, 0.5, 0),
      Vector(0.5, 0.87, 0), Vector(-0.87, 0.5, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 90˚ (E)",
    Vector(0, 0, -1), Vector(-0.5, 0, -0.87),
    {
      Vector(1, 0, 0),
      Vector(0, 1, 0), Vector(-1, 0, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 120˚ (ESE)",
    Vector(0, 0, -1), Vector(-0.43, -0.25, -0.87),
    {
      Vector(0.87, -0.5, 0),
      Vector(-0.5, 0.87, 0), Vector(-0.87, -0.5, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 150˚ (SSE)",
    Vector(0, 0, -1), Vector(-0.25, -0.43, -0.87),
    {
      Vector(0.5, -0.87, 0),
      Vector(-0.87, 0.5, 0), Vector(-0.5, -0.87, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 180 (S)",
    Vector(0, 0, -1), Vector(0, -0.5, -0.87),
    {
      Vector(0, -1, 0),
      Vector(-1, 0, 0), Vector(0, -1, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 210 (SSW)",
    Vector(0, 0, -1), Vector(0.25, -0.43, -0.87),
    {
      Vector(-0.5, -0.87, 0),
      Vector(-0.87, -0.5, 0), Vector(0.5, -0.87, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 240 (WSW)",
    Vector(0, 0, -1), Vector(0.43, -0.25, -0.87),
    {
      Vector(-0.87, -0.5, 0),
      Vector(-0.5, -0.87, 0), Vector(0.87, -0.5, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 270 (W)",
    Vector(0, 0, -1), Vector(0.5, 0, -0.87),
    {
      Vector(-1, 0, 0),
      Vector(0, -1, 0), Vector(1, 0, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 300 (WNW)",
    Vector(0, 0, -1), Vector(0.43, 0.25, -0.87),
    {
      Vector(-0.87, 0.5, 0),
      Vector(0.5, -0.87, 0), Vector(0.87, 0.5, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 330 (NNW)",
    Vector(0, 0, -1), Vector(0.25, 0.43, -0.87),
    {
      Vector(-0.5, 0.87, 0),
      Vector(0.87, -0.5, 0), Vector(0.5, 0.87, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
};

// Heading 30˚, sweep pitch 0-90˚ in 30˚ steps, no roll.
AttitudeTestData const sweep_pitch_data[] = {
  {
    "SweepPitch, -90˚",
    Vector(0, 1, 0), Vector(-0.43, 0.75, 0.50), // accel, magnetometer
    {
      Vector(0, 0, -1), // Euler
      Vector(0.87, 0, 0.5), Vector(-0.5, 0, 0.87), Vector(0, -1, 0), // ihat, jhat, khat
      0, 0, 0, // heading, pitch, roll
    }
  },
  {
    "SweepPitch, -60˚",
    Vector(0, 0.87, -0.5), Vector(-0.5, 0.87, 0),
    {
      Vector(0.5, 0.87, 0),
      Vector(0.87, 0.5, 0), Vector(-0.5, 0.87, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "SweepPitch, -30˚",
    Vector(0, 0, -1), Vector(-0.43, 0.75, -0.5),
    {
      Vector(0.87, 0.5, 0),
      Vector(0.5, 0.87, 0), Vector(-0.87, 0.5, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "SweepPitch,   0˚",
    Vector(0, 0, -1), Vector(-0.25, 0.43, -0.87), // accel, magnetometer
    {
      Vector(0.5, 0.87, 0), // Euler
      Vector(0.87, 0.5, 0), Vector(-0.5, 0.87, 0), Vector(0, 0, 1), // ihat, jhat, khat
      0, 0, 0, // heading, pitch, roll
    }
  },
  {
    "SweepPitch,  30˚",
    Vector(0, 0, -1), Vector(0, 0, -1),
    {
      Vector(0.5, 0.87, 0),
      Vector(0.87, 0.5, 0), Vector(-0.5, 0.87, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "SweepPitch,  60˚",
    Vector(0, -0.87, -0.5), Vector(-0.5, -0.43, -0.75),
    {
      Vector(-0.25, 0.43, 0.87),
      Vector(0.87, -0.25, -0.43), Vector(0.5, 0.43, -0.75), Vector(0, 0.87, 0.5),
      30, 60, 0,
    }
  },
  {
    "SweepPitch,  60, python˚",
    Vector(0, -0.87, -0.5), Vector(-0.75, 0.65, -0.13),
    {
      Vector(-0.25, 0.43, 0.87),
      Vector(0.87, -0.25, -0.43), Vector(0.5, 0.43, -0.75), Vector(0, 0.87, 0.5),
      30, 60, 0,
    }
  },
  // {
  //   "SweepPitch,  90˚",
  //   Vector(0, -1, 0), Vector(0.43, -0.75, -0.5),
  //   {
  //     Vector(0, 0, 1),
  //     Vector(0.87, 0, -0.5), Vector(0.5, 0, -0.87), Vector(0, 1, 0),
  //     30, 90, 0,
  //   }
  // },
};

class AttitudeTest : public :: testing::TestWithParam<AttitudeTestData> { };
// INSTANTIATE_TEST_SUITE_P(SweepHeadingTests, AttitudeTest, ::testing::ValuesIn(sweep_heading_data));
// INSTANTIATE_TEST_SUITE_P(SweepPitchTests, AttitudeTest, ::testing::ValuesIn(sweep_pitch_data));

TEST_P(AttitudeTest, getAttitudeFromAM)
{
  AttitudeTestData const &data = GetParam();

  Vector const &inAccel =  data.inAccel;
  Vector const &inMag =  data.inMag;

  Vector const &exp_euler(data.out.euler);
  Vector const &exp_ihat(data.out.ihat);
  Vector const &exp_jhat(data.out.jhat);
  Vector const &exp_khat(data.out.khat);

  Attitude const actual = get_attitude_from_accel_mag(inAccel, inMag);
  EXPECT_EQ(exp_euler, actual.euler);
  EXPECT_EQ(exp_ihat, actual.ihat);
  EXPECT_EQ(exp_jhat, actual.jhat);
  EXPECT_EQ(exp_khat, actual.khat);

  EXPECT_NEAR(data.out.heading, rad2deg(actual.heading), 0.2);
  EXPECT_NEAR(data.out.pitch, rad2deg(actual.pitch), 0.2);
  EXPECT_NEAR(data.out.roll, rad2deg(actual.roll), 0.2);
}

Vector const globalToLocal(float yaw, float pitch, float roll, Vector const &v)
{
	// All angles are inverted.
	float a = -pitch;
	float b = -roll;
  // Heading is the opposite direction of right-hand rule.
	float c = yaw;

  float c1 = cos(a);
  float c2 = cos(b);
  float c3 = cos(c);
  float s1 = sin(a);
  float s2 = sin(b);
  float s3 = sin(c);

  // R_yxz
  float x = v.x * ( c2*c3 + s1*s2*s3) + v.y * (-c2*s3 + s1*s2*c3) + v.z * (c1*s2);
  float y = v.x * ( c1*s3           ) + v.y * ( c1*c3)            + v.z * (-s1);
  float z = v.x * (-s2*c3 + s1*c2*s3) + v.y * ( s2*s3 + s1*c2*c3) + v.z * (c1*c2);

	return Vector(x, y, z);
}

Vector const localToGlobal(float yaw, float pitch, float roll, Vector const &v) {
	float a = pitch;
	float b = roll;
  // Heading is the opposite direction of right-hand rule.
	float c = -yaw;

  float c1 = cos(a);
  float c2 = cos(b);
  float c3 = cos(c);
  float s1 = sin(a);
  float s2 = sin(b);
  float s3 = sin(c);

  // R_zxy
  float x = v.x * ( c2*c3 - s1*s2*s3) + v.y * (-c1*s3) + v.z * (s2*c3 + s1*c2*s3);
  float y = v.x * ( c2*s3 + s1*s2*c3) + v.y * ( c1*c3) + v.z * (s2*s3 - s1*c2*c3);
  float z = v.x * (-c1*s2           ) + v.y * ( s1   ) + v.z * (c1*c2           );

	return Vector(x, y, z);
}

AttitudeTestData const generate_test_data(float yaw, float pitch, float roll) {
  // The unit vector representing the direction the stick is pointing.
  Vector stick = localToGlobal(yaw, pitch, roll, Vector(0, 1, 0));
  
  // The unit vector representing gravity (down).
  Vector accel = globalToLocal(yaw, pitch, roll, Vector(0, 0, -1));

  // The unit vector representing compass north.
  Vector mag = globalToLocal(yaw, pitch, roll, Vector(0, 1, 0));

  std::cout << "stick=" << stick << std::endl;
  std::cout << "accel=" << accel << std::endl;
  std::cout << "mag=" << mag << std::endl;

	AttitudeTestData data = {
    "Noname˚",
    accel, mag, // accel, magnetometer
    {
      Vector(1, 0, 0), // Euler
      Vector(1, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1), // ihat, jhat, khat
      yaw, pitch, roll,
    }
  };

	return data;
}

TEST(AttitudeTest, testABC)
{
	float const yaw   = 30 * M_PI / 180;
	float const pitch = 60 * M_PI / 180;
	float const roll  =  0 * M_PI / 180;

	AttitudeTestData data = generate_test_data(yaw, pitch, roll);

  Vector const &inAccel =  data.inAccel;
  Vector const &inMag =  data.inMag;

  // Vector const &exp_euler(data.out.euler);
  // Vector const &exp_ihat(data.out.ihat);
  // Vector const &exp_jhat(data.out.jhat);
  // Vector const &exp_khat(data.out.khat);

  Attitude const actual = get_attitude_from_accel_mag(inAccel, inMag);
  //  EXPECT_EQ(exp_euler, actual.euler);
  // EXPECT_EQ(exp_ihat, actual.ihat);
  // EXPECT_EQ(exp_jhat, actual.jhat);
  // EXPECT_EQ(exp_khat, actual.khat);

  EXPECT_NEAR(data.out.heading, actual.heading, 0.2);
  EXPECT_NEAR(data.out.pitch, actual.pitch, 0.2);
  EXPECT_NEAR(data.out.roll, actual.roll, 0.2);
}

