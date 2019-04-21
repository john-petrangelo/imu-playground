#include <gtest/gtest.h>

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

AttitudeTestData const flat_data[] = {
  {
    "Flat, 0˚ (N)",
    Vector(0, 0, -1), Vector(0, 1, -0.5), // accel, magnetometer
    {
      Vector(0, 1, 0), // Euler
      Vector(1, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1), // ihat, jhat, khat
      0, 0, 0, // heading, pitch, roll
    }
  },
  {
    "Flat, 30˚ (NNE)",
    Vector(0, 0, -1), Vector(-0.5, 0.87, -0.5),
    {
      Vector(0.5, 0.87, 0),
      Vector(0.87, 0.5, 0), Vector(-0.5, 0.87, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 60˚ (ENE)",
    Vector(0, 0, -1), Vector(-0.87, 0.5, -0.5),
    {
      Vector(0.87, 0.5, 0),
      Vector(0.5, 0.87, 0), Vector(-0.87, 0.5, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 90˚ (E)",
    Vector(0, 0, -1), Vector(-1, 0, -0.5),
    {
      Vector(1, 0, 0),
      Vector(0, 1, 0), Vector(-1, 0, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 120˚ (ESE)",
    Vector(0, 0, -1), Vector(-0.87, -0.5, -0.5),
    {
      Vector(0.87, -0.5, 0),
      Vector(-0.5, 0.87, 0), Vector(-0.87, -0.5, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 150˚ (SSE)",
    Vector(0, 0, -1), Vector(-0.50, -0.87, -0.5),
    {
      Vector(0.5, -0.87, 0),
      Vector(-0.87, 0.5, 0), Vector(-0.5, -0.87, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 180 (S)",
    Vector(0, 0, -1), Vector(0, -1, -0.5),
    {
      Vector(0, -1, 0),
      Vector(-1, 0, 0), Vector(0, -1, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 210 (SSW)",
    Vector(0, 0, -1), Vector(0.5, -0.87, -0.5),
    {
      Vector(-0.5, -0.87, 0),
      Vector(-0.87, -0.5, 0), Vector(0.5, -0.87, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 240 (WSW)",
    Vector(0, 0, -1), Vector(0.87, -0.5, -0.5),
    {
      Vector(-0.87, -0.5, 0),
      Vector(-0.5, -0.87, 0), Vector(0.87, -0.5, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 270 (W)",
    Vector(0, 0, -1), Vector(1, 0, -0.5),
    {
      Vector(-1, 0, 0),
      Vector(0, -1, 0), Vector(1, 0, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 300 (WNW)",
    Vector(0, 0, -1), Vector(0.87, 0.5, -0.5),
    {
      Vector(-0.87, 0.5, 0),
      Vector(0.5, -0.87, 0), Vector(0.87, 0.5, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
  {
    "Flat, 330 (NNW)",
    Vector(0, 0, -1), Vector(0.5, 0.87, -0.5),
    {
      Vector(-0.5, 0.87, 0),
      Vector(0.87, -0.5, 0), Vector(0.5, 0.87, 0), Vector(0, 0, 1),
      0, 0, 0,
    }
  },
};

class AttitudeTest : public :: testing::TestWithParam<AttitudeTestData> { };
INSTANTIATE_TEST_SUITE_P(PitchZeroTests, AttitudeTest, ::testing::ValuesIn(flat_data));

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
}
