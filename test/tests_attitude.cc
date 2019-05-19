#include <gtest/gtest.h>
#include <cmath>

#include "common.h"

using std::cout;
using std::endl;

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

Vector const localToGlobal(float yaw, float pitch, float roll, Vector const &v);

Vector const globalToLocal(float yaw, float pitch, float roll, Vector const &v)
{
	// All angles are inverted.
  float c1 = cos(-roll);
  float c2 = cos(-pitch);
  float c3 = cos(-yaw);
  float s1 = sin(-roll);
  float s2 = sin(-pitch);
  float s3 = sin(-yaw);

  // R_xyz
  float x = v.x * (c2*c3           ) + v.y * (-c2*s3           ) + v.z * ( s2);
  float y = v.x * (c1*s3 + c3*s1*s2) + v.y * ( c1*c3 - s1*s2*s3) + v.z * (-c2*s1);
  float z = v.x * (s1*s3 - c1*c3*s2) + v.y * ( c3*s1 + c1*s2*s3) + v.z * ( c1*c2);

	return Vector(x, y, z);
}

Vector const localToGlobal(float yaw, float pitch, float roll, Vector const &v)
{
  float c1 = cos(yaw);
  float c2 = cos(pitch);
  float c3 = cos(roll);
  float s1 = sin(yaw);
  float s2 = sin(pitch);
  float s3 = sin(roll);

  // R_zyx
  float x = v.x * ( c1*c2) + v.y * ( c1*s2*s3 - c3*s1) + v.z * ( s1*s3+c1*c3*s2);
  float y = v.x * ( c2*s1) + v.y * ( c1*c3 + s1*s2*s3) + v.z * ( c3*s1*s2 - c1*s3);
  float z = v.x * (-s2   ) + v.y * ( c2*s3           ) + v.z * ( c2*c3);

	return Vector(x, y, z);
}

// Angles are expressed in radians.
AttitudeTestData const generate_test_data(float yaw, float pitch, float roll) {
  // The unit vector representing gravity (down).
  Vector accel = globalToLocal(yaw, pitch, roll, Vector(0, 0, 1));

  // The unit vector representing magnetic north.
  float const inclination = -60;
  Vector mag = globalToLocal(yaw, pitch, roll,
  	Vector(cos(inclination*M_PI/180), 0, -sin(inclination*M_PI/180)));

	AttitudeTestData data = {
    "Noname",
    accel, mag, // accel, magnetometer
    {
      localToGlobal(yaw, pitch, roll, Vector(1, 0, 0)), // Euler

      globalToLocal(yaw, pitch, roll, Vector(1, 0, 0)), // ihat
      globalToLocal(yaw, pitch, roll, Vector(0, 1, 0)), // jhat
      globalToLocal(yaw, pitch, roll, Vector(0, 0, 1)), // khat

      Quaternion(yaw, pitch, roll),

      // Axis-angle
      Vector(cos(pitch) * sin(yaw), cos(pitch) * cos(yaw), sin(pitch)),
      roll,

      yaw, pitch, roll,
    }
  };

	return data;
}

using ::testing::Combine;
using ::testing::Range;
using ::testing::Values;

class AttitudeTestCombos : public :: testing::TestWithParam<std::tuple<int, int, int>> { };
INSTANTIATE_TEST_SUITE_P(AllCombos, AttitudeTestCombos, Combine(
	// Yaw
	// Values(-100),
	Range(-180, 181, 30),

  // Pitch
	// Values(-80),
	Range(-90, 91, 30),

  // Roll
	// Values(-180)
	Range(-180, 181, 30)
));

TEST_P(AttitudeTestCombos, testCombos)
{
	int yaw;
	int pitch;
	int roll;
  std::tie(yaw, pitch, roll) = GetParam();

	AttitudeTestData data = generate_test_data(
		yaw * M_PI / 180,
		pitch * M_PI / 180,
		roll * M_PI / 180);

  Vector const &inAccel =  data.inAccel;
  Vector const &inMag =  data.inMag;

  Vector const &exp_euler(data.out.euler);

  Quaternion const &exp_q(data.out.q);

  Vector const &exp_aaVector(data.out.aaVector);
  float const &exp_aaAngle(data.out.aaAngle);

  Vector const &exp_ihat(data.out.ihat);
  Vector const &exp_jhat(data.out.jhat);
  Vector const &exp_khat(data.out.khat);

  Attitude const actual = get_attitude_from_accel_mag(inAccel, inMag);
  EXPECT_EQ(exp_euler, actual.euler);

  EXPECT_EQ(exp_ihat, actual.ihat);
  EXPECT_EQ(exp_jhat, actual.jhat);
  EXPECT_EQ(exp_khat, actual.khat);

  EXPECT_TRUE(exp_q.equivalent(actual.q));

  float actualHeading   = 180/M_PI * actual.heading;
  float actualPitch     = 180/M_PI * actual.pitch;
  float actualRoll      = 180/M_PI * actual.roll;
  float expectedHeading = 180/M_PI * data.out.heading;
  float expectedPitch   = 180/M_PI * data.out.pitch;
  float expectedRoll    = 180/M_PI * data.out.roll;

  // TODO Refactor the code below.
  if (fabs(expectedPitch + 90) < 0.2) {
  	// Gimble lock, straight down
    EXPECT_NEAR(actualRoll, 0, 0.2);
    float headingErr = fabs(actualHeading - (expectedHeading + expectedRoll));
    if (headingErr < 1) {
  		EXPECT_NEAR(headingErr, 0, 0.2);
  	} else {
  		EXPECT_NEAR(headingErr, 360, 0.2);
  	}
  } else if (fabs(expectedPitch - 90) < 0.2) {
  	// Gimble lock, straight up
    EXPECT_NEAR(actualRoll, 0, 0.2);
    float headingErr = fabs(actualHeading - (expectedHeading - expectedRoll));
    if (headingErr < 1) {
  		EXPECT_NEAR(headingErr, 0, 0.2);
  	} else {
  		EXPECT_NEAR(headingErr, 360, 0.2);
  	}
  } else {
  	// No gimble lock
    float headingErr = fabs(expectedHeading - actualHeading);
    if (headingErr < 1) {
  		EXPECT_NEAR(headingErr, 0, 0.2);
  	} else {
  		EXPECT_NEAR(headingErr, 360, 0.2);
  	}

    float rollErr = fabs(expectedRoll - actualRoll);
    if (rollErr < 1) {
  		EXPECT_NEAR(rollErr, 0, 0.2);
  	} else {
  		EXPECT_NEAR(rollErr, 360, 0.2);
  	}
 }

	 EXPECT_NEAR(data.out.pitch, actual.pitch, 0.2);
}
