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

Vector const localToGlobal(float yaw, float pitch, float roll, Vector const &v)
{
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

// // Angles are expressed in radians.
// AttitudeTestData const generate_test_data(float yaw, float pitch, float roll) {
//   // The unit vector representing gravity (down).
//   Vector accel = globalToLocal(yaw, pitch, roll, Vector(0, 0, -1));

//   // The unit vector representing magnetic north.
//   float const inclination = -60;
//   Vector mag = globalToLocal(yaw, pitch, roll,
//   	Vector(0, cos(inclination*M_PI/180), sin(inclination*M_PI/180)));

//   // std::cout << "stick=" << stick << std::endl;
//   // std::cout << "accel=" << accel << std::endl;
//   // std::cout << "mag=" << mag << std::endl;

// 	AttitudeTestData data = {
//     "Noname",
//     accel, mag, // accel, magnetometer
//     {
//       localToGlobal(yaw, pitch, roll, Vector(0, 1, 0)), // Euler
//       globalToLocal(yaw, pitch, roll, Vector(1, 0, 0)), // ihat
//       globalToLocal(yaw, pitch, roll, Vector(0, 1, 0)), // jhat
//       globalToLocal(yaw, pitch, roll, Vector(0, 0, 1)), // khat
//       Quaternion(yaw, pitch, roll),
//       yaw, pitch, roll,
//     }
//   };

// 	return data;
// }

// Angles are expressed in radians.
AttitudeTestData const generate_test_data(float yaw, float pitch, float roll) {
  // The unit vector representing gravity (down).
  Vector accel = globalToLocal(yaw, pitch, roll, Vector(0, 0, -1));

  // The unit vector representing magnetic north.
  float const inclination = -60;
  Vector mag = globalToLocal(yaw, pitch, roll,
  	Vector(0, cos(inclination*M_PI/180), sin(inclination*M_PI/180)));

  // std::cout << "stick=" << stick << std::endl;
  // std::cout << "accel=" << accel << std::endl;
  // std::cout << "mag=" << mag << std::endl;

  Vector aaVector = Vector(cos(pitch) * sin(yaw), cos(pitch) * cos(yaw), sin(pitch));
  std::cout << "Yo! aaVector is " << aaVector << std::endl;
  float aaAngle = roll;



	AttitudeTestData data = {
    "Noname",
    accel, mag, // accel, magnetometer
    {
      localToGlobal(yaw, pitch, roll, Vector(0, 1, 0)), // Euler

      globalToLocal(yaw, pitch, roll, Vector(1, 0, 0)), // ihat
      globalToLocal(yaw, pitch, roll, Vector(0, 1, 0)), // jhat
      globalToLocal(yaw, pitch, roll, Vector(0, 0, 1)), // khat

      Quaternion(aaAngle, aaVector),

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
	Values(30),
	// Range(-180, 181, 30),

  // Pitch
	Values(0),
	// Range(-90, 91, 30),

  // Roll
	Values(0)
	// Range(-180, 181, 30)
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

  // EXPECT_EQ(exp_q, actual.q);
  std::cout << "Q_expected=" << exp_q << std::endl;
  std::cout << "Q_actual  =" << actual.q << std::endl;
  EXPECT_TRUE(exp_q.equivalent(actual.q));

  EXPECT_EQ(exp_aaVector, actual.aaVector);
  EXPECT_EQ(exp_aaAngle, actual.aaAngle);

  EXPECT_EQ(exp_ihat, actual.ihat);
  EXPECT_EQ(exp_jhat, actual.jhat);
  EXPECT_EQ(exp_khat, actual.khat);

  // EXPECT_NEAR(data.out.heading, actual.heading, 0.2);
  // EXPECT_NEAR(data.out.pitch, actual.pitch, 0.2);
  // EXPECT_NEAR(data.out.roll, actual.roll, 0.2);
}
