#include <cmath>
#include <gtest/gtest.h>

#include "common.h"

TEST(QuaternionTest, QuaternionMakeFromVector) {
	Vector v(1.1, 2.1, 3.5);

	Quaternion actual(v);

	EXPECT_EQ(Quaternion(0.0, v.x, v.y, v.z), actual);
}

TEST(QuaternionTest, QuaternionMakeFromAxisAngle) {
	Vector v(1.1, 2.1, 3.5);
	Quaternion expected(0.866025, 0.130107, 0.248386, 0.413977);
	Quaternion actual(30 * M_PI / 180, v);

	EXPECT_EQ(expected, actual);
}

TEST(QuaternionTest, QuaternionConjuagte) {
	Quaternion q(0.5, 1.1, 2.1, 3.5);

	Quaternion expected(0.5, -1.1, -2.1, -3.5);
	Quaternion actual = q.conjugate();

	EXPECT_EQ(expected, actual);
}

TEST(QuaternionTest, QuaternionMultiply) {
	Quaternion q1(0.5, 1.1, 2.1, 3.5);
	Quaternion q2(0.6, 1.2, 2.2, 3.6);

	Quaternion expected(-18.24, 1.12, 2.60, 3.80);
	Quaternion actual = q1.multiply(q2);

	EXPECT_EQ(expected, actual);
}

TEST(QuaternionTest, QuaternionMagnitude) {
	Quaternion q{0.5, 1.1, 2.1, 3.5};

	float expected = 4.26;
	float actual = q.magnitude();

	EXPECT_NEAR(expected, actual, 0.01);
}

TEST(QuaternionTest, QuaternionNormalize) {
	Quaternion q(0.5, 1.1, 2.1, 3.5);

	Quaternion expected (0.12, 0.26, 0.49, 0.82);
	Quaternion actual = q.normalize();

	EXPECT_EQ(expected, actual);
}

TEST(QuaternionTest, QuaternionVector) {
	Quaternion q(0.5, 1.1, 2.1, 3.5);

	Vector actual = q.vector();

	EXPECT_EQ(Vector(q.x, q.y, q.z), actual);
}
