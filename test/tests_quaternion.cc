#include <cmath>
#include <gtest/gtest.h>

#include "common.h"

float const MAX_DELTA = 0.1;

TEST(QuaternionTest, QuaternionMakeFromVector) {
	Vector v(1.1, 2.1, 3.5);

	Quaternion actual(v);

	EXPECT_NEAR(0.0, actual.w, MAX_DELTA);
	EXPECT_NEAR(v.x, actual.x, MAX_DELTA);
	EXPECT_NEAR(v.y, actual.y, MAX_DELTA);
	EXPECT_NEAR(v.z, actual.z, MAX_DELTA);
}

TEST(QuaternionTest, QuaternionMakeFromAngleAndVector) {
	Vector v{1.1, 2.1, 3.5};

	Quaternion expected(0.8660254038, 0.55, 1.05, 1.75);
	Quaternion actual(30 * M_PI / 180, v);

	EXPECT_NEAR(expected.w, actual.w, MAX_DELTA);
	EXPECT_NEAR(expected.x, actual.x, MAX_DELTA);
	EXPECT_NEAR(expected.y, actual.y, MAX_DELTA);
	EXPECT_NEAR(expected.z, actual.z, MAX_DELTA);
}

TEST(QuaternionTest, QuaternionConjuagte) {
	Quaternion q(0.5, 1.1, 2.1, 3.5);

	Quaternion expected(0.5, -1.1, -2.1, -3.5);
	Quaternion actual = q.conjugate();

	EXPECT_NEAR(expected.w, actual.w, MAX_DELTA);
	EXPECT_NEAR(expected.x, actual.x, MAX_DELTA);
	EXPECT_NEAR(expected.y, actual.y, MAX_DELTA);
	EXPECT_NEAR(expected.z, actual.z, MAX_DELTA);
}

TEST(QuaternionTest, QuaternionMultiply) {
	Quaternion q1(0.5, 1.1, 2.1, 3.5);
	Quaternion q2(0.6, 1.2, 2.2, 3.6);

	Quaternion expected(-18.24, 1.12, 2.60, 3.80);
	Quaternion actual = q1.multiply(q2);

	EXPECT_NEAR(expected.w, actual.w, MAX_DELTA);
	EXPECT_NEAR(expected.x, actual.x, MAX_DELTA);
	EXPECT_NEAR(expected.y, actual.y, MAX_DELTA);
	EXPECT_NEAR(expected.z, actual.z, MAX_DELTA);
}

TEST(QuaternionTest, QuaternionMagnitude) {
	Quaternion q{0.5, 1.1, 2.1, 3.5};

	float expected = 4.26;
	float actual = q.magnitude();

	EXPECT_NEAR(expected, actual, MAX_DELTA);
}

TEST(QuaternionTest, QuaternionNormalize) {
	Quaternion q(0.5, 1.1, 2.1, 3.5);

	Quaternion expected (0.12, 0.26, 0.49, 0.82);
	Quaternion actual = q.normalize();

	EXPECT_NEAR(expected.w, actual.w, MAX_DELTA);
	EXPECT_NEAR(expected.x, actual.x, MAX_DELTA);
	EXPECT_NEAR(expected.y, actual.y, MAX_DELTA);
	EXPECT_NEAR(expected.z, actual.z, MAX_DELTA);
}

TEST(QuaternionTest, QuaternionVector) {
	Quaternion q(0.5, 1.1, 2.1, 3.5);

	Vector actual = q.vector();

	EXPECT_NEAR(q.x, actual.x, MAX_DELTA);
	EXPECT_NEAR(q.y, actual.y, MAX_DELTA);
	EXPECT_NEAR(q.z, actual.z, MAX_DELTA);
}
