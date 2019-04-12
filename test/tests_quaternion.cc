#include <cmath>
#include <gtest/gtest.h>

#include "common.h"

TEST(QuaternionTest, QuaternionMakeFromVector) {
	Vector v{1.1, 2.1, 3.5};

	quaternion_t actual(v);

	EXPECT_NEAR(0.0, actual.w, 0.01);
	EXPECT_NEAR(v.x, actual.x, 0.01);
	EXPECT_NEAR(v.y, actual.y, 0.01);
	EXPECT_NEAR(v.z, actual.z, 0.01);
}

TEST(QuaternionTest, QuaternionMakeFromAngleAndVector) {
	Vector v{1.1, 2.1, 3.5};

	quaternion_t expected {0.8660254038, 0.55, 1.05, 1.75};
	quaternion_t actual(30 * M_PI / 180, v);

	EXPECT_NEAR(expected.w, actual.w, 0.01);
	EXPECT_NEAR(expected.x, actual.x, 0.01);
	EXPECT_NEAR(expected.y, actual.y, 0.01);
	EXPECT_NEAR(expected.z, actual.z, 0.01);
}

TEST(QuaternionTest, QuaternionConjuagte) {
	quaternion_t q{0.5, 1.1, 2.1, 3.5};

	quaternion_t expected {0.5, -1.1, -2.1, -3.5};
	quaternion_t actual = q.conjugate();

	EXPECT_NEAR(expected.w, actual.w, 0.01);
	EXPECT_NEAR(expected.x, actual.x, 0.01);
	EXPECT_NEAR(expected.y, actual.y, 0.01);
	EXPECT_NEAR(expected.z, actual.z, 0.01);
}

TEST(QuaternionTest, QuaternionMultiply) {
	quaternion_t q1{0.5, 1.1, 2.1, 3.5};
	quaternion_t q2{0.6, 1.2, 2.2, 3.6};

	quaternion_t expected {-18.24, 1.12, 2.60, 3.80};
	quaternion_t actual = q1.multiply(q2);

	EXPECT_NEAR(expected.w, actual.w, 0.01);
	EXPECT_NEAR(expected.x, actual.x, 0.01);
	EXPECT_NEAR(expected.y, actual.y, 0.01);
	EXPECT_NEAR(expected.z, actual.z, 0.01);
}

TEST(QuaternionTest, QuaternionMagnitude) {
	quaternion_t q{0.5, 1.1, 2.1, 3.5};

	float expected = 4.26;
	float actual = q.magnitude();

	EXPECT_NEAR(expected, actual, 0.01);
}

TEST(QuaternionTest, QuaternionNormalize) {
	quaternion_t q{0.5, 1.1, 2.1, 3.5};

	quaternion_t expected {0.12, 0.26, 0.49, 0.82};
	quaternion_t actual = q.normalize();

	EXPECT_NEAR(expected.w, actual.w, 0.01);
	EXPECT_NEAR(expected.x, actual.x, 0.01);
	EXPECT_NEAR(expected.y, actual.y, 0.01);
	EXPECT_NEAR(expected.z, actual.z, 0.01);
}

TEST(QuaternionTest, QuaternionVector) {
	quaternion_t q{0.5, 1.1, 2.1, 3.5};

	Vector actual = q.vector();

	EXPECT_NEAR(q.x, actual.x, 0.01);
	EXPECT_NEAR(q.y, actual.y, 0.01);
	EXPECT_NEAR(q.z, actual.z, 0.01);
}
