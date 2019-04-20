#include <cmath>
#include <gtest/gtest.h>

#include "common.h"

float const TOLERANCE = 0.01;

TEST(VectorTest, VectorCrossProduct) {
	Vector v1{1.1, 2.1, 3.5};
	Vector v2{7.2, 8.6, 11.1};

	Vector expected1 {-6.79, 12.99, -5.66};
	Vector actual1 = v1.crossproduct(v2);

	Vector expected2 {6.79, -12.99, 5.66};
	Vector actual2 = v2.crossproduct(v1);

	EXPECT_EQ(expected1, actual1);
	EXPECT_EQ(expected2, actual2);
}

TEST(VectorTest, VectorDotProduct) {
	Vector v1{1.1, 2.1, 3.5};
	Vector v2{7.2, 8.6, 11.1};

	float expected = 64.83;

	EXPECT_NEAR(expected, v1.dotproduct(v2), TOLERANCE);
	EXPECT_NEAR(expected, v2.dotproduct(v1), TOLERANCE);
}

TEST(VectorTest, VectorMagnitude) {
	Vector v0{0.0, 0.0, 0.0};
	Vector v1{1.1, 2.1, 3.5};

	float expected0 = 0.0;
	float expected1 = 4.23;

	EXPECT_NEAR(expected0, v0.magnitude(), TOLERANCE);
	EXPECT_NEAR(expected1, v1.magnitude(), TOLERANCE);
}

TEST(VectorTest, VectorNormalize) {
	Vector v0{0.0, 0.0, 0.0};
	Vector v1{1.1, 2.1, 3.5};

	Vector expected0 {0.0, 0.0, 0.0};
	Vector actual0 = v0.normalize();

	Vector expected1 {0.26, 0.50, 0.83};
	Vector actual1 = v1.normalize();

	EXPECT_NEAR(1.0, actual1.magnitude(), TOLERANCE);

	EXPECT_EQ(expected0, actual0);
	EXPECT_EQ(expected1, actual1);
}

TEST(VectorTest, VectorOpposite) {
	Vector v0{0.0, 0.0, 0.0};
	Vector v1{1.1, 2.1, 3.5};

	Vector expected0 {0.0, 0.0, 0.0};
	Vector actual0 = v0.opposite();

	Vector expected1 {-1.1, -2.1, -3.5};
	Vector actual1 = v1.opposite();

	EXPECT_EQ(expected0, actual0);
	EXPECT_EQ(expected1, actual1);
}

TEST(VectorTest, VectorScale) {
	Vector v{1.1, 2.1, 3.5};

	Vector expected1 {5.83, 11.13, 18.55};
	Vector actual1 = v.scale(5.3);

	Vector expected2 {2.2, 4.2, 7.0};
	Vector actual2 = v.scale(2.0);

	EXPECT_EQ(expected1, actual1);
	EXPECT_EQ(expected2, actual2);
}
