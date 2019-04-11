#include <cmath>
#include <gtest/gtest.h>

#include "common.h"

TEST(VectorTest, VectorCrossProduct) {
	vector_t v1{1.1, 2.1, 3.5};
	vector_t v2{7.2, 8.6, 11.1};

	vector_t expected1 {-6.79, 12.99, -5.66};
	vector_t actual1 = v1.crossproduct(v2);

	vector_t expected2 {6.79, -12.99, 5.66};
	vector_t actual2 = v2.crossproduct(v1);

	EXPECT_NEAR(expected1.x, actual1.x, 0.01);
	EXPECT_NEAR(expected1.y, actual1.y, 0.01);
	EXPECT_NEAR(expected1.z, actual1.z, 0.01);

	EXPECT_NEAR(expected2.x, actual2.x, 0.01);
	EXPECT_NEAR(expected2.y, actual2.y, 0.01);
	EXPECT_NEAR(expected2.z, actual2.z, 0.01);
}

TEST(VectorTest, VectorDotProduct) {
	vector_t v1{1.1, 2.1, 3.5};
	vector_t v2{7.2, 8.6, 11.1};

	float expected = 64.83;

	EXPECT_NEAR(expected, v1.dotproduct(v2), 0.01);
	EXPECT_NEAR(expected, v2.dotproduct(v1), 0.01);
}

TEST(VectorTest, VectorMagnitude) {
	vector_t v0{0.0, 0.0, 0.0};
	vector_t v1{1.1, 2.1, 3.5};

	float expected0 = 0.0;
	float expected1 = 4.23;

	EXPECT_NEAR(expected0, v0.magnitude(), 0.01);
	EXPECT_NEAR(expected1, v1.magnitude(), 0.01);
}

TEST(VectorTest, VectorNormalize) {
	vector_t v0{0.0, 0.0, 0.0};
	vector_t v1{1.1, 2.1, 3.5};

	vector_t expected0 {0.0, 0.0, 0.0};
	vector_t actual0 = v0.normalize();

	vector_t expected1 {0.26, 0.50, 0.83};
	vector_t actual1 = v1.normalize();

	EXPECT_NEAR(1.0, actual1.magnitude(), 0.01);

	EXPECT_NEAR(expected0.x, actual0.x, 0.01);
	EXPECT_NEAR(expected0.y, actual0.y, 0.01);
	EXPECT_NEAR(expected0.z, actual0.z, 0.01);

	EXPECT_NEAR(expected1.x, actual1.x, 0.01);
	EXPECT_NEAR(expected1.y, actual1.y, 0.01);
	EXPECT_NEAR(expected1.z, actual1.z, 0.01);
}

TEST(VectorTest, VectorOpposite) {
	vector_t v0{0.0, 0.0, 0.0};
	vector_t v1{1.1, 2.1, 3.5};

	vector_t expected0 {0.0, 0.0, 0.0};
	vector_t actual0 = v0.opposite();

	vector_t expected1 {-1.1, -2.1, -3.5};
	vector_t actual1 = v1.opposite();

	EXPECT_NEAR(expected0.x, actual0.x, 0.01);
	EXPECT_NEAR(expected0.y, actual0.y, 0.01);
	EXPECT_NEAR(expected0.z, actual0.z, 0.01);

	EXPECT_NEAR(expected1.x, actual1.x, 0.01);
	EXPECT_NEAR(expected1.y, actual1.y, 0.01);
	EXPECT_NEAR(expected1.z, actual1.z, 0.01);
}

TEST(VectorTest, VectorScale) {
	vector_t v{1.1, 2.1, 3.5};

	vector_t expected1 {5.83, 11.13, 18.55};
	vector_t actual1 = v.scale(5.3);

	vector_t expected2 {2.2, 4.2, 7.0};
	vector_t actual2 = v.scale(2.0);

	EXPECT_NEAR(expected1.x, actual1.x, 0.01);
	EXPECT_NEAR(expected1.y, actual1.y, 0.01);
	EXPECT_NEAR(expected1.z, actual1.z, 0.01);

	EXPECT_NEAR(expected2.x, actual2.x, 0.01);
	EXPECT_NEAR(expected2.y, actual2.y, 0.01);
	EXPECT_NEAR(expected2.z, actual2.z, 0.01);
}
