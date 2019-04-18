#include <cmath>
#include <gtest/gtest.h>

#include "common.h"

float const MAX_DELTA = 0.02;

TEST(UtilsTest, fmap) {
	// Verify direct 1:1 scaling.
	EXPECT_NEAR(0.0, fmap(0.0, 0.0, 1.0, 0.0, 1.0), MAX_DELTA);
	EXPECT_NEAR(0.3, fmap(0.3, 0.0, 1.0, 0.0, 1.0), MAX_DELTA);
	EXPECT_NEAR(1.0, fmap(1.0, 0.0, 1.0, 0.0, 1.0), MAX_DELTA);

	// Verify scaling -90..90 to -1..1.
	EXPECT_NEAR(-1.0, fmap(-90.0, -90.0, 90.0, -1.0, 1.0), MAX_DELTA);
	EXPECT_NEAR( 0.0, fmap(  0.0, -90.0, 90.0, -1.0, 1.0), MAX_DELTA);
	EXPECT_NEAR( 1.0, fmap( 90.0, -90.0, 90.0, -1.0, 1.0), MAX_DELTA);	

	// Verify inverted output.
	EXPECT_NEAR(1.0, fmap(0.0, 0.0, 1.0, 1.0, 0.0), MAX_DELTA);
	EXPECT_NEAR(0.7, fmap(0.3, 0.0, 1.0, 1.0, 0.0), MAX_DELTA);
	EXPECT_NEAR(0.0, fmap(1.0, 0.0, 1.0, 1.0, 0.0), MAX_DELTA);
}

TEST(UtilsTest, rad2deg) {
	EXPECT_NEAR(  0.0, rad2deg(     0.0), MAX_DELTA);
	EXPECT_NEAR( 45.0, rad2deg(M_PI / 4), MAX_DELTA);
	EXPECT_NEAR( 90.0, rad2deg(M_PI / 2), MAX_DELTA);
	EXPECT_NEAR(180.0, rad2deg(    M_PI), MAX_DELTA);
	EXPECT_NEAR(360.0, rad2deg(2 * M_PI), MAX_DELTA);
}

TEST(UtilsTest, deg2rad) {
	EXPECT_NEAR(     0.0, deg2rad(  0.0), MAX_DELTA);
	EXPECT_NEAR(M_PI / 4, deg2rad( 45.0), MAX_DELTA);
	EXPECT_NEAR(M_PI / 2, deg2rad( 90.0), MAX_DELTA);
	EXPECT_NEAR(    M_PI, deg2rad(180.0), MAX_DELTA);
	EXPECT_NEAR(2 * M_PI, deg2rad(360.0), MAX_DELTA);
}

TEST(UtilsTest, normalizeDeg) {
	// Verify negative boundary.
	EXPECT_NEAR( 179.0, normalizeDeg(-181.0), MAX_DELTA);
	EXPECT_NEAR(-180.0, normalizeDeg(-180.0), MAX_DELTA);
	EXPECT_NEAR(-179.0, normalizeDeg(-179.0), MAX_DELTA);

	// Verify positive boundary.
	EXPECT_NEAR( 179.0, normalizeDeg(179.0), MAX_DELTA);
	EXPECT_NEAR( 180.0, normalizeDeg(180.0), MAX_DELTA);
	EXPECT_NEAR(-179.0, normalizeDeg(181.0), MAX_DELTA);
}

TEST(UtilsTest, sqr) {
	EXPECT_NEAR( 0.0,  sqr(0.0), MAX_DELTA);
	EXPECT_NEAR( 0.25, sqr(0.5), MAX_DELTA);
	EXPECT_NEAR( 1.0,  sqr(1.0), MAX_DELTA);
	EXPECT_NEAR(25.0,  sqr(5.0), MAX_DELTA);
}