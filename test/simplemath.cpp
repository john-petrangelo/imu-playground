#include <cmath>
#include <gtest/gtest.h>

double cubic(double d)
{
	return pow(d,3);
}

TEST(testMyStuff, integerTests) {
	EXPECT_EQ(1000, cubic(10));
	EXPECT_EQ(8, cubic(2));
	EXPECT_EQ(0, cubic(0));
}

// int main()
// {
// 	cubic(10);
// 	return 0;
// }
