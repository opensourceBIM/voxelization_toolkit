#include "../dim3.h"

#include <gtest/gtest.h>

TEST(VecN, Arithmetic) {
	vec_n<3, int> vec123(1, 2, 3);
	vec_n<3, int> vec124(1, 2, 4);
	vec_n<3, int> vec234(2, 3, 4);
	vec_n<3, int> vec001(0, 0, 1);
	vec_n<3, int> vec003(0, 0, 3);

	EXPECT_TRUE((vec123 == vec123).all());
	EXPECT_TRUE((vec123 == vec123).any());
	EXPECT_FALSE((vec123 == vec124).all());

	EXPECT_TRUE(((vec123 + vec001) == vec124).all());
	EXPECT_TRUE(((vec123 + 1) == vec234).all());

	int a, b, c;
	a = b = c = 0;

	vec123.tie(a, b, c);

	EXPECT_EQ(a, 1);

	EXPECT_TRUE((vec123.maximum(vec003 + 1) == vec124).all());

	EXPECT_EQ(vec123.sum(), 6);
}