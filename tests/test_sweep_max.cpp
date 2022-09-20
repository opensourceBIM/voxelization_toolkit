#include "../storage.h"
#include "../sweep.h"
#include <gtest/gtest.h>

TEST(Sweep, SweepWithMax) {
	const double d = 1;
	auto a = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 20, 20, 20, 10);
	a->Set(make_vec<size_t>(1U, 1U, 1U));
	auto b = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 20, 20, 20, 10);
	b->Set(make_vec<size_t>(1U, 1U, 10U));

	{
		sweep s;
		s.until = b;
		ASSERT_EQ(s(a, 0, 0, 1)->count(), 10 - 1);
	}
	{
		sweep s;
		s.until = b;
		s.max_depth = 3;
		ASSERT_EQ(s(a, 0, 0, 1)->count(), 1 + 3);
	}
}