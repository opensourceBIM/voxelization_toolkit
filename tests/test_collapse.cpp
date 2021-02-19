#include "../collapse.h"
#include "../sweep.h"

#include <gtest/gtest.h>

TEST(SweepAndCollapse, Same) {
	// Collapse and Sweep should be each other's reverse
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 100, 100, 100, 32);
	auto loc = make_vec<size_t>(1U, 1U, 1U);
	storage->Set(loc);
	
	sweep s;
	collapse c;
	collapse_count cc;
	
	auto swept = s(storage, 0, 0, 5);
	ASSERT_EQ(swept->count(), 5);
	auto result = c(swept, 0, 0, -1);
	ASSERT_EQ(result->count(), 1);
	ASSERT_TRUE(result->Get(loc));

	auto result2 = cc(swept, 0, 0, -1);
	ASSERT_EQ(result2->value_bits(), 32);
	size_t value = 0;
	result2->Get(loc, &value);
	ASSERT_EQ(value, 5);
	// @nb dz=1 because the extrusion depth is now stored in the voxel value
	auto swept2 = s(result2, 0, 0, 1);
	ASSERT_EQ(swept2->count(), 5);
}