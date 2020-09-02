#include "../collapse.h"
#include "../sweep.h"

#include <gtest/gtest.h>

TEST(SweepAndCollapse, Same) {
	// Collapse and Sweep should be each other's reverse
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 100, 100, 100, 32);
	storage->Set(make_vec<size_t>( 1U,1U,1U ));
	sweep s;
	collapse c;
	auto swept = s(storage, 0, 0, 5);
	auto result = c(swept, 0, 0, -1);
	ASSERT_EQ(result->count(), 1);
	ASSERT_TRUE(result->Get(make_vec<size_t>( 1U,1U,1U )));
}