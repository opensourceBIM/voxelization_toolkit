#include "../voxec.h"

#include <gtest/gtest.h>

TEST(Keep, Neighbours) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 8, 8, 8, 8);

	for (int i = 1; i <= 3; ++i) {
		for (int j = 1; j <= 3; ++j) {
			for (int k = 1; k <= 3; ++k) {
				storage->Set(make_vec<size_t>(i, j, k));
			}
		}
	}

	{
		op_keep_neighbours keep;
		scope_map arguments;
		{
			symbol_value v = storage;
			arguments["input"] = v;
		}
		{
			symbol_value v = 6;
			arguments["num_neighbours"] = v;
		}
		{
			symbol_value v = 6;
			arguments["connectivity"] = v;
		}
		
		auto result = (regular_voxel_storage*)boost::get<abstract_voxel_storage*>(keep.invoke(arguments));
		ASSERT_EQ(result->count(), 1);
	}

	{
		op_keep_neighbours keep;
		scope_map arguments;
		{
			symbol_value v = storage;
			arguments["input"] = v;
		}
		{
			symbol_value v = 26;
			arguments["num_neighbours"] = v;
		}
		{
			symbol_value v = 26;
			arguments["connectivity"] = v;
		}

		auto result = (regular_voxel_storage*)boost::get<abstract_voxel_storage*>(keep.invoke(arguments));
		ASSERT_EQ(result->count(), 1);
	}

	{
		op_keep_neighbours keep;
		scope_map arguments;
		{
			symbol_value v = storage;
			arguments["input"] = v;
		}
		{
			symbol_value v = 5;
			arguments["num_neighbours"] = v;
		}
		{
			symbol_value v = 6;
			arguments["connectivity"] = v;
		}

		auto result = (regular_voxel_storage*)boost::get<abstract_voxel_storage*>(keep.invoke(arguments));
		ASSERT_EQ(result->count(), 7);
	}
}
