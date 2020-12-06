#include "../offset.h"

#include <gtest/gtest.h>

#include <BRepPrimAPI_MakeBox.hxx>

TEST(Postproc, Offset) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 10, 10, 10, 10);

	for (size_t i = 2; i < 5; ++i) {
		for (size_t j = 2; j < 5; ++j) {
			for (size_t k = 2; k < 5; ++k) {
				if ((make_vec(i, j, k) == make_vec<size_t>(3U, 3U, 3U)).all()) {
					continue;
				}
				storage->Set(make_vec(i, j, k));
			}
		}
	}

	ASSERT_FALSE(storage->Get(make_vec<size_t>(1U, 1U, 1U)));
	ASSERT_FALSE(storage->Get(make_vec<size_t>(3U, 3U, 3U)));

	auto storage2 = offset<>()(storage);

	ASSERT_TRUE(storage2->Get(make_vec<size_t>(1U, 1U, 1U)));
	ASSERT_FALSE(storage2->Get(make_vec<size_t>(2U, 2U, 2U)));
	ASSERT_TRUE(storage2->Get(make_vec<size_t>(3U, 3U, 3U)));
}
