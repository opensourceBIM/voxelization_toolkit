#include "../writer.h"
#include "../fill_gaps.h"

#include <gtest/gtest.h>

#include <BRepPrimAPI_MakeBox.hxx>

TEST(Postprocesses, FillGaps) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 10, 10, 10, 10);

	for (size_t i = 2; i < 5; ++i) {
		for (size_t j = 2; j < 5; ++j) {
			for (size_t k = 2; k < 5; ++k) {
				auto ijk = make_vec(i, j, k);
				if ((ijk == make_vec<size_t>(3U, 3U, 3U)).all()) {
					continue;
				}
				storage->Set(ijk);
			}
		}
	}

	ASSERT_TRUE(storage->Get(make_vec<size_t>( 2U, 2U, 2U )));
	ASSERT_FALSE(storage->Get(make_vec<size_t>( 3U, 3U, 3U )));

	auto storage2 = fill_gaps()(storage);

	ASSERT_TRUE(storage2->Get(make_vec<size_t>( 3U, 3U, 3U )));
}
