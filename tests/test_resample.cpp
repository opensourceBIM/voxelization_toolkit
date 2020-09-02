#include "../resample.h"

#include <gtest/gtest.h>

TEST(DownSampling, Constant) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 100, 100, 100, 32);
	storage->create_constant(make_vec<size_t>(0U,0U,0U), 1);
	auto result = resampler(-4)(storage);
	ASSERT_EQ(storage->count(), result->count() * 4 * 4 * 4);
}

TEST(UpSampling, Constant) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 100, 100, 100, 32);
	storage->create_constant(make_vec<size_t>(0U,0U,0U), 1);
	auto result = (chunked_voxel_storage<bit_t>*) resampler(+2)(storage);
	BEGIN_LOOP(size_t(0), 2U, 0U, 2U, 0U, 2U)
		ASSERT_TRUE(result->get_chunk(ijk)->is_constant());
	END_LOOP;
}

TEST(UpSampling, Plane) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 100, 100, 100, 32);
	// Z=20
	storage->create_plane_primitive(make_vec<size_t>(0U,0U,0U), 2, 20);
	auto result = (chunked_voxel_storage<bit_t>*) resampler(+2)(storage);
	ASSERT_TRUE(result->get_chunk(make_vec<size_t>(0U, 0U, 0U)) == nullptr);
	ASSERT_FALSE(result->get_chunk(make_vec<size_t>(0U, 0U, 1U))->is_explicit());
	// Z = (20 * 2) % 32 = 8
	ASSERT_TRUE(result->get_chunk(make_vec<size_t>(0U, 0U, 1U))->Get(make_vec<size_t>(0U, 0U, 8U)));
}

TEST(UpSampling, Continuous) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 100, 100, 100, 32);
	storage->Set(make_vec<size_t>(1U,1U,1U ));
	auto result = (chunked_voxel_storage<bit_t>*) resampler(+2)(storage);
	ASSERT_FALSE(result->Get(make_vec<size_t>(1U, 1U, 1U )));
	ASSERT_TRUE(result->Get(make_vec<size_t>(2U, 2U, 2U )));
	ASSERT_TRUE(result->Get(make_vec<size_t>(3U, 3U, 3U )));
	ASSERT_FALSE(result->Get(make_vec<size_t>(4U, 4U, 4U )));
}
