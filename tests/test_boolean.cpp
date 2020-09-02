#include "../storage.h"
#include <gtest/gtest.h>

TEST(Boolean, UnionConstantChunks) {
	const double d = 1;
	auto a = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 20, 20, 20, 10);
	auto b = new chunked_voxel_storage<bit_t>(15, 15, 15, d, 20, 20, 20, 10);
	a->create_constant(make_vec<size_t>( 0U, 0U, 0U ), 1);
	b->create_constant(make_vec<size_t>( 0U, 0U, 0U ), 1);

	auto c = a->boolean_union(b);
	ASSERT_EQ(c->count(), 2000);

	ASSERT_TRUE((c->bounds()[1] == make_vec<size_t>(19U, 19U, 19U)).all());
}

TEST(Boolean, UnionConstantChunksInPlaceInvalid) {
	const double d = 1;
	auto a = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 20, 20, 20, 10);
	auto b = new chunked_voxel_storage<bit_t>(15, 15, 15, d, 20, 20, 20, 10);
	a->create_constant(make_vec<size_t>( 0U, 0U, 0U ), 1);
	b->create_constant(make_vec<size_t>( 0U, 0U, 0U ), 1);

	ASSERT_THROW(a->boolean_union_inplace(b), std::runtime_error);
}

TEST(Boolean, UnionConstantChunksInPlace) {
	const double d = 1;
	auto a = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 20, 20, 20, 10);
	auto b = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 20, 20, 20, 10);
	a->create_constant(make_vec<size_t>( 0U, 0U, 0U ), 1);
	b->create_constant(make_vec<size_t>( 1U, 1U, 1U ), 1);

	a->boolean_union_inplace(b);
	ASSERT_EQ(a->count(), 2000);

	ASSERT_TRUE((a->bounds()[1] == make_vec<size_t>(19U, 19U, 19U)).all());
}

TEST(Boolean, SubtractPlane) {
	const double d = 1;
	auto a = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 10, 10, 10, 10);
	auto b = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 10, 10, 10, 10);
	a->create_constant(make_vec<size_t>( 0U, 0U, 0U ), 1);
	b->create_plane_primitive(make_vec<size_t>( 0U, 0U, 0U ), 2, 0);

	auto c = a->boolean_subtraction(b);
	ASSERT_EQ(c->count(), 900);

	ASSERT_TRUE((c->bounds()[0] == make_vec<size_t>(0U, 0U, 1U)).all());
}

TEST(Boolean, IntersectPlane) {
	{
		const double d = 1;
		auto a = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 10, 10, 10, 10);
		auto b = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 10, 10, 10, 10);
		a->create_constant(make_vec<size_t>( 0U, 0U, 0U ), 1);
		b->create_plane_primitive(make_vec<size_t>( 0U, 0U, 0U ), 2, 0);

		// In rare cases primitives are preserved
		auto c = (chunked_voxel_storage<bit_t>*) a->boolean_intersection(b);
		ASSERT_FALSE(c->get_chunk(make_vec<size_t>( 0U, 0U, 0U ))->is_explicit());
	}

	{
		const double d = 1;
		auto a = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 10, 10, 10, 10);
		auto b = new chunked_voxel_storage<bit_t>(0, 0, 0, d, 10, 10, 10, 10);
		a->create_plane_primitive(make_vec<size_t>( 0U, 0U, 0U ), 1, 0);
		b->create_plane_primitive(make_vec<size_t>( 0U, 0U, 0U ), 2, 0);

		// In rare cases primitives are preserved
		auto c = (chunked_voxel_storage<bit_t>*) a->boolean_intersection(b);
		ASSERT_TRUE(c->get_chunk(make_vec<size_t>( 0U, 0U, 0U ))->is_explicit());
		ASSERT_EQ(c->count(), 10);
	}
}

