#include "../traversal.h"
#include "../storage.h"

#include <iostream>

#include <gtest/gtest.h>

TEST(Traversal, Float) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 10, 10, 10, 10);

	for (int i = 2; i < 7; ++i) {
		for (int j = 2; j < 7; ++j) {
			for (int k = 2; k < 7; ++k) {
				storage->Set(make_vec(i, j, k).as<size_t>());
			}
		}
	}

	auto seed = (regular_voxel_storage*)storage->empty_copy();

	for (int i = 4; i < 5; ++i) {
		for (int j = 4; j < 5; ++j) {
			for (int k = 3; k < 6; ++k) {
				seed->Set(make_vec(i, j, k).as<size_t>());
			}
		}
	}

	visitor<26> v;
	v.max_depth = sqrt(3.);
	int count = 0;
	v([&count](const tagged_index& v) {
		count++;
	}, storage, seed);

	ASSERT_EQ(count, 45);
}

TEST(Traversal, Float2) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 10, 10, 10, 10);

	for (int i = 2; i < 7; ++i) {
		for (int j = 2; j < 7; ++j) {
			for (int k = 4; k < 5; ++k) {
				storage->Set(make_vec(i, j, k).as<size_t>());
			}
		}
	}

	auto seed = (regular_voxel_storage*)storage->empty_copy();

	for (int i = 3; i < 6; ++i) {
		for (int j = 4; j < 5; ++j) {
			for (int k = 4; k < 5; ++k) {
				seed->Set(make_vec(i, j, k).as<size_t>());
			}
		}
	}

	visitor<26> v;
	v.max_depth = 2.;
	int count = 0;
	v([&count](const tagged_index& v) {
		count++;
	}, storage, seed);

	ASSERT_EQ(count, 21);
}
