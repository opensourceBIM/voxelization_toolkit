#include "../memoized_traversal.h"
#include "dim3.h"

#include <chrono>
#include <iostream>
#include <unordered_set>

#include <gtest/gtest.h>

TEST(Vec, Hashing) {
	auto vec = make_vec(1, 2, 3);
	std::hash<vec_n<3, int>> hash_fn;
	auto vec2 = make_vec(1, 2, 3);
	ASSERT_EQ(hash_fn(vec), hash_fn(vec));
}

TEST(Traversal, Comparison) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 10, 10, 10, 10);

	for (size_t i = 2; i < 7; ++i) {
		for (size_t j = 2; j < 7; ++j) {
			for (size_t k = 2; k < 7; ++k) {
				if (((make_vec(4, 4, 4) - make_vec(i, j, k).as<int>()).abs() < 2).all()) {
					continue;
				}
				storage->Set(make_vec(i, j, k));
			}
		}
	}

	ASSERT_EQ(storage->count(), 5 * 5 * 2 + 5 * 3 * 2 + 3 * 3 * 2);
	ASSERT_EQ(storage->count(), 5 * 5 * 5 - 3 * 3 * 3);

	/*
	for (size_t k = 2; k < 4; ++k) {
		for (size_t j = 2; j < 4; ++j) {
			for (size_t i = 2; i < 9; ++i) {
				storage->Set(make_vec(i, j, k));
			}
		}
	}

	ASSERT_EQ(storage->count(), 7 * 2 * 2);
	*/

	double accum_1 = 0.;
	double accum_2 = 0.;
	
	using std::chrono::duration_cast;
	using std::chrono::duration;
	using std::chrono::high_resolution_clock;

	auto t1 = high_resolution_clock::now();
	memoized_traversal trav(storage, 1000);
	auto t2 = high_resolution_clock::now();
	duration<double, std::milli> ms = t2 - t1;
	accum_2 += ms.count();

	for (int k = 2; k < 3; ++k) {
		for (int j = 2; j < 5; ++j) {
			auto seed = make_vec<size_t>(j, k, 2);
			std::cout << "From: " << seed.format() << std::endl;

			visitor<> v;
			v.max_depth = 16;
			int count = 0;

			std::vector<vec_n<3, size_t>> vecs;

			t1 = high_resolution_clock::now();
			v([&count, &vecs](const tagged_index& v) {
				vecs.push_back(v.pos);
				count++;
			}, storage, seed);
			t2 = high_resolution_clock::now();
			ms = t2 - t1;
			accum_1 += ms.count();

			std::sort(vecs.begin(), vecs.end(), [](auto& a, auto& b) { return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()); });
			for (auto& p : vecs) {
				std::cerr << p.format() << std::endl;
			}
			std::cerr << "vs" << std::endl;
			vecs.clear();

			std::unordered_map<vec_n<3, size_t>, size_t> out;
			t1 = high_resolution_clock::now();
			trav(seed, (size_t)*v.max_depth, out);
			t2 = high_resolution_clock::now();
			ms = t2 - t1;
			accum_2 += ms.count();
			for (auto& p : out) {
				vecs.push_back(p.first);
			}
			std::sort(vecs.begin(), vecs.end(), [](auto& a, auto& b) { return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()); });
			for (auto& p : vecs) {
				std::cerr << p.format() << std::endl;
			}

			ASSERT_EQ(out.size(), count);

			std::cerr << "-------------------" << std::endl;
		}
	}
	std::cerr << accum_1 << " vs " << accum_2 << std::endl;
	std::cerr << "h: " << trav.cache_hits << " m: " << trav.cache_misses << std::endl;
}

TEST(DISABLED_Traversal, Squaring) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 10, 10, 10, 10);

	for (size_t i = 2; i < 7; ++i) {
		for (size_t j = 2; j < 7; ++j) {
			for (size_t k = 2; k < 7; ++k) {
				if (((make_vec(4, 4, 4) - make_vec(i, j, k).as<int>()).abs() < 2).all()) {
					continue;
				}
				storage->Set(make_vec(i, j, k));
			}
		}
	}

	ASSERT_EQ(storage->count(), 5 * 5 * 2 + 5 * 3 * 2 + 3 * 3 * 2);
	ASSERT_EQ(storage->count(), 5 * 5 * 5 - 3 * 3 * 3);

	double accum_1 = 0.;
	double accum_2 = 0.;

	using std::chrono::duration_cast;
	using std::chrono::duration;
	using std::chrono::high_resolution_clock;

	auto t1 = high_resolution_clock::now();
	squaring_traversal trav(storage);
	auto t2 = high_resolution_clock::now();
	duration<double, std::milli> ms = t2 - t1;
	accum_2 += ms.count();

	for (int k = 2; k < 4; ++k) {
		for (int j = 2; j < 7; ++j) {
			auto seed = make_vec<size_t>(j, k, 2);

			visitor<> v;
			v.max_depth = 32;
			int count = 0;

			std::vector<vec_n<3, size_t>> vecs;

			t1 = high_resolution_clock::now();
			v([&count, &vecs](const tagged_index& v) {
				vecs.push_back(v.pos);
				count++;
			}, storage, seed);
			t2 = high_resolution_clock::now();
			ms = t2 - t1;
			accum_1 += ms.count();

			std::sort(vecs.begin(), vecs.end(), [](auto& a, auto& b) { return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()); });
			for (auto& p : vecs) {
				std::cerr << p.format() << std::endl;
			}
			std::cerr << std::endl << std::endl;
			vecs.clear();

			squaring_traversal::set_type out;
			t1 = high_resolution_clock::now();
			trav(seed, out);
			t2 = high_resolution_clock::now();
			ms = t2 - t1;
			accum_2 += ms.count();

			std::transform(out.begin(), out.end(), std::back_inserter(vecs), [](auto& v) {
				return make_vec<size_t>(v.v[0], v.v[1], v.v[2]);
			});
			std::sort(vecs.begin(), vecs.end(), [](auto& a, auto& b) { return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()); });
			for (auto& p : vecs) {
				std::cerr << p.format() << std::endl;
			}

			ASSERT_EQ(out.size(), count);

			std::cout << "-------------------" << std::endl;
		}
	}
	std::cerr << accum_1 << " vs " << accum_2 << std::endl;
	// std::cerr << "h: " << trav.cache_hits << " m: " << trav.cache_misses << std::endl;
}
