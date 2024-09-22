#include "../voxec.h"

#include <gtest/gtest.h>

#include <chrono>

TEST(Dimensionality, Estimate) {
	using std::chrono::duration_cast;
	using std::chrono::duration;
	using std::chrono::high_resolution_clock;

	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 10, 10, 10, 10);

	for (size_t i = 1; i < 8; ++i) {
		for (size_t j = 1; j < 8; ++j) {
			for (size_t k = 1; k < 8; ++k) {
				storage->Set(make_vec(i, j, k));
			}
		}
	}

	std::array<double, 4> times_ms;

	for (int i = 0; i < 4; ++i) {
		scope_map args;

		args["input"] = storage;
		args["max_depth"] = 5;
		args["max_depth_2"] = 2;
		args["reposition"] = 1;

		if (i == 1) {
			args["inward_distance_approximate"] = 1;
			args["subsampling_factor"] = 2;
		}
		if (i == 2) {
			args["inward_distance_skip"] = 1;
		}
		if (i == 3) {
			args["THREADS"] = 4;
		}

		auto t1 = high_resolution_clock::now();
		auto v = op_dimensionality_estimate{}.invoke(args);
		auto t2 = high_resolution_clock::now();
		duration<double, std::milli> ms = t2 - t1;

		std::cerr << "Time (ms) " << (times_ms[i] = ms.count()) << std::endl;

		ASSERT_EQ(v.which(), 3);

		auto vp = boost::get<abstract_voxel_storage*>(v);

		ASSERT_EQ(vp->value_bits(), 4 * 2 * 8);

		normal_and_curvature<int16_t> vi16;

		static auto ONE_DIV_SQRT_THREE_F = 1.f / std::sqrt(3.f);

		for (auto ijk : *storage) {
			vp->Get(ijk, &vi16);
			auto n = vi16.convert<float>().normal();
			ASSERT_TRUE(((n - ONE_DIV_SQRT_THREE_F) < 1.e-5).all());
		}
	}

	ASSERT_LT(times_ms[1] * 1.5, times_ms[0]);
	ASSERT_LT(times_ms[2] * 1.1, times_ms[1]);
	ASSERT_LT(times_ms[3] * 3, times_ms[0]);
}
