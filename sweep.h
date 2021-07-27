#ifndef SWEEP_H
#define SWEEP_H

#include "storage.h"

class sweep {
public:
	abstract_voxel_storage* until = nullptr;
	boost::optional<int> max_depth;

	regular_voxel_storage* operator()(abstract_voxel_storage* storage, int dx, int dy, int dz) {
		int d[3] = { dx, dy, dz };
		int nonzero = 0;
		int D = -1;
		for (int i = 0; i < 3; ++i) {
			if (d[i] != 0) {
				nonzero++;
				D = i;
			}
		}

		if (nonzero != 1) {
			throw std::runtime_error("Only orthogonal sweeps supported");
		}

		uint32_t v = 1;
		const bool use_count = storage->value_bits() == 32;

		regular_voxel_storage* swepts = (regular_voxel_storage*) storage->copy();
		auto bounds = storage->bounds();

		auto extents = swepts->extents().as<long>();
		auto zero = make_vec<long>(0, 0, 0);

		bool pos = d[D] > 0;

		BEGIN_LOOP_I2(bounds[0], bounds[1])
			if (storage->Get(ijk)) {
				if (use_count) {
					storage->Get(ijk, &v);
				}
				auto ijk2 = ijk.as<long>();
				int i = 1;
				while (until || (i < std::abs(d[D]) * v)) {

					if (pos) {
						ijk2.get(D)++;
					}
					else {
						ijk2.get(D)--;
					}

					if (!((ijk2 >= zero).all() && (ijk2 < extents).all())) {
						break;
					}

					if (until) {
						if (until->Get(ijk2.as<size_t>())) {
							break;
						}
					}
					
					swepts->Set(ijk2.as<size_t>());

					if (max_depth && i >= *max_depth) {
						break;
					}

					++i;
				}
			}
		END_LOOP;

		
		return swepts;
	}
};

#endif
