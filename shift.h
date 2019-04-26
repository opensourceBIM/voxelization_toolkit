#ifndef SHIFT_H
#define SHIFT_H

#include "storage.h"

class shift {
public:
	regular_voxel_storage* operator()(abstract_voxel_storage* storage, int dx, int dy, int dz) {
		regular_voxel_storage* shifted = (regular_voxel_storage*)storage->empty_copy();
		auto bounds = storage->bounds();

		auto extents = shifted->extents().as<long>();
		auto zero = make_vec<long>(0, 0, 0);

		auto dxyz = make_vec<long>(dx, dy, dz);

		BEGIN_LOOP_I2(bounds[0], bounds[1])
			if (storage->Get(ijk)) {
				auto ijk2 = ijk.as<long>() + dxyz;
				if ((ijk2 >= zero).all() && (ijk2 < extents).all()) {
					shifted->Set(ijk2.as<size_t>());
				}
			}
		END_LOOP;

		return shifted;
	}
};

#endif