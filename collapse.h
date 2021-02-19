#ifndef COLLAPSE_H
#define COLLAPSE_H

#include "storage.h"

class collapse {
public:
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

		if (nonzero != 1 && D != 2 && dz != -1) {
			throw std::runtime_error("Only collapse over negative Z is implemented");
		}

		regular_voxel_storage* collapsed = (regular_voxel_storage*)storage->empty_copy();
		auto bounds = storage->bounds();

		size_t i0, j0, k0, i1, j1, k1;

		bounds[0].tie(i0, j0, k0);
		bounds[1].tie(i1, j1, k1);

		vec_n<3, size_t> ijk;
;		for (ijk.get(0) = i0; ijk.get(0) <= i1; ++ijk.get(0)) {
			for (ijk.get(1) = j0; ijk.get(1) <= j1; ++ijk.get(1)) {
				for (ijk.get(2) = k0; ijk.get(2) <= k1;) {
					if (storage->Get(ijk)) {
						collapsed->Set(ijk);
						while (storage->Get(ijk)) {
							if (++ijk.get(2) > k1) {
								break;
							}							
						}
					} else {
						++ijk.get(2);
					}
				}
			}
		}

		return collapsed;
	}
};

// @todo merge with above
class collapse_count {
public:
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

		if (nonzero != 1 && D != 2 && dz != -1) {
			throw std::runtime_error("Only collapse over negative Z is implemented");
		}

		regular_voxel_storage* collapsed = (regular_voxel_storage*)storage->empty_copy_as(voxel_uint32_t{});
		auto bounds = storage->bounds();

		size_t i0, j0, k0, i1, j1, k1;

		bounds[0].tie(i0, j0, k0);
		bounds[1].tie(i1, j1, k1);

		uint32_t count = 0;

		vec_n<3, size_t> ijk;
		;		for (ijk.get(0) = i0; ijk.get(0) <= i1; ++ijk.get(0)) {
			for (ijk.get(1) = j0; ijk.get(1) <= j1; ++ijk.get(1)) {
				for (ijk.get(2) = k0; ijk.get(2) <= k1;) {
					if (storage->Get(ijk)) {
						
						count = 0;

						auto ijk_0 = ijk;

						while (storage->Get(ijk)) {
							++count;
							if (++ijk.get(2) > k1) {
								break;
							}
						}

						collapsed->Set(ijk_0, &count);
					}
					else {
						++ijk.get(2);
					}
				}
			}
		}

		return collapsed;
	}
};

#endif
