#ifndef OFFSET_H
#define OFFSET_H

#include "storage.h"
#include "edge_detect.h"

template <size_t N=1U, size_t D=3U>
class offset : public post_process {
private:
	regular_voxel_storage * storage_;
	regular_voxel_storage * output_;
	std::array<vec_n<3, size_t>, 2> extents_;
	bool same_dim_;

	bool get_(const vec_n<3, size_t>& pos) {
		if ((pos < extents_[0]).any()) {
			return false;
		}

		if ((pos > extents_[1]).any()) {
			return false;
		}

		return storage_->Get(pos);
	}

	void init_(regular_voxel_storage* storage) {
		storage_ = storage;

		bool enough_padding = true;

		regular_voxel_storage* base = storage;
		if (dynamic_cast<voxel_region*>(storage_)) {
			base = dynamic_cast<voxel_region*>(storage_)->base();
		}

		if (false && ((base->bounds()[0] < N).any() || ((base->extents() - base->bounds()[1]) < (N + 1U)).any())) {

			// @todo output might be truncated.

			same_dim_ = false;

			double ox, oy, oz, d = storage->voxel_size();
			size_t dimx, dimy, dimz, chunk_size = 128;

			storage_->origin().tie(ox, oy, oz);
			storage_->extents().tie(dimx, dimy, dimz);

			if (dynamic_cast<voxel_region*>(storage_)) {
				regular_voxel_storage* base = dynamic_cast<voxel_region*>(storage_)->base();
				base->origin().tie(ox, oy, oz);
				base->extents().tie(dimx, dimy, dimz);
			}

			// Grow voxel volume
			ox -= d * N;
			oy -= d * N;
			oz -= d * N;
			dimx += 2 * N;
			dimy += 2 * N;
			dimz += 2 * N;

			if (dynamic_cast<chunked_voxel_storage<bit_t>*>(storage_)) {
				chunk_size = dynamic_cast<chunked_voxel_storage<bit_t>*>(storage_)->chunk_size();
			}

			output_ = new chunked_voxel_storage<bit_t>(
				ox, oy, oz, d, dimx, dimy, dimz, chunk_size
				);

		} else {

			same_dim_ = true;

			output_ = (regular_voxel_storage*) storage_->empty_copy();

		}

		extents_ = storage->original_bounds();
	}

public:
	static const bool UNION_INPUT = false;

	regular_voxel_storage * operator()(regular_voxel_storage* storage) {
		init_(storage);

		size_t i0, i1, j0, j1, k0, k1;
		storage->bounds()[0].tie(i0, j0, k0);
		storage->bounds()[1].tie(i1, j1, k1);

		auto extents = output_->extents().as<long>();
		auto zero = make_vec<long>(0, 0, 0);

		vec_n<3, size_t> ijk;
		for (long i = i0; i <= (long)i1; ++i) {
			progress(static_cast<float>(i - i0) / (i1 - i0));

			ijk.get(0) = i;
			for (long j = j0; j <= (long)j1; ++j) {
				ijk.get(1) = j;
				bool inside = false;
				for (long k = k0; k <= (long)k1; ++k) {
					ijk.get(2) = k;

					const vec_n<3, long> ijk = vec_n<3, long>(i, j, k);

					if (!get_(ijk.as<size_t>())) {
						continue;
					}

					for (long di = -1; di <= 1; ++di) {
						for (long dj = -1; dj <= 1; ++dj) {
							for (long dk = (D == 3 ? -1 : 0); dk <= (D == 3 ? 1 : 0); ++dk) {
								const vec_n<3, long> ijk2 = ijk + vec_n<3, long>(di, dj, dk);
								if ((ijk2 >= zero).all() && (ijk2 < extents).all()) {
									if (!get_(ijk2.as<size_t>())) {
										if (same_dim_) {
											output_->Set(ijk2.as<size_t>());
										} else {
											output_->Set(ijk2.as<size_t>() + vec_n<3, size_t>(N, N, N));
										}
									}
								}
							}
						}
					}
				}
			}
		}

		progress(1.);

		return output_;
	}
};

#endif