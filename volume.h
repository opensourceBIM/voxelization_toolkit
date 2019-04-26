#ifndef VOLUME_H
#define VOLUME_H

#include "storage.h"

class volume_filler {
private:
	regular_voxel_storage * storage_;
	bool* filled_chunks_;
	vec_n<3, size_t> num_chunks_;
	const std::array<vec_n<3, size_t>, 2>& extents_;
	
public:
	volume_filler(regular_voxel_storage* storage)
		: storage_(storage)
		, filled_chunks_(nullptr)
		, extents_(storage->bounds())
	{
		chunked_voxel_storage<bit_t>* cvs;
		if ((cvs = dynamic_cast<chunked_voxel_storage<bit_t>*>(storage))) {
			num_chunks_ = storage_->extents().ceil_div(cvs->chunk_size());
			size_t n = num_chunks_.get<0>() * num_chunks_.get<1>() * num_chunks_.get<2>();
			filled_chunks_ = new bool[n] {false};
		}
	}

	~volume_filler() {
		delete[] filled_chunks_;
	}

	bool& filled_chunk(const vec_n<3, size_t>& c) {
		return filled_chunks_[c.get(0) + (num_chunks_.get(0) * c.get(1)) + (num_chunks_.get(0) * num_chunks_.get(1) * c.get(2))];
	}

	void fill() {
		std::vector<vec_n<3, size_t>> constant_blocks_to_create;

		chunked_voxel_storage<bit_t>* cvs;
		if ((cvs = dynamic_cast<chunked_voxel_storage<bit_t>*>(storage_))) {
			auto cs = cvs->empty_chunks();
			for (auto& c : cs) {
				std::array< vec_n<3, size_t>, 2 > cext;
				cvs->chunk_extents(c, cext);
				if (storage_->ray_intersect_n(cext[0], { 0U,0U,1U }) % 2 == 1) {
					constant_blocks_to_create.push_back(c);
					filled_chunk(c) = true;
				}
			}
		}

		size_t i0, i1, j0, j1, k0, k1;
		extents_[0].tie(i0, j0, k0);
		extents_[1].tie(i1, j1, k1);

		vec_n<3, size_t> ijk;

		for (size_t i = i0; i <= i1; ++i) {
			ijk.get(0) = i;
			for (size_t j = j0; j <= j1; ++j) {
				ijk.get(1) = j;
				bool inside = false;
				bool set0 = false;
				bool set1 = false;
				for (size_t k = k0; k <= k1;) {
					ijk.get(2) = k;
					if (filled_chunk(ijk / cvs->chunk_size())) {
						k += cvs->chunk_size();
					} else {
						bool set2 = storage_->Get(ijk);
						if (!set2 && set1 && !set0) {
							inside = !inside;
						}
						if (inside) {
							storage_->Set(ijk);
						}
						set0 = set1;
						set1 = set2;
						++k;
					}
				}
			}
		}
		
		for (size_t k = k0; k <= k1; ++k) {
			ijk.get(2) = k;
			for (size_t j = j0; j <= j1; ++j) {
				ijk.get(1) = j;
				bool inside = false;
				bool set0 = false;
				bool set1 = false;
				for (size_t i = i0; i <= i1;) {
					ijk.get(0) = i;
					if (filled_chunk(ijk / cvs->chunk_size())) {
						i += cvs->chunk_size();
					} else {
						bool set2 = storage_->Get(ijk);
						if (!set2 && set1 && !set0) {
							inside = !inside;
						}
						if (inside) {
							storage_->Set(ijk);
						}
						set0 = set1;
						set1 = set2;
						++i;
					}
				}
			}
		}

		for (size_t k = k0; k <= k1; ++k) {
			ijk.get(2) = k;
			for (size_t i = i0; i <= i1; ++i) {
				ijk.get(0) = i;
				bool inside = false;
				bool set0 = false;
				bool set1 = false;
				for (size_t j = j0; j <= j1;) {
					ijk.get(1) = j;
					if (filled_chunk(ijk / cvs->chunk_size())) {
						j += cvs->chunk_size();
					} else {
						bool set2 = storage_->Get(ijk);
						if (!set2 && set1 && !set0) {
							inside = !inside;
						}
						if (inside) {
							storage_->Set(ijk);
						}
						set0 = set1;
						set1 = set2;
						++j;
					}
				}
			}
		}

		for (auto& c : constant_blocks_to_create) {
			cvs->create_constant(c, 1);
		}		
	}
};

#endif