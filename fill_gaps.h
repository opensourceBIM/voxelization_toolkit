#ifndef FILL_GAPS_H
#define FILL_GAPS_H

#include "storage.h"
#include "edge_detect.h"

typedef bool(abstract_voxel_storage::*voxel_getter_t)(const vec_n<3, size_t>&) const;
typedef void(abstract_voxel_storage::*voxel_setter_t)(const vec_n<3, size_t>&);

namespace {

	void specialize_getter_(chunked_voxel_storage<bit_t>* t, voxel_getter_t& getter_, voxel_setter_t& setter_) {
		if (!t) {
			return;
		}
		getter_ = static_cast<voxel_getter_t>(&chunked_voxel_storage<bit_t>::Get);
		setter_ = static_cast<voxel_setter_t>(&chunked_voxel_storage<bit_t>::Set);
	}

	void specialize_getter_(abstract_voxel_storage* t, voxel_getter_t& getter_, voxel_setter_t& setter_) {
		if (!t) {
			return;
		}
		getter_ = &abstract_voxel_storage::Get;
		setter_ = &abstract_voxel_storage::Set;
	}

}

class fill_gaps : public post_process {
private:
	regular_voxel_storage * storage_;
	regular_voxel_storage * output_;
	std::array<vec_n<3, size_t>, 2> extents_;

	bool(abstract_voxel_storage::*getter_)(const vec_n<3, size_t>&) const;
	void(abstract_voxel_storage::*setter_)(const vec_n<3, size_t>&);

	bool get_(const vec_n<3, size_t>& pos) {
		if ((pos < extents_[0]).any()) {
			return false;
		}

		if ((pos > extents_[1]).any()) {
			return false;
		}

		return (storage_->*getter_)(pos);
	}


	void init_(regular_voxel_storage* storage) {
		storage_ = storage;
		output_ = (regular_voxel_storage*)storage_->empty_copy();
		extents_ = storage->original_bounds();
		getter_ = nullptr;
		setter_ = nullptr;
		
		// nah this is likely not going to make a difference
		specialize_getter_(storage_, getter_, setter_);
		specialize_getter_(dynamic_cast<chunked_voxel_storage<bit_t>*>(storage_), getter_, setter_);
		if (dynamic_cast<voxel_region*>(storage_)) {
			storage_ = dynamic_cast<voxel_region*>(storage_)->base();
			specialize_getter_(dynamic_cast<chunked_voxel_storage<bit_t>*>(storage_), getter_, setter_);
		}
	}

public:
	static const bool UNION_INPUT = true;

	regular_voxel_storage* operator()(regular_voxel_storage* storage) {
		init_(storage);

		size_t i0, i1, j0, j1, k0, k1;
		storage->bounds()[0].tie(i0, j0, k0);
		storage->bounds()[1].tie(i1, j1, k1);

		std::vector<vec_n<3, long>> opposites{ {1,0,0}, {0,1,0}, {0,0,1} };

		vec_n<3, size_t> ijk;
		for (long i = i0; i <= (long)i1; ++i) {
			if (progress_callback) {
				(*progress_callback)((i - i0) * 100 / (i1 - i0));
			}
			ijk.get(0) = i;
			for (long j = j0; j <= (long)j1; ++j) {
				ijk.get(1) = j;
				bool inside = false;
				for (long k = k0; k <= (long)k1; ++k) {
					ijk.get(2) = k;

					const vec_n<3, long> ijk = vec_n<3, long>(i, j, k);

					if (get_(ijk.as<size_t>())) {
						continue;
					}

					bool opposing_set = false;

					for (const auto& opp : opposites) {
						if (get_((ijk + opp).as<size_t>()) &&
							get_((ijk - opp).as<size_t>()))
						{
							opposing_set = true;
							break;
						}
					}

					if (opposing_set) {
						(output_->*setter_)(ijk.as<size_t>());
					}
				}
			}
		}

		if (progress_callback) {
			(*progress_callback)(100);
		}

		return output_;
	}
};

#endif