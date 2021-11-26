#ifndef RESAMPLE_H
#define RESAMPLE_H

#include "storage.h"

// Function so that integer division is selected
template <typename T>
T multiply_or_divide(const T& t, int factor, bool ceil) {
	if (factor < 0) {
		/*if (ceil && std::is_integral<T>::value) {
			integer_ceil_div(t, (T)(-factor));
		} else {*/
			return t / (-factor);
		// }
	} else {
		return t * factor;
	}
}


template <typename U>
vec_n<3, U> multiply_or_divide(const vec_n<3, U>& t, int factor, bool ceil) {
	if (factor < 0) {
		if (ceil) {
			return t.ceil_div(-factor);
		} else {
			return t.floor_div(-factor);
		}
	} else {
		return t * ((U)factor);
	}
}


class resampler {
private:
	// An integer factor. Negative means division by absolute
	int factor_;

public:
	resampler(int factor)
		: factor_(factor)
	{}

	regular_voxel_storage* operator()(regular_voxel_storage* storage) {
		chunked_voxel_storage<bit_t>* input = (chunked_voxel_storage<bit_t>*) storage;

		// left is floored
		auto left = multiply_or_divide(input->grid_offset(), factor_, false);
		// right is ceiled
		auto right = multiply_or_divide((input->grid_offset() + input->num_chunks().as<long>()), factor_, true);

		chunked_voxel_storage<bit_t>* result = new chunked_voxel_storage<bit_t>(
			left,
			multiply_or_divide(input->voxel_size(), -factor_, false),
			input->chunk_size(),
			(right - left).as<size_t>()
		);

		// upsampling retains all primitives. downsampling can only retain primitives
		// if there are similar adjacent primitives, but this is currently not checked

		const bool is_upsampling = factor_ > 0;

		if (is_upsampling) {
			auto block = make_vec<int>(factor_, factor_, factor_).as<size_t>();

			BEGIN_LOOP_ZERO_2(input->num_chunks())
				const auto& chunk_idx = ijk;
				auto c = input->get_chunk(ijk);
				auto ijk2 = ijk * ((size_t)factor_);
				if (c != nullptr) {
					if (c->is_constant()) {
						BEGIN_LOOP_ZERO_2(block)
							result->create_constant(ijk2 + ijk, 1);
						END_LOOP;
					} else if (!c->is_explicit()) {
						auto p = (planar_voxel_storage<bit_t>*) c;
						for (auto& i : p->offsets()) {
							BEGIN_LOOP_ZERO_2(block)
								if ((i * factor_) / result->chunk_size() == ijk.get(p->axis())) {
									result->create_plane_primitive(ijk2 + ijk, p->axis(), (i * factor_) % result->chunk_size());
								}
							END_LOOP;
						}						
					} else {
						auto cv = (continuous_voxel_storage<bit_t>*) c;
						for (auto& v : (*cv)) {
							BEGIN_LOOP_ZERO_2(block)
								result->Set((v + chunk_idx * input->chunk_size()) * ((size_t)factor_) + ijk);
							END_LOOP;
						}
					}
				}
			END_LOOP;
		} else {
			auto block = make_vec<int>(-factor_, -factor_, -factor_).as<size_t>();
			auto grid_difference_in_voxels = input->grid_offset() * (long)input->chunk_size() / ((long)(-factor_)) - result->grid_offset() * (long)input->chunk_size();
			// std::cout << "grid_difference_in_voxels " << grid_difference_in_voxels.format() << std::endl;

			// @todo constrain loop to input so that no further check is necessary?
			BEGIN_LOOP_ZERO_2(result->extents())
				auto ijk2 = ijk * ((size_t)(-factor_));
				if ((ijk2 < input->extents()).all()) {
					bool any = false;
					BEGIN_LOOP_ZERO_2(block)
						if (input->Get(ijk2 + ijk)) {
							any = true;
							goto outer;
						}
					END_LOOP;

				outer:
					if (any) {
						// @todo double check that this does not underflow
						result->Set((ijk.as<long>() + grid_difference_in_voxels).as<size_t>());
					}
				}
			END_LOOP;
		}

		return result;
	}

};

#endif