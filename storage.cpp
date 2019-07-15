#include "storage.h"
#include "traversal.h"

set_voxel_iterator::set_voxel_iterator(regular_voxel_storage* storage, vec_n<3, size_t> current)
	: storage_(storage)
	, current_(current)
	, bounds_(storage->bounds()) {}

set_voxel_iterator& set_voxel_iterator::operator++() {
	size_t i, j, k;
	current_.tie(i, j, k);

	size_t i0, j0, k0, i1, j1, k1;
	bounds_[0].tie(i0, j0, k0);
	bounds_[1].tie(i1, j1, k1);

	goto break_into;

	for (i = i0; i <= i1; ++i) {
		for (j = j0; j <= j1; ++j) {
			for (k = k0; k <= k1; ++k) {
				if (storage_->Get({ i, j, k })) {
					current_ = { i,j,k };
					goto break_out;
				}
			break_into:
				1 == 1;
			}
		}
	}

	current_ = storage_->end().current_;

break_out:
	return *this;
}

bool set_voxel_iterator::neighbour(const vec_n<3, size_t>& d) const {
	vec_n<3, size_t> v = current_ + d;
	if ((v < bounds_[0]).any() || (v > bounds_[1]).any()) {
		return false;
	}
	return storage_->Get(v);
}

void regular_voxel_storage::obj_export(std::ostream& fs, bool with_components) {
	obj_export_helper helper(fs);
	obj_export(helper, with_components);
}

void regular_voxel_storage::obj_export(obj_export_helper& obj, bool with_components) {
	std::ostream& fs = *obj.stream;

	if (with_components) {
		size_t counter = 0;
		connected_components(this, [&obj, &fs, &counter](regular_voxel_storage* component) {
			fs << "g component" << (counter++) << "\n";
			component->obj_export(obj, false);
		});
		return;
	}

	// for progress print
	// auto N = count();
	// size_t n = 0;

	size_t& nv = obj.vert_counter;

	const double d = voxel_size();
	for (auto it = begin(); it != end(); ++it) {
		for (size_t f = 0; f < 6; ++f) {
			vec_n<3, size_t> n;
			size_t normal = f / 2;
			size_t o0 = (normal + 1) % 3;
			size_t o1 = (normal + 2) % 3;
			size_t side = f % 2;
			for (size_t i = 0; i < 3; ++i) {
				if (i == normal) {
					n.get(i) = side ? 1 : -1;
					break;
				}
			}
			if (!it.neighbour(n)) {
				std::array<vec_n<3, double >, 4> vs;
				vs.fill((*it).as<double>() * d + origin());
				vs[1].get(o0) += d;
				vs[2].get(o0) += d;
				vs[2].get(o1) += d;
				vs[3].get(o1) += d;
				if (!side) {
					std::reverse(vs.begin(), vs.end());
				}
				for (auto& v : vs) {
					if (side) {
						v.get(normal) += d;
					}
					fs << "v" << " " << v.format(false) << "\n";
				}
				fs << "f " << (nv + 0) << " " << (nv + 1) << " " << (nv + 2) << "\n";
				fs << "f " << (nv + 0) << " " << (nv + 2) << " " << (nv + 3) << "\n";
				nv += 4;
			}
		}
		/*
		if (n % 1000) {
			std::cout << n * 100 / N << std::endl;
		}
		n++;
		*/
	}
}

regular_voxel_storage* storage_for(std::array< vec_n<3, double>, 2 >& bounds, size_t max_extents, size_t padding, size_t chunk_size) {
	auto bounds_size = bounds[1] - bounds[0];
	auto voxel_size = (bounds_size / (double)max_extents).min_element();
	auto extents = (bounds_size / voxel_size).ceil().as<size_t>();

	double x1, y1, z1;
	decltype(extents)::element_type nx, ny, nz;

	(bounds[0] - (padding*voxel_size)).tie(x1, y1, z1);
	(extents + 2 * padding).tie(nx, ny, nz);

	return new chunked_voxel_storage<bit_t>(x1, y1, z1, voxel_size, nx, ny, nz, chunk_size);
}
