#include "storage.h"
#include "traversal.h"

#include <map>
#include <array>

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
				if (storage_->Get(make_vec(i, j, k))) {
					current_ = make_vec(i, j, k);
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

bool set_voxel_iterator::neighbour(const vec_n<3, long>& d) const {
	vec_n<3, size_t> v = (current_.as<long>() + d).as<size_t>();
	if ((v < bounds_[0]).any() || (v > bounds_[1]).any()) {
		return false;
	}
	return storage_->Get(v);
}

void* set_voxel_iterator::value(void* val) const {
	storage_->Get(current_, val);
	return val;
}

void* set_voxel_iterator::neighbour_value(const vec_n<3, long>& d, void* val) const {
	vec_n<3, size_t> v = (current_.as<long>() + d).as<size_t>();
	if ((v < bounds_[0]).any() || (v > bounds_[1]).any()) {
		uint8_t* loc = (uint8_t*)val;
		// @todo is this correct?
		for (int i = 0; i < storage_->value_bits() / 8; ++i) {
			(*loc++) = 0;
		}
	} else {
		storage_->Get(v, val);
	}
	return val;
}

void regular_voxel_storage::obj_export(std::ostream& fs, bool with_components, bool use_value) {
	obj_export_helper helper(fs);
	obj_export(helper, with_components, use_value);
}

void regular_voxel_storage::obj_export(obj_export_helper& obj, bool with_components, bool use_value) {
	std::ostream& fs = *obj.stream;

	// Take care to only emit normals once, even though it does not really affect file integrity
	if (!obj.normals_emitted) {
		fs << "vn  1  0  0\n";
		fs << "vn -1  0  0\n";
		fs << "vn  0  1  0\n";
		fs << "vn  0 -1  0\n";
		fs << "vn  0  0  1\n";
		fs << "vn  0  0 -1\n";
		obj.normals_emitted = true;
	}
	
	
	if (with_components && !use_value) {
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

	// big enough?
	char V0[8];
	char V1[8];

	std::map< std::array<size_t, 3>, size_t > vertex_map;
	std::map< std::array<long long, 2>, std::vector< std::pair<std::array<size_t, 3>, size_t> > > triangles;

	const double d = voxel_size();
	for (auto it = begin(); it != end(); ++it) {
		it.value(V1);
		for (size_t f = 0; f < 6; ++f) {
			vec_n<3, long> n;
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
			if (use_value 
				? (
					!equal_pointed_to(value_bits() / 8, it.neighbour_value(n, V0), V1) &&
					(!(!is_zero(value_bits() / 8, V0) && !is_zero(value_bits() / 8, V1)) || side)
					)
				: !it.neighbour(n)) 
			{
				std::array< std::array<size_t, 3>, 4 > vs;
				vs.fill((*it).as_array());
				vs[1][o0] += 1;
				vs[2][o0] += 1;
				vs[2][o1] += 1;
				vs[3][o1] += 1;
				if (!side) {
					std::reverse(vs.begin(), vs.end());
				}
				for (auto& v : vs) {
					if (side) {
						v[normal] += 1;
					}
					auto inserted = vertex_map.insert({ v, vertex_map.size() + nv });
					if (inserted.second) {
						fs << "v";
						for (int i = 0; i < 3; ++i) {
							fs << " " << (v[i] * d + origin().get(i));
						}
						fs << "\n";
					}
				}
				static std::array< std::array<int, 3>, 2 > indices = {{
					{{0,1,2}},
					{{0,2,3}}
				}};
				if (!use_value) {
					for (auto& i : indices) {
						fs << "f";
						for (auto& j : i) {
							fs << " " << vertex_map.find(vs[j])->second << "//" << (f+1);
						}
						fs << "\n";
					}
				} else {
					auto va = to_number(value_bits() / 8, V0);
					auto vb = to_number(value_bits() / 8, V1);
					if (va > vb) {
						std::swap(va, vb);
					}
					for (auto& i : indices) {
						std::array<size_t, 3> arr;
						for (int j = 0; j < 3; ++j) {
							arr[j] = vertex_map.find(vs[i[j]])->second;
						}
						triangles[{ { va, vb }}].push_back({ arr, f + 1 });
					}					
				}
			}
		}
		/*
		if (n % 1000) {
			std::cout << n * 100 / N << std::endl;
		}
		n++;
		*/
	}

	for (const auto& p : triangles) {
		fs << "g " << p.first[0] << "-" << p.first[1] << "\n";
		for (const auto& t : p.second) {
			fs << "f";
			for (auto& i : t.first) {
				fs << " " << i << "//" << t.second;
			}
			fs << "\n";
		}
	}
	
	nv += vertex_map.size();
}

regular_voxel_storage* storage_for(std::array< vec_n<3, double>, 2 >& bounds, size_t max_extents, size_t padding, size_t chunk_size) {
	auto bounds_size = bounds[1] - bounds[0];
	auto voxel_size = (bounds_size / (double)max_extents).max_element();
	auto extents = (bounds_size / voxel_size).ceil().as<size_t>();

	double x1, y1, z1;
	decltype(extents)::element_type nx, ny, nz;

	(bounds[0] - (padding*voxel_size)).tie(x1, y1, z1);
	(extents + 2 * padding).tie(nx, ny, nz);

	return new chunked_voxel_storage<bit_t>(x1, y1, z1, voxel_size, nx, ny, nz, chunk_size);
}
