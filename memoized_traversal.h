#ifndef MEMOIZED_TRAVERSAL_H
#define MEMOIZED_TRAVERSAL_H

#include "lru_cache.h"
#include "storage.h"
#include "traversal.h"

#include <unordered_set>

namespace std {
	// Specialization of std::equal_to for vec_n, because operator== is overloaded and
	// returns vec_n<bool>
	template <>
	struct equal_to<std::pair<vec_n<3, size_t>, size_t>> {
		bool operator()(const std::pair<vec_n<3, size_t>, size_t>& lhs, const std::pair<vec_n<3, size_t>, size_t>& rhs) const {
			return lhs.first.as_array() == rhs.first.as_array() && lhs.second == rhs.second;
		}
	};

	// Specialization of std::equal_to for vec_n, because operator== is overloaded and
	// returns vec_n<bool>
	template <>
	struct hash<std::pair<vec_n<3, size_t>, size_t>> {
		size_t operator()(const std::pair<vec_n<3, size_t>, size_t>& p) const {
			return std::hash<decltype(p.first)>{}(p.first) ^ std::hash<decltype(p.second)>{}(p.second);
		}
	};
};

/*
class memoized_traversal {
	// use 6-connectedness for commutativity
	LRUCache<std::pair<vec_n<3, size_t>, size_t>, std::set<std::array<size_t, 3>>> cache_;
	regular_voxel_storage* storage_;
	regular_voxel_storage* visited_;

public:
	size_t cache_hits = 0;
	size_t cache_misses = 0;

	memoized_traversal(regular_voxel_storage* storage, size_t capacity)
		: cache_(capacity)
		, storage_(storage)
		, visited_(nullptr)
	{
		clear();
	}

	void clear() {
		delete visited_;
		visited_ = (regular_voxel_storage*)storage_->empty_copy();
	}

	void operator()(const vec_n<3, size_t>& cur, size_t d, size_t neighbourhood_size, std::set<std::array<size_t, 3>>& out) {
		if (visited_->Get(cur)) {
			return;
		}
		// d can only reduce, so keeping track of visited is valid
		visited_->Set(cur);
		std::set<std::array<size_t, 3>> v = { { cur.get<0>(), cur.get<1>(), cur.get<2>() } };
		if (cache_.get({ cur, d }, v)) {
			cache_hits += 1;
		} else if (d == 0) {
		} else {
			cache_misses += 1;
			// // with depth=1 you obviously don't need a queue
			// visitor<6> visitor_;
			// visitor_.max_depth = 1;
			// visitor_([&v, neighbourhood_size, d, this](const tagged_index& pos) {
			// 	if (pos.which == tagged_index::VOXEL) {
			// 		if (!neighbourhood_size || (v.size() < neighbourhood_size)) {
			// 			// out.push_back({ pos.pos.get(0),pos.pos.get(1),pos.pos.get(2) });
			// 			(*this)(pos.pos, d - 1, neighbourhood_size, v);
			// 		}
			// 	} else {
			// 		throw std::runtime_error("Unexpected");
			// 	}
			// }, storage_, i);
			auto pos = cur;
			for (size_t i = 0; i < 3; ++i) {
				pos.get(i)--;
				for (size_t j = 0; j < 2; ++j) {
					if (storage_->Get(pos)) {
						(*this)(pos, d - 1, neighbourhood_size, v);
					}
					pos.get(i) += 2;
				}
				pos.get(i) -= 3;
			}
			cache_.insert({ cur, d }, v);
		}
		out.insert(v.begin(), v.end());
	}
};
*/

class memoized_traversal {
	// use 6-connectedness for commutativity
	LRUCache<std::pair<vec_n<3, size_t>, size_t>, std::vector<vec_n<3, size_t>>> cache_;
	regular_voxel_storage* storage_;
	regular_voxel_storage* visited_;

public:
	size_t cache_hits = 0;
	size_t cache_misses = 0;

	memoized_traversal(regular_voxel_storage* storage, size_t capacity)
		: cache_(capacity)
		, storage_(storage)
		, visited_(nullptr)
	{
		clear();
	}

	void clear() {
	}

	void operator()(const vec_n<3, size_t>& cur, size_t d, std::unordered_map<vec_n<3, size_t>, size_t>& out, size_t doff = 0) {
		visited_ = (regular_voxel_storage*)storage_->empty_copy();

		std::deque<std::pair<vec_n<3, size_t>, size_t>> queue = { { cur, 0} };

		while (!queue.empty()) {
			auto& current = queue.front();
			auto& pos = current.first;
			out.insert(current);

			for (size_t i = 0; i < 3; ++i) {
				pos.get(i)--;
				for (size_t j = 0; j < 2; ++j) {
					if (current.second < d && storage_->Get(pos) && !visited_->Get(pos)) {
						visited_->Set(pos);

						out.insert({ pos, current.second + 1 });

						bool used_cache = false;
						std::vector<vec_n<3, size_t>> vs;
						auto D = d - current.second - 1;
						while (cache_.get({ pos, D }, vs)) {
							used_cache = true;
							for (auto& v : vs) {
								if (!visited_->Get(v)) {
									out.insert({ v, current.second + D });
									visited_->Set(v);
								}
							}
							D--;
						}
						if (!used_cache) {
							cache_misses += 1;
							queue.push_back({ pos, current.second + 1 });
						} else {
							cache_hits += 1;
						}
					}
					pos.get(i) += 2;
				}
				pos.get(i) -= 3;
			}
			queue.pop_front();
		}

		std::vector<std::vector<vec_n<3, size_t>>> vss(d + 1);

		for (auto& p : out) {
			vss[p.second].push_back(p.first);
			out.insert({ p.first, p.second});
		}

		for (size_t i = 0; i < d + 1; ++i) {
			cache_.insert({ cur, i }, vss[i]);
		}
	}
};

struct vec_with_hash {
	std::array<size_t, 4> v;

	vec_with_hash(const vec_n<3, size_t>& vec) {
		v = { vec.get<0>(), vec.get<1>(), vec.get<2>(), std::hash<vec_n<3, size_t>>{}(vec) };
	}
};


namespace std {
	template <>
	struct hash<vec_with_hash> {
		size_t operator()(const vec_with_hash& vec) const {
			return vec.v.back();
		}
	};

	template <>
	struct equal_to<vec_with_hash> {
		bool operator()(const vec_with_hash& lhs, const vec_with_hash& rhs) const {
			return lhs.v == rhs.v;
		}
	};
}

class squaring_traversal {
public:
	using set_type = std::unordered_set<vec_with_hash>;
private:
	std::unordered_map<vec_with_hash, set_type> neighbour_map;
	regular_voxel_storage* storage_;

public:
	squaring_traversal(regular_voxel_storage* storage)
		: storage_(storage)
	{
		for (auto ijk : *storage) {
			auto pos = ijk;
			for (size_t i = 0; i < 3; ++i) {
				pos.get(i)--;
				for (size_t j = 0; j < 2; ++j) {
					if (storage_->Get(pos)) {
						neighbour_map[ijk].insert(pos);
					}
					pos.get(i) += 2;
				}
				pos.get(i) -= 3;
			}
		}
		std::vector<decltype(neighbour_map)> maps = { neighbour_map };

		for (int N = 0; N < 5; ++N) {
			maps.push_back(maps.back());
			auto& prev = *(++maps.rbegin());
			for (auto& p : prev) {
				for (auto& q : p.second) {
					// @todo this doesn't work in case of concavities, where topological
					// distance is 
					auto D = std::abs((int)(p.first.v[0] - q.v[0])) + std::abs((int)(p.first.v[1] - q.v[1])) + std::abs((int)(p.first.v[2] - q.v[2]));
					if ((1 << N) == D) {
						maps.back()[p.first].insert(prev[q].begin(), prev[q].end());
					}
				}
			}
		}

		neighbour_map = maps.back();
	}

	void operator()(const vec_n<3, size_t>& seed, set_type& out) {
		out = neighbour_map[seed];
	}
};


#endif