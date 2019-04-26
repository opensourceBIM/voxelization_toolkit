#ifndef TRAVERSAL_H
#define TRAVERSAL_H

#include "storage.h"
#include "edge_detect.h"

#include <deque>

struct DOF_XYZ {
	static const bool use_chunks = true;
	static const size_t dims = 3;
};

struct DOF_XY {
	static const bool use_chunks = false; // simpler this way, but probably not necessary
	static const size_t dims = 2;
};

struct POST_CHECK_ALWAYS {
	inline bool operator()(const vec_n<3, size_t>& pos) {
		return true;
	}
};

struct tagged_index {
	enum index_type {
		CHUNK, VOXEL
	};
	index_type which;
	vec_n<3, size_t> pos;
};

template <typename DofT = DOF_XYZ, typename PostT = POST_CHECK_ALWAYS>
class visitor {
private:
	regular_voxel_storage* storage_;
	regular_voxel_storage* visited_;
	std::array<vec_n<3, size_t>, 2> bounds_;
	vec_n<3, size_t> extents_;
	multi_dim_array<int, 3> chunks_;
	bool is_chunked_;
	size_t chunk_size_;
	int search_value_;
	PostT post_condition_;

	static const int VOXEL_OUT_OF_BOUNDS = -1;
	static const int VOXEL_FALSE = 0;
	static const int VOXEL_TRUE = 1;
	
	static const int CHUNK_MIXED = -1;
	static const int CHUNK_EMPTY = 0;
	static const int CHUNK_FILLED = 1;

	int get_(const vec_n<3, size_t>& pos, bool v = false) {
		if (v) {
			return visited_->Get(pos) ? VOXEL_TRUE : VOXEL_FALSE;
		}

		if ((pos < bounds_[0]).any()) {
			return VOXEL_FALSE;
		}

		if ((pos > bounds_[1]).any()) {
			return VOXEL_FALSE;
		}

		return storage_->Get(pos) ? VOXEL_TRUE : VOXEL_FALSE;
	}

	bool is_visited_(const vec_n<3, size_t>& pos) {
		return get_(pos, true) == VOXEL_TRUE;
	}

	void set_visited_(const vec_n<3, size_t>& pos) {
		// std::cerr << " visited " << pos.format() << std::endl;
		visited_->Set(pos);
	}

	void set_visited_chunk_(const vec_n<3, size_t>& pos) {
		// std::cerr << " visited chunk " << pos.format() << std::endl;
		((chunked_voxel_storage<bit_t>*)visited_)->create_constant(pos, 1);
	}
	
	void init_(regular_voxel_storage* storage) {
		storage_ = storage;
		visited_ = (regular_voxel_storage*) storage->empty_copy();
		bounds_ = storage->bounds();
		extents_ = storage->extents();
		is_chunked_ = false;

		abstract_chunked_voxel_storage* cvs;
		if ((cvs = dynamic_cast<abstract_chunked_voxel_storage*>(storage))) {
			is_chunked_ = true;
			chunk_size_ = cvs->chunk_size();
			auto n = storage_->extents().ceil_div(cvs->chunk_size());
			size_t nx, ny, nz;
			n.tie(nx, ny, nz);
			chunks_.resize(nullptr, nx, ny, nz);
			BEGIN_LOOP(size_t(0), nx, 0U, ny, 0U, nz)
				auto c = cvs->get_chunk(ijk);
				chunks_.get(ijk.get<0>(), ijk.get<1>(), ijk.get<2>()) = CHUNK_MIXED;
				if (c == nullptr || c->count() == 0) {
					chunks_.get(ijk.get<0>(), ijk.get<1>(), ijk.get<2>()) = CHUNK_EMPTY;
				} else {
					// chunk_size_ ** 3
					auto sz = c->extents().get<0>() * c->extents().get<1>() * c->extents().get<2>();
					if (c->count() == sz) {
						chunks_.get(ijk.get<0>(), ijk.get<1>(), ijk.get<2>()) = CHUNK_FILLED;
					}
				}
			END_LOOP;
		}
	}

	void chunk_neighbours_queue_add_(const vec_n<3, size_t>& cijk) {
		for (size_t i = 0; i < 3; ++i) {
			for (size_t j = 0; j < 2; ++j) {
				if (j == 0 && cijk.get(i) == 0) {
					went_out_of_bounds = true;
					continue;
				}
				
				auto cijk2 = cijk;

				if (j == 0) {
					cijk2.get(i) -= 1U;
				} else {
					cijk2.get(i) += 1U;
				}

				if (j == 1 && cijk2.get(i) >= chunks_.dimensions().at(i)) {
					went_out_of_bounds = true;
					continue;
				}

				auto v = chunks_.get(cijk2.get<0>(), cijk2.get<1>(), cijk2.get<2>());
				if (v == search_value_) {
					// neighbour chunk is same constant value
					// only a single voxel position within the chunk is added to queue
					auto lower = cijk2 * chunk_size_;
					queue.push_back(lower);
				} else if (v == CHUNK_MIXED) {
					// calculate neighbour plane
					// @todo is it necessary to cast to long here?
					auto lower = (cijk2 * chunk_size_).as<long>();
					auto upper = ((cijk2 + 1U) * chunk_size_).as<long>();
					if (j == 0) {
						lower.get(i) = upper.get(i) - 1;
					} else {
						upper.get(i) = lower.get(i) + 1;
					}
					BEGIN_LOOP2(lower, upper)
						if (get_(ijk.as<size_t>()) == search_value_) {
							queue.push_back(ijk.as<size_t>());
						}
					END_LOOP;
				}
			}
		}
	}

	void neighbours_queue_add_(const vec_n<3, size_t>& current) {
		for (size_t i = 0; i < 3; ++i) {
			for (size_t j = 0; j < 2; ++j) {
				if (j == 0 && current.get(i) == 0) {
					went_out_of_bounds = true;
					continue;
				}

				auto pos = current;

				if (j == 0) {
					pos.get(i)--;
				} else {
					pos.get(i)++;
				}

				if (j == 1 && (pos >= extents_).any()) {
					went_out_of_bounds = true;
					continue;
				}

				auto v = get_(pos);
				if (v == search_value_) {
					queue.push_back(pos);
				}
			}
		}
	}

	template <typename Fn>
	void process_(Fn fn, const vec_n<3, size_t>& pos) {
		if (is_visited_(pos)) {
			return;
		}

		auto c = pos / chunk_size_;
		if (DofT::use_chunks && is_chunked_ && chunks_.get(c.get<0>(), c.get<1>(), c.get<2>()) != CHUNK_MIXED) {
			auto lower = c * chunk_size_;
			auto upper = (c + 1U) * chunk_size_;

			fn(tagged_index{ tagged_index::CHUNK, c });

			chunk_neighbours_queue_add_(c);
			set_visited_chunk_(c);
		} else if (post_condition_(pos)) {
			fn(tagged_index{ tagged_index::VOXEL, pos });

			neighbours_queue_add_(pos);
			set_visited_(pos);
		}
	}

public:
	std::deque<vec_n<3, size_t>> queue;
	bool went_out_of_bounds;

	visitor() : post_condition_(POST_CHECK_ALWAYS()) {}
	explicit visitor(const PostT& p) : post_condition_(p) {}

	template <typename Fn>
	void operator()(Fn fn, regular_voxel_storage* storage, const vec_n<3, size_t>& seed) {
		init_(storage);
		went_out_of_bounds = false;

		search_value_ = get_(seed);
		queue = { };
		process_(fn, seed);

		while (!queue.empty()) {
			const vec_n<3, size_t>& current = queue.front();
			process_(fn, current);
			queue.pop_front();
		}		
	}

	template <typename Fn>
	void operator()(Fn fn, regular_voxel_storage* storage, regular_voxel_storage* seed) {
		init_(storage);
		went_out_of_bounds = false;
		
		queue = { };

		for (const auto& pos : *seed) {
			search_value_ = get_(pos);
			process_(fn, pos);
		}

		while (!queue.empty()) {
			const vec_n<3, size_t>& current = queue.front();
			process_(fn, current);
			queue.pop_front();
		}
	}
};

class query_leftmost {
public:
	vec_n<3, size_t> operator()(regular_voxel_storage* storage) const {
		size_t i0, i1, j0, j1, k0, k1;
		storage->bounds()[0].tie(i0, j0, k0);
		storage->bounds()[1].tie(i1, j1, k1);

		for (size_t j = j0; j <= j1; ++j) {
			for (size_t k = k0; k <= k1; ++k) {
				auto v = make_vec(i0, j, k);
				if (storage->Get(v)) {
					return v;
				}
			}
		}

		throw std::runtime_error("query yields no result");
	}
};

class keep_outmost : public post_process {
public:
	static const bool UNION_INPUT = false;

	regular_voxel_storage * operator()(regular_voxel_storage* storage) {
		visitor<> v;
		auto seed = query_leftmost()(storage);
		regular_voxel_storage* output = (regular_voxel_storage*)storage->empty_copy();
		size_t processed = 0;
		v([this, output, &processed, &v](const tagged_index& pos) {
			if (pos.which == tagged_index::VOXEL) {
				output->Set(pos.pos);
			} else {
				((abstract_chunked_voxel_storage*)output)->create_constant(pos.pos, 1U);
			}
			processed++;
			if (progress_callback) {
				(*progress_callback)(processed * 100 / (processed + v.queue.size()));
			}
		}, storage, seed);
		if (progress_callback) {
			(*progress_callback)(100);
		}
		return output;
	}
};

namespace {
	template <typename Fn>
	void connected_components(regular_voxel_storage* storage, Fn fn) {
		auto storage_copy = (regular_voxel_storage*) storage->copy();
		while (storage_copy->count() > 0) {
			auto seed = query_leftmost()(storage_copy);
			
			regular_voxel_storage* output = (regular_voxel_storage*)storage_copy->empty_copy();
			
			visitor<> v;
			v([output](const tagged_index& pos) {
				if (pos.which == tagged_index::VOXEL) {
					output->Set(pos.pos);
				} else {
					((abstract_chunked_voxel_storage*)output)->create_constant(pos.pos, 1U);
				}
			}, storage_copy, seed);

			fn(output);

			storage_copy->boolean_subtraction_inplace(output);

			// NB: deleted
			delete output;
		}
	}
}

class traversal_voxel_filler : public post_process {
public:
	static const bool UNION_INPUT = false;
	
	regular_voxel_storage* output;
	bool start_outside;

	traversal_voxel_filler() : output(nullptr), start_outside(false) {}

	regular_voxel_storage * operator()(regular_voxel_storage* storage) {
		// NB: this only works for one single connected component

		visitor<> v;
		auto seed = start_outside
			// Assumes there is enough padding
			? (storage->bounds()[0] - make_vec(size_t(1), size_t(1), size_t(1)))
			: (storage->bounds()[0] + storage->bounds()[1]) / 2U;

		if ((seed >= storage->extents()).any()) {
			throw std::runtime_error("Not enough padding for outside fill");
		}

		// Make sure seed is 0
		while (storage->Get(seed)) {
			seed.get<0>() += 1;
		}

		if (output == nullptr) {
			output = (regular_voxel_storage*)storage->empty_copy();
		}

		size_t processed = 0;
		v([this, &processed, &v](const tagged_index& pos) {
			if (pos.which == tagged_index::VOXEL) {
				this->output->Set(pos.pos);
				processed++;
			} else {
				((abstract_chunked_voxel_storage*)output)->create_constant(pos.pos, 1U);
				const auto cs = ((abstract_chunked_voxel_storage*)output)->chunk_size();
				processed += cs * cs * cs;
			}
			
			if (progress_callback) {
				(*progress_callback)(processed * 100 / (processed + v.queue.size()));
			}
		}, storage, seed);

		if (start_outside && !v.went_out_of_bounds) {
			throw std::runtime_error("Failed to select seed out of voxel volume");
		}

		if (v.went_out_of_bounds) {
			// Invert if we detect we detect left storage bounds

			regular_voxel_storage* inverted = (regular_voxel_storage*) output->inverted();
			
			// Subtract original to detect if we have an empty volume
			inverted->boolean_subtraction_inplace(storage);

			/*
			// Empty result should not be an issue
			if (inverted->count() == 0) {
				throw std::runtime_error("Empty volume");
			}*/

			delete output;
			output = inverted;
		}
		
		if (progress_callback) {
			(*progress_callback)(100);
		}

		return output;
	}
};

class traversal_voxel_filler_separate_components : public post_process {
public:
	static const bool UNION_INPUT = false;

	regular_voxel_storage * operator()(regular_voxel_storage* storage) {
		traversal_voxel_filler filler;
		// Same output is re-used to remove the need for boolean union
		filler.output = (regular_voxel_storage*) storage->empty_copy();

		connected_components(storage, [&filler](regular_voxel_storage* c) {
			filler(c);
		});

		return filler.output;
	}
};

class traversal_voxel_filler_inverse : public post_process {
public:
	static const bool UNION_INPUT = false;

	regular_voxel_storage * operator()(regular_voxel_storage* storage) {
		traversal_voxel_filler filler;
		filler.start_outside = true;
		filler.progress_callback = boost::none;
		return filler(storage);
	}
};

#endif