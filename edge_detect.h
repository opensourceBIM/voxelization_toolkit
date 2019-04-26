#ifndef EDGE_DETECT_H
#define EDGE_DETECT_H

#include "storage.h"
#include "progress.h"

#include <boost/optional.hpp>

#include <thread>

class post_process {
public:
	virtual regular_voxel_storage* operator()(regular_voxel_storage*) = 0;
	boost::optional < std::function<void(int)> > progress_callback;

	virtual ~post_process() {}
};

template <typename T>
class threaded_post_process : public post_process {
private:
	size_t n_;
public:
	threaded_post_process(size_t n)
		: n_(n > 0 ? n : std::thread::hardware_concurrency())
	{}

	regular_voxel_storage* operator()(regular_voxel_storage* storage) {
		progress_writer p(typeid(T).name());
		
		std::vector<std::thread> ts;
		std::vector<regular_voxel_storage*> results(n_);
		ts.reserve(n_);
		auto progress = p.thread(n_);
		
		size_t x0 = 0;

		// std::cerr << std::endl << "full " << storage->bounds()[0].format() << " - " << storage->bounds()[1].format() << std::endl;
		// std::cerr << "    count " << storage->count() << std::endl;

		if (n_ == 1) {
			T t;
			t.progress_callback = std::function<void(int)>([&progress](int p) {
				progress(0, p);
			});
			results[0] = t(storage);
		} else {
			// Create regions
			std::vector<voxel_region*> regions;
			regions.reserve(n_);
			for (size_t i = 0; i < n_; ++i) {
				regions.push_back(voxel_region::make(storage, i, n_));
				// std::cerr << "region " << i << " " << regions.back()->bounds()[0].format() << " - " << regions.back()->bounds()[1].format() << std::endl;
				// std::cerr << "         count " << regions.back()->count() << std::endl;
			}

			auto is_chunked = dynamic_cast<abstract_chunked_voxel_storage*>(storage);
			if (is_chunked) {
				is_chunked->lock_bounds();
			}

			// Launch threads
			for (size_t i = 0; i < n_; ++i) {
				ts.emplace_back(std::thread([this, i, &regions, &results, &progress]() {
					T t;
					t.progress_callback = std::function<void(int)>([&progress, i](int p) {
						progress(i, p);
					});
					// @todo: on nix we sometimes get an empty result back. Is this due to the mutable bounds_ in abstract abstract_voxel_storage?
					results[i] = t(regions[i]);
				}));
			}

			// Wait for completion
			for (auto jt = ts.begin(); jt != ts.end(); ++jt) {
				jt->join();
			}

			if (is_chunked) {
				is_chunked->unlock_bounds();
			}
		}

		// std::cerr << std::endl;

		regular_voxel_storage* first = 0;
		for (auto jt = results.begin(); jt != results.end(); ++jt) {
			if (*jt) {
				if (first == 0) {
					first = *jt;
				} else {
					// std::cerr << " + " << (*jt)->bounds()[0].format() << " - " << (*jt)->bounds()[1].format() << std::endl;
					first->boolean_union_inplace(*jt);
					delete *jt;
				}
				if (n_ > 1) {
					// std::cerr << "boolean union " << std::distance(results.begin(), jt) << " " << first->bounds()[0].format() << " - " << first->bounds()[1].format() << std::endl;
				}
			}
		}

		if (T::UNION_INPUT) {
			first->boolean_union_inplace(storage);
			// std::cerr << "boolean union final " << first->bounds()[0].format() << " - " << first->bounds()[1].format() << std::endl;
		}

		return first;
	}
};

class edge_detect : public post_process {
private:
	regular_voxel_storage* storage_;
	regular_voxel_storage* output_;
	bool* empty_chunks_;
	size_t chunk_size_;
	vec_n<3, size_t> num_chunks_;
	std::array<vec_n<3, size_t>, 2> extents_;

	bool get_(const vec_n<3, size_t>& pos) {
		if ((pos < extents_[0]).any()) {
			return false;
		}

		if ((pos > extents_[1]).any()) {
			return false;
		}
		
		if (chunk_size_) {
			if (empty_chunk(pos / chunk_size_)) {
				return false;
			}
		}

		return storage_->Get(pos);
	}

	void init_(regular_voxel_storage* storage) {
		storage_ = storage;
		output_ = (regular_voxel_storage*)storage_->empty_copy();
		extents_ = storage->bounds();

		chunked_voxel_storage<bit_t>* cvs;
		
		if ((cvs = dynamic_cast<chunked_voxel_storage<bit_t>*>(storage))) {
			chunk_size_ = cvs->chunk_size();
			num_chunks_ = storage_->extents().ceil_div(chunk_size_);
			size_t n = num_chunks_.get<0>() * num_chunks_.get<1>() * num_chunks_.get<2>();
			empty_chunks_ = new bool[n] {false};

			auto cs = cvs->empty_chunks();
			for (auto& c : cs) {
				empty_chunk(c) = true;
			}
		}
	}

	bool& empty_chunk(const vec_n<3, size_t>& c) {
		return empty_chunks_[c.get(0) + (num_chunks_.get(0) * c.get(1)) + (num_chunks_.get(0) * num_chunks_.get(1) * c.get(2))];
	}

public:
	static const bool UNION_INPUT = false;

	edge_detect()
		: empty_chunks_(nullptr)
		, chunk_size_(0) {}

	~edge_detect() {
		delete[] empty_chunks_;
	}

	regular_voxel_storage* operator()(regular_voxel_storage* storage) {
		init_(storage);

		size_t i0, i1, j0, j1, k0, k1;
		extents_[0].tie(i0, j0, k0);
		extents_[1].tie(i1, j1, k1);

		vec_n<3, size_t> ijk;
		for (long i = i0; i <= (long) i1; ++i) {
			ijk.get(0) = i;
			for (long j = j0; j <= (long)j1; ++j) {
				ijk.get(1) = j;
				bool inside = false;
				for (long k = k0; k <= (long) k1; ++k) {
					ijk.get(2) = k;
					

					bool some_set = false, some_unset = false;

					const vec_n<3> ijk = vec_n<3, long>(i, j, k).as<size_t>();

					if (!get_(ijk)) {
						continue;
					}

					for (int di = -1; di <= 1; ++di) {
						for (int dj = -1; dj <= 1; ++dj) {
							for (int dk = -1; dk <= 1; ++dk) {
								const vec_n<3> ijk0 = vec_n<3, long>(i + di, j + dj, k + dk).as<size_t>();
								if ((ijk0 == ijk).all()) {
									continue;
								}
								if (get_(ijk0)) {
									some_set = true;
									if (some_unset) {
										goto break_all;
									}
								} else {
									some_unset = true;
									if (some_set) {
										goto break_all;
									}
								}
							}
						}
					}

				break_all:
					if (some_set && some_unset) {
						output_->Set(ijk);
					}
				}
			}
		}

		return output_;
	}
};

#endif