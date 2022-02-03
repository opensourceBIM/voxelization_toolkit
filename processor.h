#ifndef PROCESSOR_H
#define PROCESSOR_H

#include "traversal.h"
#include "factory.h"
#include "storage.h"
#include "writer.h"
#include "progress_writer.h"
#include "volume.h"
#include "voxelizer.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>

#include <thread>

#ifdef WITH_IFC
#include <map>                    // @todo < commit in IfcopenShell
#include <ifcparse/IfcLogger.h>   // @todo < commit in IfcopenShell
#include <ifcgeom/IfcGeomElement.h>

#ifdef IFCOPENSHELL_07
typedef IfcGeom::BRepElement elem_t;
#else
typedef IfcGeom::BRepElement<double> elem_t;
#endif

#endif

typedef TopoDS_Shape geometry_t;
typedef std::vector<std::pair<int, TopoDS_Compound > > geometry_collection_t;

class voxelization_mode {
public:
	int tag;
	voxelization_mode(int t) : tag(t) {}
};
class MERGED : public voxelization_mode {
public:
	MERGED() : voxelization_mode(1) {}
};
class SEPARATE : public voxelization_mode {
public:
	SEPARATE() : voxelization_mode(2) {}
};
class SEPARATE_MINUS : public voxelization_mode {
public:
	SEPARATE_MINUS() : voxelization_mode(3) {}
};
enum VOXEL_VALUATION {
	CONSTANT_ONE,
	PRODUCT_ID
};
class fill_volume_t {
public:
	bool y;
	VOXEL_VALUATION value;
	fill_volume_t(bool b, VOXEL_VALUATION v = CONSTANT_ONE) : y(b), value(v) {}
};
class VOLUME : public fill_volume_t {
public:
	VOLUME() : fill_volume_t(true) {}
};
class SURFACE : public fill_volume_t {
public:
	SURFACE() : fill_volume_t(false) {}
};
class VOLUME_PRODUCT_ID : public fill_volume_t {
public:
	VOLUME_PRODUCT_ID() : fill_volume_t(true, PRODUCT_ID) {}
};

class output {
private:
	std::string path_;
	const abstract_voxel_storage* voxels_;
	std::vector<std::function<regular_voxel_storage*(regular_voxel_storage*)>> post_processors_;

public:
	int mode;

	output(const MERGED& mode, const std::string& path = "")
		: mode(mode.tag), path_(path), voxels_(0) {}
	output(const SEPARATE& mode, const std::string& path)
		: mode(mode.tag), path_(path), voxels_(0) {}
	output(const SEPARATE_MINUS& mode, const abstract_voxel_storage* voxels, const std::string& path)
		: mode(mode.tag), path_(path), voxels_(voxels) {}

	output parallelize() const {
		if (mode == 1) {
			return output(MERGED());
		} else if (mode == 2 || mode == 3) {
			return *this;
		} else {
			throw std::runtime_error("invalid mode");
		}
	}

	static void write(const std::string& filename, abstract_voxel_storage* voxels) {
		voxel_writer writer;
		writer.SetVoxels(voxels);
		writer.Write(filename);
	}

	static void interpolate(char* buff, const std::string& path, int i) {
		int r = snprintf(buff, 255, path.c_str(), i);
		if (r < 0 || r >= 255) {
			std::cerr << std::endl << "error allocating buffer" << std::endl;
			abort();
		}
	}

	void intermediate_result(int i, abstract_voxel_storage*& a, abstract_voxel_storage*& b) const {
		if (mode == 1) {
			a->boolean_union_inplace(b);
		} else if (mode == 2 || mode == 3) {
			char buff[255];
			interpolate(buff, path_, i);
			if (mode == 3) {
				b->boolean_subtraction_inplace(voxels_);
			}
			write(buff, b);
		}
	}

	abstract_voxel_storage* final_result(abstract_voxel_storage* a, abstract_voxel_storage* b) const {
		if (mode == 1 && path_.size()) {
			regular_voxel_storage* tmp = (regular_voxel_storage*)a;
			int i = 0;
			for (auto& fn : post_processors_) {
				write(path_ + "-step-" + boost::lexical_cast<std::string>(i++), tmp);
				// std::cerr << "Step " << i << " bounds:" << tmp->bounds()[0].format() << " - " << a->bounds()[1].format() << std::endl;
				regular_voxel_storage* b = fn(tmp);
				if (tmp != a) {
					delete tmp;
				}
				tmp = b;
			}
			// std::cerr << "Final bounds:" << tmp->bounds()[0].format() << " - " << a->bounds()[1].format() << std::endl;
			write(path_, tmp);
			return tmp;
		}
		return nullptr;
	}

	template <typename Fn>
	output& post(Fn p) {
		post_processors_.emplace_back(p);
		return *this;
	}

};

class abstract_processor {
public:
	virtual void process(geometry_collection_t::const_iterator start, geometry_collection_t::const_iterator end, const fill_volume_t& volume, const output& output) = 0;
	virtual ~abstract_processor() {}
	virtual abstract_voxel_storage* voxels() = 0;
};

class processor : public abstract_processor {
private:
	bit_t use_bits;
	factory factory_;
	abstract_voxel_storage* voxels_;
	abstract_voxel_storage* voxels_temp_;
	std::function<void(int)> progress_;
	int nx_, ny_, nz_;
	double x1_, y1_, z1_, d_;
	bool use_copy_, use_scanline_;
public:
	processor(double x1, double y1, double z1, double d, int nx, int ny, int nz, size_t chunk_size, const std::function<void(int)>& progress)
		: factory_(factory().chunk_size(chunk_size))
		, voxels_(factory_.create(x1, y1, z1, d, nx, ny, nz))
		, voxels_temp_(factory_.create(x1, y1, z1, d, nx, ny, nz))
		, progress_(progress)
		, nx_(nx), ny_(ny), nz_(nz)
		, x1_(x1), y1_(y1), z1_(z1), d_(d)
		, use_copy_(false)
		, use_scanline_(true) {}

	processor(abstract_voxel_storage* storage, const std::function<void(int)>& progress)
		: voxels_(storage)
		, voxels_temp_(storage->empty_copy_as(&use_bits))
		, progress_(progress)
		, use_copy_(true)
		, use_scanline_(true) {}

	bool& use_scanline() { return use_scanline_; }
	
	~processor() {
		if (!use_copy_) {
			delete voxels_;
		}
		delete voxels_temp_;
	}

	void process(geometry_collection_t::const_iterator a, geometry_collection_t::const_iterator b, const fill_volume_t& volume, const output& output) {
		int N = std::distance(a, b);
		int n = 0;
		for (geometry_collection_t::const_iterator it = a; it < b; ++it) {
			abstract_voxel_storage* to_write = volume.y ? voxels_temp_ : voxels_;

			if (volume.y) {
				voxelizer voxeliser(it->second, (regular_voxel_storage*) to_write, !use_scanline_, !use_scanline_);
				voxeliser.epsilon() = 1e-9;
				voxeliser.Convert();

				auto filled = traversal_voxel_filler_inverse()((regular_voxel_storage*)to_write);
				// @todo this is not very efficient as the above does exactly the inverse subtraction
				to_write->boolean_union_inplace(filled);
				delete filled;

				if (volume.value == PRODUCT_ID) {
					int n = voxels_->value_bits();
					// @todo this still needs to be generalized
					if (n != 32) {
						throw std::runtime_error("Unable to assign product ids to this voxel value type");
					}
					auto input = (regular_voxel_storage*)to_write;
					uint32_t v = it->first;
					for (auto& ijk : *input) {
						voxels_->Set(ijk, &v);
					}
				} else {
					output.intermediate_result(it->first, voxels_, voxels_temp_);
				}
				
				if (use_copy_) {
					auto tmp = voxels_temp_->empty_copy();
					delete voxels_temp_;
					voxels_temp_ = tmp;
				} else {
					delete voxels_temp_;
					voxels_temp_ = factory_.create(x1_, y1_, z1_, d_, nx_, ny_, nz_);
				}
			} else {
				voxelizer voxeliser(it->second, (regular_voxel_storage*) to_write, !use_scanline_, !use_scanline_);
				voxeliser.epsilon() = 1e-9;
				voxeliser.Convert();
			}

			n++;
			progress_(n * 100 / N);
		}

		auto post_result = output.final_result(voxels_, voxels_temp_);

		if (post_result && post_result != voxels_) {
			delete voxels_;
			voxels_ = post_result;
		}
	}

	abstract_voxel_storage* voxels() { return voxels_; }
};


class threaded_processor : public abstract_processor {
	double x1_, y1_, z1_, d_;
	int nx_, ny_, nz_;
	std::vector<processor*> cs;
	size_t num_threads_;
	progress_writer& p_;
	size_t chunk_size_;
	abstract_voxel_storage* result_;
	abstract_voxel_storage* voxels_;
public:
	threaded_processor(double x1, double y1, double z1, double d, int nx, int ny, int nz, size_t chunk_size, progress_writer& p)
		: x1_(x1), y1_(y1), z1_(z1), d_(d), nx_(nx), ny_(ny), nz_(nz), chunk_size_(chunk_size), num_threads_(std::thread::hardware_concurrency()), p_(p), result_(nullptr), voxels_(nullptr) {}

	threaded_processor(double x1, double y1, double z1, double d, int nx, int ny, int nz, size_t chunk_size, size_t num_threads, progress_writer& p)
		: x1_(x1), y1_(y1), z1_(z1), d_(d), nx_(nx), ny_(ny), nz_(nz), chunk_size_(chunk_size), num_threads_(num_threads > 0 ? num_threads : std::thread::hardware_concurrency()), p_(p), result_(nullptr), voxels_(nullptr) {}

	threaded_processor(abstract_voxel_storage* storage, size_t num_threads, progress_writer& progress)
		: voxels_(storage)
		, num_threads_(num_threads)
		, p_(progress)
		, result_(nullptr)
	{}

	~threaded_processor() {
		for (auto it = cs.begin(); it != cs.end(); ++it) {
			// delete *it;
		}
	}

	void process(geometry_collection_t::const_iterator start, geometry_collection_t::const_iterator end, const fill_volume_t& volume, const output& output) {
		const size_t N = std::distance(start, end);
		const double d = (double)N / num_threads_;

		std::vector<std::thread> ts;
		cs.resize(num_threads_);
		ts.reserve(num_threads_);
		auto progress = p_.thread(num_threads_);
		size_t x0 = 0;
		bool first = true;
		for (size_t i = 0; i < num_threads_; ++i) {
			size_t x1 = (size_t)floor((i + 1) * d);
			if (x1 == x0) continue;
			first = false;
			if (i == num_threads_ - 1) {
				x1 = N;
			}
			if (voxels_) {
				auto local_storage = first ? voxels_ : voxels_->empty_copy();
				ts.emplace_back(std::thread([this, local_storage, x0, x1, i, &start, &volume, &output, &progress]() {
					processor* vf = new processor(local_storage, [i, &progress](int p) {
						progress(i, p);
					});
					cs[i] = vf;
					vf->process(start + x0, start + x1, volume, output.parallelize());
				}));
			} else {
				// @todo refactor this.
				ts.emplace_back(std::thread([this, x0, x1, i, &start, &volume, &output, &progress]() {
					processor* vf = new processor(x1_, y1_, z1_, d_, nx_, ny_, nz_, chunk_size_, [i, &progress](int p) {
						progress(i, p);
					});
					cs[i] = vf;
					vf->process(start + x0, start + x1, volume, output.parallelize());
				}));
			}
			x0 = x1;
		}
		for (auto jt = ts.begin(); jt != ts.end(); ++jt) {
			jt->join();
		}
		if (output.mode == 1) {
			abstract_voxel_storage* first = 0;
			for (auto jt = cs.begin(); jt != cs.end(); ++jt) {
				if (*jt) {
					if (first == 0) {
						first = (**jt).voxels();
					} else {
						first->boolean_union_inplace((**jt).voxels());
						delete *jt;
					}
				}
			}
			if (first) {
				progress.end();

				result_ = first;
				auto post_process_result = output.final_result(first, first);
				if (post_process_result) {
					result_ = post_process_result;
				}
			}
		}
		progress.end();
	}

	abstract_voxel_storage* voxels() {
		return result_;
	}
};


#endif
