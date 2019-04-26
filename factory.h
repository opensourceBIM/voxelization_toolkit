#ifndef FACTORY_H
#define FACTORY_H

#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

#include <thread>
#include <sstream>

class abstract_voxel_storage;

class factory {
private:
	boost::optional<size_t> chunk_size_;
	static boost::optional<std::string> mmap_;
	static size_t post_fix_;

	static std::string thread_id() {
		std::stringstream ss;
		ss << std::this_thread::get_id();
		return ss.str();
	}
public:
	factory& chunk_size() {
		chunk_size_ = boost::none;
		return *this;
	}

	factory& chunk_size(size_t n) {
		chunk_size_ = n;
		return *this;
	}

	static void mmap(const std::string& filename) {
		mmap_ = filename;
		post_fix_ = 0;
	}

	static void post_fix(size_t pf) {
		post_fix_ = pf;
	}

	static std::string mmap_filename() {
		return *mmap_ + "." + thread_id() + "." + std::to_string(post_fix_++);
	}
	
	abstract_voxel_storage* create(double x1, double y1, double z1, double d, int nx, int ny, int nz);
};

#endif
