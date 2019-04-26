#include "factory.h"

#include "storage.h"

boost::optional<std::string> factory::mmap_;
size_t factory::post_fix_;

abstract_voxel_storage* factory::create(double x1, double y1, double z1, double d, int nx, int ny, int nz) {
	if (chunk_size_ && *chunk_size_) {
		if (mmap_) {
			return new memory_mapped_chunked_voxel_storage(x1, y1, z1, d, nx, ny, nz, *chunk_size_, mmap_filename());
		} else {
			return new chunked_voxel_storage<bit_t>(x1, y1, z1, d, nx, ny, nz, *chunk_size_);
		}
	} else {
		return new continuous_voxel_storage<bit_t>(x1, y1, z1, d, nx, ny, nz);
	}
}