#include "../storage.h"

#include <gtest/gtest.h>

// Disabled because files are always re-created
TEST(DISABLED_Storage, MemoryMapped) {
	std::remove("test.vox");
	{
		memory_mapped_chunked_voxel_storage storage(0., 0., 0., 0.1, 1024, 1024, 1024, 128, "test.vox");
		storage.Set({1U, 1U, 1U});
	}
	{
		memory_mapped_chunked_voxel_storage storage(0., 0., 0., 0.1, 1024, 1024, 1024, 128, "test.vox");
		ASSERT_FALSE(storage.Get({ 0U, 0U, 0U }));
		ASSERT_TRUE(storage.Get({ 1U, 1U, 1U }));
		storage.Set({2U, 2U, 2U});
		storage.Set({1000U, 1000U, 1000U});
	}
	{
		memory_mapped_chunked_voxel_storage storage(0., 0., 0., 0.1, 1024, 1024, 1024, 128, "test.vox");
		ASSERT_FALSE(storage.Get({ 0U, 0U, 0U }));
		ASSERT_TRUE(storage.Get({ 1U, 1U, 1U }));
		ASSERT_TRUE(storage.Get({ 2U, 2U, 2U }));
		ASSERT_TRUE(storage.Get({ 1000U, 1000U, 1000U }));
	}
}
