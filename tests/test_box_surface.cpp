#include "../voxelizer.h"
#include "../volume.h"
#include "../writer.h"

#include <gtest/gtest.h>

#include <BRepPrimAPI_MakeBox.hxx>

TEST(Voxelizer, Volume) {
	const double d = 0.1;
	auto storage = new continuous_voxel_storage<bit_t>(-d, -d, -d, d, 100, 100, 100);

	BRepPrimAPI_MakeBox mb(8., 8., 8.);
	auto vox = voxelizer(mb.Solid(), storage);
	vox.Convert();
	
	voxel_writer writer;
	writer.SetVoxels(storage);
	writer.Write("test_box_surface.vox");

	ASSERT_FALSE(storage->Get(make_vec<size_t>(0U, 0U, 1U)));
	
	ASSERT_TRUE(storage->Get(make_vec<size_t>(1U, 1U, 1U)));
	ASSERT_TRUE(storage->Get(make_vec<size_t>(80U, 1U, 1U)));
	ASSERT_TRUE(storage->Get(make_vec<size_t>(1U, 80U, 1U)));
	ASSERT_TRUE(storage->Get(make_vec<size_t>(80U, 80U, 1U)));
	ASSERT_TRUE(storage->Get(make_vec<size_t>(80U, 80U, 2U)));

	ASSERT_FALSE(storage->Get(make_vec<size_t>(81U, 80U, 1U)));
	ASSERT_FALSE(storage->Get(make_vec<size_t>(80U, 81U, 1U)));
	ASSERT_FALSE(storage->Get(make_vec<size_t>(81U, 81U, 1U)));
	ASSERT_FALSE(storage->Get(make_vec<size_t>(81U, 80U, 2U)));
	ASSERT_FALSE(storage->Get(make_vec<size_t>(80U, 81U, 2U)));
	ASSERT_FALSE(storage->Get(make_vec<size_t>(81U, 81U, 2U)));
}
