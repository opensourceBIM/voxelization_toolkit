#include "../voxelizer.h"
#include "../writer.h"

#include <gtest/gtest.h>

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>

TEST(Voxelizer, ChunkPrimitives) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 200, 150, 10, 32);
	
	{
		BRepBuilderAPI_MakePolygon mp(gp_Pnt(1, 1, 0), gp_Pnt(16, 1, 0), gp_Pnt(16, 9.8, 0), gp_Pnt(1, 9.8, 0), true);
		BRepBuilderAPI_MakeFace mf(mp.Wire());
		TopoDS_Face face = mf.Face();

		auto vox = voxelizer(face, storage);
		vox.Convert();
	}

	voxel_writer writer;
	writer.SetVoxels(storage);
	writer.Write("test_voxelizer.vox");

	return;

	EXPECT_TRUE((storage->num_chunks() == vec_n<3>(4U, 4U, 1U)).all());
	EXPECT_TRUE(storage->get_chunk({0U, 0U, 0U})->is_explicit());
	std::array< vec_n<3, size_t>, 2 > out;
	storage->chunk_extents({ 1U,1U,0U }, out);
	std::cerr << out[0].format() << " - " << out[1].format() << std::endl;
	EXPECT_FALSE(storage->get_chunk({1U, 1U, 0U})->is_explicit());

	{
		BRepBuilderAPI_MakePolygon mp(gp_Pnt(1, 1, 0.5), gp_Pnt(9.8, 1, 0.5), gp_Pnt(9.8, 9.8, 0.5), gp_Pnt(1, 9.8, 0.5), true);
		BRepBuilderAPI_MakeFace mf(mp.Wire());
		TopoDS_Face face = mf.Face();

		auto vox = voxelizer(face, storage);
		vox.Convert();
	}

	EXPECT_TRUE((storage->num_chunks() == vec_n<3>(4U, 4U, 1U)).all());
	EXPECT_TRUE(storage->get_chunk({0U, 0U, 0U})->is_explicit());
	EXPECT_FALSE(storage->get_chunk({1U, 1U, 0U})->is_explicit());
	EXPECT_FALSE(storage->get_chunk({2U, 1U, 0U})->is_explicit());

	{
		BRepBuilderAPI_MakePolygon mp(gp_Pnt(3.2, 3.2, 1.0), gp_Pnt(6.39, 3.2, 1.0), gp_Pnt(6.39, 6.39, 1.0), gp_Pnt(3.2, 6.39, 1.0), true);
		BRepBuilderAPI_MakeFace mf(mp.Wire());
		TopoDS_Face face = mf.Face();

		auto vox = voxelizer(face, storage);
		vox.Convert();
	}

	EXPECT_TRUE((storage->num_chunks() == vec_n<3>(4U, 4U, 1U)).all());
	EXPECT_TRUE(storage->get_chunk({0U, 0U, 0U})->is_explicit());
	EXPECT_FALSE(storage->get_chunk({1U, 1U, 0U})->is_explicit());

	std::ostringstream ss;
	storage->get_chunk({1U, 1U, 0U})->write(file_part_primitives, ss);
	EXPECT_EQ(ss.str(), "Z=0,Z=5,Z=10");

	ss.str("");
	EXPECT_FALSE(storage->get_chunk({2U, 1U, 0U})->is_explicit());
	storage->get_chunk({2U, 1U, 0U})->write(file_part_primitives, ss);
	EXPECT_EQ(ss.str(), "Z=0,Z=5");

	voxel_writer writer2;
	writer2.SetVoxels(storage);
	writer2.Write("test_voxelizer.vox");
}