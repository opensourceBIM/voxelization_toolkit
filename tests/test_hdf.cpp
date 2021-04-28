#include "../voxelizer.h"
#include "../writer.h"
#include "H5Cpp.h"
#include <gtest/gtest.h>

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepMesh_IncrementalMesh.hxx>



TEST(HdfFileName, HDF) {

	auto storage2 = new chunked_voxel_storage<bit_t>(0., 0., 0., 1, 32, 32, 32, 32);

	BRepPrimAPI_MakeBox mb(gp_Pnt(1, 1, 1), gp_Pnt(10, 10, 10));
	auto x = mb.Solid();
	BRepMesh_IncrementalMesh(x, 0.001);
	auto vox = voxelizer(x, storage2);
	vox.Convert();


	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 200, 150, 10, 32);

	{
		BRepBuilderAPI_MakePolygon mp(gp_Pnt(1, 1, 0), gp_Pnt(16, 1, 0), gp_Pnt(16, 9.8, 0), gp_Pnt(1, 9.8, 0), true);
		BRepBuilderAPI_MakeFace mf(mp.Wire());
		TopoDS_Face face = mf.Face();

		auto vox = voxelizer(face, storage);
		vox.Convert();
	}

	hdf_writer writer; 
	writer.SetVoxels(storage2);
	writer.Write("voxels2.h5");

	std::ofstream fs("voxobj.obj");
	obj_export_helper oeh{ fs };
	storage2->obj_export(oeh, false, false);


}
