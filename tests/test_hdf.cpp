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

	hdf_writer writer; 
	writer.SetVoxels(storage2);
	writer.Write("voxels.h5");

	std::ofstream fs("voxobj.obj");
	obj_export_helper oeh{ fs };
	storage2->obj_export(oeh, false, false);


}
