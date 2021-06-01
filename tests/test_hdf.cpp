#include "../voxelizer.h"
#include "../writer.h"
#include "../processor.h"

#include "H5Cpp.h"
#include <gtest/gtest.h>

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>

#ifdef WITH_IFC
#include <ifcparse/IfcFile.h>
#ifdef IFCOPENSHELL_05
#include <ifcgeom/IfcGeomIterator.h>
using namespace Ifc2x3;
#else
#include <ifcgeom_schema_agnostic/IfcGeomIterator.h>
#endif
#endif


#ifdef WIN32
#define DIRSEP "\\"
#else
#define DIRSEP "/"
#endif


TEST(HdfFileName, HDF) {


	BRepPrimAPI_MakeBox mb(gp_Pnt(0, 0, 0), gp_Pnt(10, 10, 10));
	BRepPrimAPI_MakeSphere s(gp_Pnt(10, 10, 10), 7);

	auto x = s.Solid();

	auto storage2 = new chunked_voxel_storage<bit_t>( 0, 0, 0, 1, 32, 32, 32, 32);
	BRepTools breptools; 

	breptools.Write(x, "sphere.brep");

	BRepMesh_IncrementalMesh(x, 0.001);
	auto vox = voxelizer(x, storage2);
	vox.Convert();

	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 200, 150, 10, 32);

	{
		BRepBuilderAPI_MakePolygon mp(gp_Pnt(1, 1, 2), gp_Pnt(16, 1, 2), gp_Pnt(16, 9.8, 2), gp_Pnt(1, 9.8, 2), true);
		BRepBuilderAPI_MakeFace mf(mp.Wire());
		TopoDS_Face face = mf.Face();

		auto vox = voxelizer(face, storage);
		vox.Convert();
	}

	const double dim = 0.1;
	auto storage3 = new chunked_voxel_storage<bit_t>(-dim, -dim, -dim, dim, 100, 100, 100, 16);	
	BRepPrimAPI_MakeBox make_box(8., 8., 8.);
	auto vox3 = voxelizer(make_box.Solid(), storage3);
	vox3.Convert();

	hdf_writer writer;
	writer.SetVoxels(storage3);
	writer.Write("multi_dim_vox.h5");

	//std::ofstream fs("voxobj.obj");
	//obj_export_helper oeh{ fs };
	//storage2->obj_export(oeh, false, false);

	// Write a 4D dataset

	const H5std_string  FILE_NAME("multidim.h5");
	const H5std_string  DATASET_NAME("continuous_chunks");
	const int   NX = 32;                    // dataset dimensions
	const int   NY = 32;
	const int   NZ = 32;
	const int   NC = 3;

	const int   RANK = 4;
	H5::H5File file(FILE_NAME, H5F_ACC_TRUNC);

	hsize_t     dimsf[4];              // dataset dimensions
	dimsf[0] = NC;
	dimsf[1] = NX;
	dimsf[2] = NY;
	dimsf[3] = NZ;

	H5::DataSpace dataspace(RANK, dimsf);

	H5::IntType datatype(H5::PredType::NATIVE_INT);
	datatype.setOrder(H5T_ORDER_LE);

	H5::DataSet dataset = file.createDataSet(DATASET_NAME, datatype, dataspace);

	//std::vector<int> data; 

	//for (int i = 0; i < NX*NY*NZ*NC; i++) {
	//	data.push_back(0);
	//}

	//dataset.write(data.data(), H5::PredType::NATIVE_INT);

}
