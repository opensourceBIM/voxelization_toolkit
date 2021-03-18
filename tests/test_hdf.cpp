#include "../voxelizer.h"
#include "../writer.h"
#include "H5Cpp.h"
#include <gtest/gtest.h>

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>

TEST(HdfFileName, HDF) {

	const H5std_string  FILE_NAME("SDS.h5");
	const H5std_string  DATASET_NAME("IntArray");
	const int   NX = 5;                    
	const int   NY = 6;
	const int   RANK = 2;

	int i, j;
	int data[NX][NY];          
	for (j = 0; j < NX; j++)
	{
		for (i = 0; i < NY; i++)
			data[j][i] = i + j;
	}

	H5::H5File file(FILE_NAME, H5F_ACC_TRUNC);
	hsize_t     dimsf[2];              
	dimsf[0] = NX;
	dimsf[1] = NY;
	H5::DataSpace dataspace(RANK, dimsf);
	H5::IntType datatype(H5::PredType::NATIVE_INT);
	datatype.setOrder(H5T_ORDER_LE);

	H5::DataSet dataset = file.createDataSet(DATASET_NAME, datatype, dataspace);
	dataset.write(data, H5::PredType::NATIVE_INT);

	std::string file_name = file.getFileName();
	EXPECT_TRUE(file_name == FILE_NAME);

}
