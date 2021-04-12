#ifndef WRITER_H
#define WRITER_H

#include "storage.h"
#include "H5Cpp.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <string>
#include <fstream>


class abstract_writer {

public:
	abstract_voxel_storage* voxels_;
	void SetVoxels(abstract_voxel_storage* voxels) {
		voxels_ = voxels;
	}

	virtual void Write(const std::string& fnc) = 0;


};

class voxel_writer :public abstract_writer {
private:

	std::ofstream& assert_good_(const std::string& fn, std::ofstream& fs) {
		if (!fs.good()) {
			throw std::runtime_error("Unable to open file for writing. " + fn);
		}
		return fs;
	}
public:
	void Write(const std::string& fnc) {
		{
			std::string fn = fnc + std::string(".index");
			std::ofstream fs(fn.c_str());
			voxels_->write(file_part_index, assert_good_(fn, fs));
		}
		{
			std::string fn = fnc + std::string(".contents");
			std::ofstream fs(fn.c_str(), std::ios::binary);
			voxels_->write(file_part_contents, assert_good_(fn, fs));
		}
		{
			std::string fn = fnc + std::string(".meta");
			std::ofstream fs(fn.c_str());
			voxels_->write(file_part_meta, assert_good_(fn, fs));
		}
		{
			std::string fn = fnc + std::string(".primitives");
			std::ofstream fs(fn.c_str());
			voxels_->write(file_part_primitives, assert_good_(fn, fs));
		}
	}
};



class hdf_writer :public abstract_writer {

public:
	void Write(const std::string& fnc) {
		const int      RANK = 2;
		const H5std_string FILE_NAME(fnc);
		const H5std_string DATASET_NAME("Continuous");

		H5::H5File file(FILE_NAME, H5F_ACC_TRUNC);

		hsize_t      dims[2] = { 3, 3 };  // dataset dimensions at creation
		hsize_t      maxdims[2] = { H5S_UNLIMITED, H5S_UNLIMITED };
		H5::DataSpace mspace1(RANK, dims, maxdims);

		H5::DSetCreatPropList cparms;
		hsize_t      chunk_dims[2] = { 2, 5 };
		cparms.setChunk(RANK, chunk_dims);

		H5::DataSet dataset = file.createDataSet(DATASET_NAME, H5::PredType::NATIVE_INT, mspace1, cparms);

		chunked_voxel_storage<bit_t>* storage = (chunked_voxel_storage<bit_t>*)voxels_;

		for (int i = 0; i < storage->num_chunks().get(0); i++) {
			auto c = storage->get_chunk(make_vec<size_t>(i, 0, 0));

			if (c && c->is_explicit()) {
				continuous_voxel_storage<bit_t>* convox = (continuous_voxel_storage<bit_t>*)c;
				auto d = convox->data();

				for (int j = 0; j < convox->size(); j++) {
					auto vd = convox->data()[j];
					std::bitset<8> b(convox->data()[j]);

					/*std::cout << b<<std::endl;*/

				/*	for (int k = 0; k < 8; k++) {
						std::cout << b[k];

					}*/

					std::cout << (int)convox->data()[j] << std::endl;
					for (int k = 0; k < 8; k++) {
						int bitmask = 1 << k;
						int masked = vd & bitmask;
						int voxbit = masked >> vd;
						std::cout << voxbit;
					}

				}

				if (c && c->is_constant()) {
					std::cout << "Constant handling to implement." << std::endl;

				}
				else {
					std::cout << "Plane handling to implement." << std::endl;

				}
			}
		}
	std:cout << std::endl;
	}


};




#endif
