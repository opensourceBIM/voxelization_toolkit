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

		chunked_voxel_storage<bit_t>* storage = (chunked_voxel_storage<bit_t>*)voxels_;

		int continuous_count = 0;
		int planar_count = 0;
		int constant_count = 0;

		int col_n = 0;

		for (int i = 0; i < storage->num_chunks().get(0); i++) {
			auto chunk = storage->get_chunk(make_vec<size_t>(i, 0, 0));
			
			if (chunk && chunk->is_explicit()) {
				if (col_n == 0) {
					continuous_voxel_storage<bit_t>* casted = (continuous_voxel_storage<bit_t>*)chunk;
					col_n = casted->size() * casted->value_bits() * 8;
				}
			
				continuous_count++;

			}
			else {
				if (chunk && chunk->is_constant()) {
					constant_count++;
				}

				else {
					planar_count++;
				}
			}
		}


		int chunk_stream = storage->chunk_size() * storage->voxel_size();

		std::vector<int> bits_container;

		const H5std_string FILE_NAME(fnc);
		const H5std_string DATASET_NAME("continuous_chunks");
		const int      NX = continuous_count;
		const int      NY = col_n; 
		const int      RANK = 2;
		H5::H5File file(FILE_NAME, H5F_ACC_TRUNC);

		hsize_t     dimsf[2];
		dimsf[0] = NX;
		dimsf[1] = NY;
		H5::DataSpace dataspace(RANK, dimsf);
		H5::IntType datatype(H5::PredType::NATIVE_INT);
		datatype.setOrder(H5T_ORDER_LE);
		H5::DataSet dataset = file.createDataSet(DATASET_NAME, datatype, dataspace);

		int column_number = col_n;
		hsize_t     offset[2];
		offset[0] = 0;
		offset[1] = 0;

		hsize_t     slab_dimsf[2] = { 1, col_n  };
		dataspace.selectHyperslab(H5S_SELECT_SET, slab_dimsf, offset);
		H5::DataSpace mspace2(RANK, slab_dimsf);


		int cont_count = 0;

		for (int i = 0; i < storage->num_chunks().get(0); i++) {

			std::vector<int> bits_container;
			auto c = storage->get_chunk(make_vec<size_t>(i, 0, 0));

			if (c && c->is_explicit()) {

				offset[0] = cont_count;
				cont_count++;
				offset[1] = 0;
				dataspace.selectHyperslab(H5S_SELECT_SET, slab_dimsf, offset);

				continuous_voxel_storage<bit_t>* convox = (continuous_voxel_storage<bit_t>*)c;
				auto d = convox->data();
			
				for (int j = 0; j < convox->size(); j++) {
					auto vd = convox->data()[j];
					// consider using boost::dynamic_bitset
					std::bitset<8> b(convox->data()[j]);
			
					for (int k = 0; k < 8; k++) {
						bits_container.push_back(b[k]);
					
					}
				}

				dataset.write(bits_container.data(), H5::PredType::NATIVE_INT, mspace2, dataspace);
			}

			if (c && c->is_constant()) {
				std::cout << "Constant handling to implement." << std::endl;

			}
			else {
				std::cout << "Plane handling to implement." << std::endl;

			}


			}
		}


	};
	





#endif
