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

		for (int i = 0; i < storage->num_chunks().get(0); i++) {
			auto chunk = storage->get_chunk(make_vec<size_t>(i, 0, 0));

			if (chunk && chunk->is_explicit()) {
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

		//std::cout << continuous_count << std::endl;
		//std::cout << planar_count << std::endl;
		//std::cout << constant_count << std::endl; 
		
		int chunk_stream = storage->chunk_size() * storage->voxel_size(); 

		const H5std_string FILE_NAME(fnc);
		const H5std_string DATASET_NAME("continuous_chunks");
		const int      NX = 10;
		const int      NY = 5;
		const int      RANK = 2;
		hsize_t      dims[2] = { 3, 3 };  // dataset dimensions at creation
		hsize_t      maxdims[2] = { H5S_UNLIMITED, H5S_UNLIMITED };
		H5::DataSpace mspace1(RANK, dims, maxdims);
	
		H5::H5File file(FILE_NAME, H5F_ACC_TRUNC);

		H5::DSetCreatPropList cparms;
		hsize_t      chunk_dims[2] = { 2, 5 };
		cparms.setChunk(RANK, chunk_dims);
	
		int fill_val = 0;
		cparms.setFillValue(H5::PredType::NATIVE_INT, &fill_val);
	
		H5::DataSet dataset = file.createDataSet(DATASET_NAME, H5::PredType::NATIVE_INT, mspace1, cparms);
	
		hsize_t      size[2];
		size[0] = continuous_count;
		size[1] = 32 * 32 * 32;
		dataset.extend(size);
	
		H5::DataSpace fspace1 = dataset.getSpace();
		hsize_t     offset[2];
		offset[0] = 1;
		offset[1] = 0;
		hsize_t      dims1[2] = { 3, 3 };            /* data1 dimensions */
		fspace1.selectHyperslab(H5S_SELECT_SET, dims1, offset);

		H5::DataSpace fspace2 = dataset.getSpace();
		offset[0] = 1;
		offset[1] = 0;
		hsize_t   dims2[2] = { 1, 32*32*32 };

		fspace2.selectHyperslab(H5S_SELECT_SET, dims2, offset);

		H5::DataSpace mspace2(RANK, dims2);

		int cont_count = 0;

		for (int i = 0; i < storage->num_chunks().get(0); i++) {

			std::vector<int> bits_container;
			auto c = storage->get_chunk(make_vec<size_t>(i, 0, 0));

			if (c && c->is_explicit()) {

				offset[0] = cont_count;
				cont_count++;
				offset[1] = 0;
				fspace2.selectHyperslab(H5S_SELECT_SET, dims2, offset);

				continuous_voxel_storage<bit_t>* convox = (continuous_voxel_storage<bit_t>*)c;
				auto d = convox->data();
				//std::cout << convox->size();

				for (int j = 0; j < convox->size(); j++) {
					auto vd = convox->data()[j];
					// consider using boost::dynamic_bitset
					std::bitset<8> b(convox->data()[j]);
			
					for (int k = 0; k < 8; k++) {
						bits_container.push_back(b[k]);
					
					}
				}


				dataset.write(bits_container.data(), H5::PredType::NATIVE_INT, mspace2, fspace2);
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
