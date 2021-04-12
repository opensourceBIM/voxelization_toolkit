#ifndef WRITER_H
#define WRITER_H

#include "storage.h"
#include "H5Cpp.h"

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

		for (int i = 0; i < storage->num_chunks().get(0); i++) {
			auto c = storage->get_chunk(make_vec<size_t>(i, 1, 0));			

			if (c && c->is_explicit()) {
				continuous_voxel_storage<bit_t>* convox = (continuous_voxel_storage<bit_t>*)c;
				auto d = convox->data();
				//std::cout << c->value_bits(); 
				for (int j = 0; j < convox->size(); j++) {
					auto vd = convox->data()[j];
					std::bitset<8> b(convox->data()[j]); 
					std::cout << (int)convox->data()[j]<<std::endl;
				}

				if (c && c->is_constant() ){
					std::cout << "Constant handling to implement."<<std::endl; 
			
				}
				else {
					std::cout << "Plane handling to implement."<<std::endl;

				}
			}

		}
	}


};




#endif
