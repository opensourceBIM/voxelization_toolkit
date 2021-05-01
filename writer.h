#ifndef WRITER_H
#define WRITER_H

#include "storage.h"
#include "H5Cpp.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <string>
#include <fstream>


#define BEGIN_LOOP(i0, i1, j0, j1, k0, k1) {\
	vec_n<3, typename std::remove_reference<decltype(i0)>::type> ijk;\
	for (ijk.get(0) = i0; ijk.get(0) < i1; ++ijk.get(0)) {\
		for (ijk.get(1) = j0; ijk.get(1) < j1; ++ijk.get(1)) {\
			for (ijk.get(2) = k0; ijk.get(2) < k1; ++ijk.get(2)) {

#define BEGIN_LOOP_I(i0, i1, j0, j1, k0, k1) {\
	vec_n<3, typename std::remove_reference<decltype(i0)>::type> ijk;\
	for (ijk.get(0) = i0; ijk.get(0) <= i1; ++ijk.get(0)) {\
		for (ijk.get(1) = j0; ijk.get(1) <= j1; ++ijk.get(1)) {\
			for (ijk.get(2) = k0; ijk.get(2) <= k1; ++ijk.get(2)) {

#define BEGIN_LOOP2(v0, v1) BEGIN_LOOP(v0.template get<0>(), v1.template get<0>(), v0.template get<1>(), v1.template get<1>(), v0.template get<2>(), v1.template get<2>())

#define BEGIN_LOOP_I2(v0, v1) BEGIN_LOOP_I(v0.template get<0>(), v1.template get<0>(), v0.template get<1>(), v1.template get<1>(), v0.template get<2>(), v1.template get<2>())

#define BEGIN_LOOP_ZERO_2(v1) BEGIN_LOOP2(make_vec<decltype(v1)::element_type>((decltype(v1)::element_type)0, (decltype(v1)::element_type)0, (decltype(v1)::element_type)0), v1)

#define END_LOOP }}}}


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


		const H5std_string FILE_NAME(fnc);
		const H5std_string DATASET_NAME("continuous_chunks");

		const int      NC = continuous_count;
		const int      NX = storage->chunk_size();
		const int      NY = storage->chunk_size();
		const int	   NZ = storage->chunk_size();

		const int      RANK = 4;
		H5::H5File file(FILE_NAME, H5F_ACC_TRUNC);

		hsize_t     dimsf[4];
		dimsf[0] = NC;
		dimsf[1] = NX;
		dimsf[2] = NY;
		dimsf[3] = NZ;

		H5::DataSpace dataspace(RANK, dimsf);
		H5::IntType datatype(H5::PredType::NATIVE_INT);
		datatype.setOrder(H5T_ORDER_LE);
		H5::DataSet dataset = file.createDataSet(DATASET_NAME, datatype, dataspace);

		int cont_count = 0;

		std::vector<int> bits_container;

		for (int i = 0; i < storage->num_chunks().get(0); i++) {
			for (int j = 0; j < storage->num_chunks().get(1); j++) {
				for (int k = 0; k < storage->num_chunks().get(2); k++) {
					auto c = storage->get_chunk(make_vec<size_t>(i, j, k));
					if (c && c->is_explicit()) {

						continuous_voxel_storage<bit_t>* convox = (continuous_voxel_storage<bit_t>*)c;
						size_t i0, j0, k0, i1, j1, k1;
						i0 = 0;
						j0 = 0;
						k0 = 0;
						convox->extents().tie(i1, j1, k1);

						BEGIN_LOOP_ZERO_2(make_vec<size_t>(i1, j1, k1))
							bits_container.push_back(convox->Get(ijk));
						END_LOOP;
					}

					if (c && c->is_constant()) {
						std::cout << "Constant handling to implement." << std::endl;
					}
					else {
						std::cout << "Plane handling to implement." << std::endl;

					}
				}
			}
		}

		dataset.write(bits_container.data(), H5::PredType::NATIVE_INT);

	}


};






#endif
