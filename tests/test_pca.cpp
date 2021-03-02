#include "../voxelizer.h"
#include "../volume.h"
#include "../traversal.h"


#include <gp_Pln.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

#include <Eigen/Dense>

#include <gtest/gtest.h>

TEST(NormalEstimation, EigenPCA) {
	static Eigen::Vector3f Z(0, 0, 1);
	Eigen::MatrixXf points(10, 3);
	points.setRandom();
	points.col(2).setZero();

	Eigen::MatrixXf centered = points.rowwise() - points.colwise().mean();
	Eigen::MatrixXf cov = centered.adjoint() * centered;
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);
	
	ASSERT_FLOAT_EQ(std::abs(eig.eigenvectors().col(0).dot(Z)), 1.);
}


TEST(NormalEstimation, OCC) {
	gp_Pln p(1, 2, 3, 4);
	auto face = BRepBuilderAPI_MakeFace(p, -1, 1, -1, 1).Face();
	BRepMesh_IncrementalMesh(face, 0.1);

	auto surf = BRep_Tool::Surface(face);
	auto dir = Handle_Geom_Plane::DownCast(surf)->Position().Direction();

	Eigen::Vector3f norm(dir.X(), dir.Y(), dir.Z());
	std::cout << "Face normal " << norm << std::endl;
	
	Bnd_Box B;
	BRepBndLib::AddClose(face, B);
	
	double d = 0.01;
	double x0, y0, z0, x1, y1, z1;
	B.Get(x0, y0, z0, x1, y1, z1);
	
	auto storage = new chunked_voxel_storage<bit_t>(
		x0 - d, 
		y0 - d, 
		z0 - d, d, 
		(x1 - x0) / d + 2, 
		(y1 - y0) / d + 2, 
		(z1 - z0) / d + 2, 
		64);
	auto vox = voxelizer(face, storage);
	vox.Convert();	

	std::cout << "count " << storage->count() << std::endl;

	auto it = storage->begin();
	std::advance(it, storage->count() / 2);

	std::cout << "center " << (*it).format() << std::endl;

	int i = 2;

	visitor<26> vis;
	vis.max_depth = 0.1 * i / d;

	std::cout << "md " << (0.1 * i) << std::endl;

	std::vector<float> coords;

	auto selection = storage->empty_copy();

	vis([&coords, &selection](const tagged_index& pos) {
		if (pos.which == tagged_index::VOXEL) {
			coords.push_back(pos.pos.get(0));
			coords.push_back(pos.pos.get(1));
			coords.push_back(pos.pos.get(2));
			selection->Set(pos.pos);
		}
		else {
			throw std::runtime_error("Unexpected");
		}
	}, storage, *it);

	/*{
		std::ofstream ofs("m2.obj");
		((chunked_voxel_storage<bit_t>*)selection)->obj_export(ofs);
	}

	storage->boolean_subtraction_inplace(selection);

	{
		std::ofstream ofs("m1.obj");
		storage->obj_export(ofs);
	}*/

	std::cout << "neighbours " << (coords.size() / 3) << std::endl;

	Eigen::MatrixXf points = Eigen::Map<Eigen::MatrixXf>(coords.data(), 3, coords.size() / 3).transpose();

	Eigen::MatrixXf centered = points.rowwise() - points.colwise().mean();
	Eigen::MatrixXf cov = centered.adjoint() * centered;
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);

	auto estimated = eig.eigenvectors().col(0);

	std::cout << estimated << std::endl;

	auto angle = std::acos(estimated.dot(norm));
	if (angle > M_PI / 2) {
		angle = M_PI - angle;
	}
	auto angle_degrees = angle / M_PI * 180.;

	std::cout << "angle " << angle_degrees << "d" << std::endl;

	ASSERT_LT(angle_degrees, 1.);
}