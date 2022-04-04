#include "../voxelizer.h"
#include "../volume.h"
#include "../writer.h"
#include "../voxec.h"

#include <gtest/gtest.h>

#include <BRepPrimAPI_MakeBox.hxx>

#include <math.h>

TEST(Voxelizer, Volume) {
	const double d = 0.1;
	auto storage = new chunked_voxel_storage<bit_t>(-d, -d, -d, d, 100, 100, 100, 16);

	BRepPrimAPI_MakeBox mb(8., 8., 8.);
	auto vox = voxelizer(mb.Solid(), storage);
	vox.Convert();

	op_normal_estimate normal_estimate;

	scope_map arguments;
	{
		symbol_value v = storage;
		arguments["input"] = v;
	}
	{
		symbol_value v = 5;
		arguments["max_depth"] = v;
	}

	auto r = normal_estimate.invoke(arguments);
	auto result = (regular_voxel_storage*)boost::get<abstract_voxel_storage*>(r);

	normal_and_curvature<int16_t> v;
	for (auto ijk : *result) {

		auto v_float = v.convert<float>();
		std::cout << ijk.format() << ": " << v.nxyz_curv[0] << " " << v.nxyz_curv[1] << " " << v.nxyz_curv[2] << " " << v.nxyz_curv[3] << std::endl;
		std::cout << ijk.format() << ": " << v_float.nxyz_curv[0] << " " << v_float.nxyz_curv[1] << " " << v_float.nxyz_curv[2] << " " << v_float.nxyz_curv[3] << std::endl;
	}

	static const double tol = 1.e-5;
	static const double l = 1. / std::sqrt(3.);
	static const Eigen::Vector3f ref0(l, l, l);
	static const Eigen::Vector3f ref1(1, 0, 0);

	float curv_corner;

	{
		// corner

		result->Get(make_vec<size_t>(95, 95, 95), &v);
		auto v_float = v.convert<float>();

		Eigen::Map<Eigen::Vector3f> v0(v_float.nxyz_curv.data());

		double angle = std::acos(std::abs(ref0.dot(v0)));

		ASSERT_TRUE((std::abs(angle) < tol) || ((std::abs(angle) - M_PI) < tol));

		// store for later use
		curv_corner = v_float.curvature();
	}

	{
		// corner

		result->Get(make_vec<size_t>(16, 16, 16), &v);
		auto v_float = v.convert<float>();

		Eigen::Map<Eigen::Vector3f> v0(v_float.nxyz_curv.data());

		double angle = std::acos(std::abs(ref0.dot(v0)));

		ASSERT_TRUE((std::abs(angle) < tol) || ((std::abs(angle) - M_PI) < tol));
	}

	{
		// mid-face

		result->Get(make_vec<size_t>(95, 55, 55), &v);
		auto v_float = v.convert<float>();

		Eigen::Map<Eigen::Vector3f> v0(v_float.nxyz_curv.data());

		double angle = std::acos(std::abs(ref1.dot(v0)));

		ASSERT_TRUE((std::abs(angle) < tol) || ((std::abs(angle) - M_PI) < tol));

		// curvature in corner should be (much) higher
		ASSERT_LT(v_float.curvature(), curv_corner);

		// curvature mid-face should be near zero
		ASSERT_TRUE(std::abs(v_float.curvature()) < tol);
	}


}
