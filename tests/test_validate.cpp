#include "../voxelizer.h"
#include "../volume.h"
#include "../writer.h"
#include "../traversal.h"

#include <gtest/gtest.h>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

#include <ctime>

namespace {
	void voxelize(double vsize, bool rotated) {
		auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., vsize, 10. / vsize, 10. / vsize, 10. / vsize, 32);

		gp_Ax2 ax;
		if (rotated) {
			ax = gp_Ax2(gp_Pnt(3, 4, 4), gp_Dir(1, 1, 1));
		} else {
			ax = gp_Ax2(gp_Pnt(1, 1, 1), gp::DZ());
		}

		BRepPrimAPI_MakeBox mb(ax, 3., 3., 3.);
		auto solid = mb.Solid();
		BRepMesh_IncrementalMesh(solid, 0.001);

		/*
		TopExp_Explorer exp(solid, TopAbs_VERTEX);
		std::set< std::tuple<double, double, double> > points;
		for (; exp.More(); exp.Next()) {
			gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(exp.Current()));
			points.insert(std::make_tuple(p.X(), p.Y(), p.Z()));
		}

		for (auto& p : points) {
			std::cout << std::get<0>(p) << " " << std::get<1>(p) << " " << std::get<2>(p) << std::endl;
		}
		*/

		double t_vox, t_vol;

		{
			std::clock_t c_start = std::clock();
			auto vox = voxelizer(solid, storage);
			vox.Convert();
			std::clock_t c_end = std::clock();

			t_vox = (double) (c_end - c_start) / CLOCKS_PER_SEC;
		}

		/*
		std::ofstream fs("rotated.obj");
		storage->obj_export(fs);
		*/
		
		regular_voxel_storage* volume;
		{
			std::clock_t c_start = std::clock();
			traversal_voxel_filler_inverse filler;
			volume = filler(storage);
			std::clock_t c_end = std::clock();

			t_vol = (double) (c_end - c_start) / CLOCKS_PER_SEC;
		}


		auto surface_count = storage->count();
		auto volume_count = volume->count();

		double total_volume = (volume_count + surface_count / 2) * (vsize * vsize * vsize);

		std::cout << std::setprecision(15) << vsize << "," << rotated << "," << t_vol << "," << t_vox << "," << total_volume << std::endl; 

		delete storage;
		delete volume;
	}
}

TEST(Validation, Volume) {
	std::cout.precision(15);

	for (int j = 1; j < 3; ++j) {
		double b = 1. / std::pow(10, j);
		for (int i = 1; i < 10; ++i) {
			voxelize(b * i, false);
			voxelize(b * i, true);
		}
	}
}
