#include "../voxelizer.h"
#include "../volume.h"
#include "../writer.h"
#include "../edge_detect.h"

#include <gtest/gtest.h>

#include <BRepPrimAPI_MakeBox.hxx>

TEST(Volume, DISABLED_Edges) {
	auto storage = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 1000, 1000, 100, 32);

	for (int i = 0; i < 10; ++i) {
		for (int j = 0; j < 10; ++j) {
			BRepPrimAPI_MakeBox mb(gp_Pnt(10 * i, 10 * j, 0.), 8., 8., 8.);

			auto vox = voxelizer(mb.Solid(), storage);
			vox.Convert();
		}
	}

	/*
	volume_filler volume(storage);
	volume.fill();
	*/

	auto storage2 = edge_detect()(storage);

	std::cerr << storage->count() << " " << storage2->count() << std::endl;

	voxel_writer writer;
	writer.SetVoxels(storage2);
	writer.Write("test_edges.vox");
}
