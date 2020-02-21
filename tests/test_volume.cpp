#include "../voxelizer.h"
#include "../volume.h"
#include "../writer.h"
#include "../traversal.h"

#include <gtest/gtest.h>

#include <BRepPrimAPI_MakeBox.hxx>

TEST(DISABLED_Voxelizer, ApproxVolume) {
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

	size_t ncx, ncy, ncz;
	storage->num_chunks().tie(ncx, ncy, ncz);

	for (size_t i = 0; i < ncx; ++i) {
		for (size_t j = 0; j < ncy; ++j) {
			for (size_t k = 0; k < ncz; ++k) {
				if (storage->get_chunk({ i, j, k }) != nullptr) {
					std::cerr << vec_n<3, size_t>(i, j, k).format() << ": " << storage->get_chunk({ i, j, k })->count() << std::endl;
				}
			}
		}
	}

	std::cerr << "count: " << storage->count() << std::endl;

	voxel_writer writer;
	writer.SetVoxels(storage);
	writer.Write("test_volume.vox");
}

TEST(DISABLED_Voxelizer, TraversalVolumeSingle) {
	auto surface = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 100, 100, 100, 32);
	BRepPrimAPI_MakeBox mb(gp_Pnt(1., 1., 1.), 8., 8., 8.);
	auto vox = voxelizer(mb.Solid(), surface);
	vox.Convert();

	traversal_voxel_filler filler;
	auto volume = filler(surface);
	auto A = surface->count();
	auto B = volume->count();

	/* {
		voxel_writer writer;
		writer.SetVoxels(surface);
		writer.Write("test_volume_surface.vox");
	}

	{
		voxel_writer writer;
		writer.SetVoxels(volume);
		writer.Write("test_volume_volume.vox");
	} */

	ASSERT_EQ(A, 81 * 81 * 2 + 81 * 79 * 2 + 79 * 79 * 2);
	ASSERT_EQ(B, 79 * 79 * 79);
	ASSERT_EQ(A + B, 81 * 81 * 81);

	std::cerr << "count: " << A  << " + " << B << " = " << (A+B) << std::endl;

	delete surface;
	delete volume;
}

TEST(DISABLED_Voxelizer, TraversalVolumeDouble) {
	auto surface = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 200, 100, 100, 32);
	
	for (int i = 0; i < 2; ++i) {
		BRepPrimAPI_MakeBox mb(gp_Pnt(i * 10. + 1., 1., 1.), 8., 8., 8.);
		auto vox = voxelizer(mb.Solid(), surface);
		vox.Convert();
	}

	// Traversal finds a seed outside of the volumes and after inversion two volumes remain.
	traversal_voxel_filler filler;
	auto volume = filler(surface);
	auto A = surface->count();
	auto B = volume->count();

	ASSERT_EQ(A, 2 * (81 * 81 * 2 + 81 * 79 * 2 + 79 * 79 * 2));
	ASSERT_EQ(B, 2 * (79 * 79 * 79));
	ASSERT_EQ(A + B, 2 * (81 * 81 * 81));

	std::cerr << "count: " << A << " + " << B << " = " << (A + B) << std::endl;

	delete surface;
	delete volume;
}

TEST(Voxelizer, TraversalVolumeTriple) {
	auto surface = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 300, 100, 100, 32);

	for (int i = 0; i < 3; ++i) {
		BRepPrimAPI_MakeBox mb(gp_Pnt(i * 10. + 1., 1., 1.), 8., 8., 8.);
		auto vox = voxelizer(mb.Solid(), surface);
		vox.Convert();
	}

	// Traversal finds a seed inside one of the volumes and two remain empty
	traversal_voxel_filler filler;
	auto volume = filler(surface);
	auto A = surface->count();
	auto B = volume->count();

	// NB: 3 Surfaces 1 volume
	ASSERT_EQ(A, 3 * (81 * 81 * 2 + 81 * 79 * 2 + 79 * 79 * 2));
	ASSERT_EQ(B, 1 * (79 * 79 * 79));

	delete surface;
	delete volume;
}

TEST(Voxelizer, TraversalVolumeTripleSeparate) {
	auto surface = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 300, 100, 100, 32);

	for (int i = 0; i < 3; ++i) {
		BRepPrimAPI_MakeBox mb(gp_Pnt(i * 10. + 1., 1., 1.), 8., 8., 8.);
		auto vox = voxelizer(mb.Solid(), surface);
		vox.Convert();
	}

	// Traversal is done on separate distinct connected components
	traversal_voxel_filler_separate_components filler;
	auto volume = filler(surface);
	auto A = surface->count();
	auto B = volume->count();

	ASSERT_EQ(A, 3 * (81 * 81 * 2 + 81 * 79 * 2 + 79 * 79 * 2));
	ASSERT_EQ(B, 3 * (79 * 79 * 79));
	ASSERT_EQ(A + B, 3 * (81 * 81 * 81));

	delete surface;
	delete volume;
}

TEST(Voxelizer, TraversalVolumeTripleInverted) {
	auto surface = new chunked_voxel_storage<bit_t>(0., 0., 0., 0.1, 300, 100, 100, 32);

	for (int i = 0; i < 3; ++i) {
		BRepPrimAPI_MakeBox mb(gp_Pnt(i * 10. + 1., 1., 1.), 8., 8., 8.);
		auto vox = voxelizer(mb.Solid(), surface);
		vox.Convert();
	}

	// Traversal is done on external and then inverted
	traversal_voxel_filler_inverse filler;
	auto volume = filler(surface);
	auto A = surface->count();
	auto B = volume->count();

	ASSERT_EQ(A, 3 * (81 * 81 * 2 + 81 * 79 * 2 + 79 * 79 * 2));
	ASSERT_EQ(B, 3 * (79 * 79 * 79));
	ASSERT_EQ(A + B, 3 * (81 * 81 * 81));

	delete surface;
	delete volume;
}

