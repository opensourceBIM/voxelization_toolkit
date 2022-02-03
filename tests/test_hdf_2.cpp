#include "../voxelizer.h"
#include "../writer.h"
#include "../processor.h"

#ifdef WITH_IFC
#include <ifcparse/IfcFile.h>
#ifdef IFCOPENSHELL_05
#include <ifcgeom/IfcGeomIterator.h>
using namespace Ifc2x3;
#else
#include <ifcgeom_schema_agnostic/IfcGeomIterator.h>
#endif
#endif

#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

#include <gtest/gtest.h>

#ifdef WIN32
#define DIRSEP "\\"
#else
#define DIRSEP "/"
#endif


#if defined(WITH_IFC) && !defined(IFCOPENSHELL_05)
TEST(Voxelization, IfcWallIds) {
	const std::string input_filename = ".." DIRSEP "tests" DIRSEP "fixtures" DIRSEP "duplex.ifc";

	IfcParse::IfcFile ifc_file(input_filename);

	const double d = 0.5;

	ASSERT_TRUE(ifc_file.good());

	IfcGeom::IteratorSettings settings_surface;
	settings_surface.set(IfcGeom::IteratorSettings::DISABLE_TRIANGULATION, true);
	settings_surface.set(IfcGeom::IteratorSettings::USE_WORLD_COORDS, true);

	IfcGeom::entity_filter ef;
	ef.include = true;
	ef.entity_names = { "IfcWall" };
	ef.traverse = false;

	auto filters = std::vector<IfcGeom::filter_t>({ ef });

#ifdef IFCOPENSHELL_07
	IfcGeom::Iterator it(settings_surface, &ifc_file, filters, 1);
#else
	IfcGeom::Iterator<double> it(settings_surface, &ifc_file, filters, 1);
#endif
	ASSERT_TRUE(it.initialize());

	geometry_collection_t geoms;

	while (true) {
		TopoDS_Compound C = it.get_native()->geometry().as_compound();
		BRepMesh_IncrementalMesh(C, 0.001);
		geoms.push_back({ it.get_native()->id(), C });
		if (!it.next()) {
			break;
		}
	}

	Bnd_Box global_bounds;
	for (auto& P : geoms) {
		BRepBndLib::Add(P.second, global_bounds);
	}

	double x1, y1, z1, x2, y2, z2;
	global_bounds.Get(x1, y1, z1, x2, y2, z2);
	int nx = (int)ceil((x2 - x1) / d) + 10;
	int ny = (int)ceil((y2 - y1) / d) + 10;
	int nz = (int)ceil((z2 - z1) / d) + 10;

	x1 -= d * 5;
	y1 -= d * 5;
	z1 -= d * 5;

	chunked_voxel_storage<voxel_uint32_t>* storage = new chunked_voxel_storage<voxel_uint32_t>(x1, y1, z1, d, nx, ny, nz, 64);

	{
		progress_writer progress_1("test_wall_ids");
		processor pr(storage, progress_1);
		pr.use_scanline() = false;
		pr.process(geoms.begin(), geoms.end(), VOLUME_PRODUCT_ID(), output(MERGED(), "test_wall_ids.vox"));
	}

	// PRINT WORLD BOUNDS OF STORAGE...e b

	//std::ofstream fs("boundaries.obj");
	//obj_export_helper oeh{ fs };
	//storage->obj_export(oeh, false, true);


	hdf_writer writer;
	writer.SetVoxels(storage);
	writer.Write("test_walls.h5");

}
#else
TEST(Voxelization, DISABLED_IfcSpaceIds) {}
#endif