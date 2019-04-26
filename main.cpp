// todo memmap
// todo blosc

#include "processor.h"
#include "progress.h"
#include "voxelizer.h"
#include "writer.h"
#include "storage.h"
#include "dim3.h"
#include "polyfill.h"
#include "edge_detect.h"
#include "fill_gaps.h"
#include "offset.h"
#include "traversal.h"

#include <ifcgeom/IfcGeomIterator.h>
#include <ifcparse/IfcFile.h>

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Shell.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <BRepTools_WireExplorer.hxx>

#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/program_options.hpp>

#include <stdlib.h>
#include <vector>
#include <array>
#include <iostream>
#include <functional>
#include <thread>
#include <mutex>

#define PADDING 10

typedef chunked_voxel_storage<bit_t> voxel_storage_t;

template <typename T>
TopAbs_ShapeEnum get_geometry_enum();

template <>
TopAbs_ShapeEnum get_geometry_enum<TopoDS_Shell>() {
	return TopAbs_SHELL;
}

template <>
TopAbs_ShapeEnum get_geometry_enum<TopoDS_Solid>() {
	return TopAbs_SOLID;
}

template <>
TopAbs_ShapeEnum get_geometry_enum<TopoDS_Shape>() {
	return TopAbs_SHAPE;
}

template <typename T> T cast_geometry(const TopoDS_Shape& s);

template <>
TopoDS_Shell cast_geometry<>(const TopoDS_Shape& s) {
	return TopoDS::Shell(s);
}

template <>
TopoDS_Solid cast_geometry<>(const TopoDS_Shape& s) {
	return TopoDS::Solid(s);
}

template <>
TopoDS_Shape cast_geometry<>(const TopoDS_Shape& s) {
	return s;
}


static size_t num_threads;

namespace po = boost::program_options;


// 0.1 Duplex_A_20110907_optimized.ifc surfaces.vox walkable.vox site.vox spaces\%d.vox doors\%d.vox
// D:\documents\Singapore\models\duplex
int main(int argc, char** argv) {

	po::options_description opts("Command line options");
	opts.add_options()
		("threads,t", po::value<size_t>(), "number of parallel processing threads")
		("size,d", po::value<double>(), "voxel size in meters")
		("chunk,c", po::value<size_t>(), "chunk size in number of voxels")
		("mmap,m", "use memory-mapped files instead of pure RAM")
		("input-file", po::value<std::string>(), "input IFC file")
		("output-file", po::value<std::string>(), "output voxel file");

	po::positional_options_description positional_options;
	positional_options.add("input-file", 1);
	positional_options.add("output-file", 1);

	po::variables_map vmap;
	try {
		po::store(po::command_line_parser(argc, argv).
			options(opts).positional(positional_options).run(), vmap);
	} catch (const std::exception& e) {
		std::cerr << "[Error] " << e.what() << "\n\n";
		return 1;
	}

	if (!vmap.count("input-file")) {
		std::cerr << "[Error] Input file not specified" << std::endl;
		return 1;
	}

	if (!vmap.count("output-file")) {
		std::cerr << "[Error] Output file not specified" << std::endl;
		return 1;
	}

	double d = 0.01;

	if (!vmap.count("size")) {
		std::cerr << "[Info] Using default size 0.01m" << std::endl;
	}

	if (vmap.count("size")) {
		d = vmap["size"].as<double>();
	}

	boost::optional<size_t> threads, chunk;
	if (vmap.count("threads")) {
		threads = vmap["threads"].as<size_t>();
	}
	if (vmap.count("chunk")) {
		chunk = vmap["chunk"].as<size_t>();
	}

	const bool mmap = vmap.count("mmap") != 0;

	const std::string input_filename = vmap["input-file"].as<std::string>();
	const std::string output_filename = vmap["output-file"].as<std::string>();
	const std::string output_filename_obj = output_filename + ".obj";

	IfcParse::IfcFile* ifc_file = new IfcParse::IfcFile;
	if (!ifc_file->Init(input_filename)) {
		std::cout << "Failed to open file" << std::endl;
		return 1;
	}

	IfcGeom::IteratorSettings settings_surface;
	settings_surface.set(IfcGeom::IteratorSettings::DISABLE_TRIANGULATION, true);
	// settings_surface.set(IfcGeom::IteratorSettings::USE_WORLD_COORDS, true);
	
	IfcGeom::IteratorSettings settings_solid;
	settings_solid.set(IfcGeom::IteratorSettings::DISABLE_TRIANGULATION, true);
	// settings_solid.set(IfcGeom::IteratorSettings::USE_WORLD_COORDS, true);
	settings_solid.set(IfcGeom::IteratorSettings::SEW_SHELLS, true);
		
	IfcGeom::entity_filter ef_surface;
	ef_surface.include = false;
	ef_surface.traverse = false;
	ef_surface.values = { /* IfcSchema::Type::IfcBuildingElementProxy, */ IfcSchema::Type::IfcSpace, IfcSchema::Type::IfcOpeningElement, /* IfcSchema::Type::IfcDoor, */ IfcSchema::Type::IfcFurnishingElement };
	
	IfcGeom::entity_filter ef_solid;
	ef_solid.include = true;
	ef_solid.traverse = false;
	ef_solid.values = { IfcSchema::Type::IfcOpeningElement, IfcSchema::Type::IfcSpace };
	
	auto filters_surface = std::vector<IfcGeom::filter_t>({ ef_surface });
	auto filters_solid = std::vector<IfcGeom::filter_t>({ ef_solid });
	// IfcGeom::Iterator<double> it(settings, ifc_file, filters);
	
	std::vector< IfcGeom::Iterator<double>* > iterators(2);
	iterators[0] = new IfcGeom::Iterator<double>(settings_surface, ifc_file, filters_surface);
	iterators[1] = new IfcGeom::Iterator<double>(settings_solid,   ifc_file, filters_solid  );

	bool initialized[2] = { true, false };
	
	for (auto jt = iterators.begin(); jt != iterators.end() - 1; ++jt) {
		if (!(**jt).initialize()) {
			initialized[std::distance(iterators.begin(), jt)] = false;
		}
	}

	if (!initialized[0]) { // && !initialized[1]) {
		std::cout << Logger::GetLog();
		std::cout << "Failed to open IFC file" << std::endl;
		return 1;
	}
	
	const std::set<std::string> walkable_types = { "IfcSlab", "IfcStairFlight", "IfcRampFlight" };
	
	Bnd_Box global_bounds;
	geometry_collection_t geometries;
	geometry_collection_t spaces;
	geometry_collection_t openings;
	geometry_collection_t sites;
	
	IfcGeom::Kernel kernel;
	kernel.initializeUnits(*ifc_file->entitiesByType<IfcSchema::IfcUnitAssignment>()->begin());

	auto buildings = ifc_file->entitiesByType<IfcSchema::IfcBuilding>();
	if (buildings->size() != 1) {
		std::cerr << "Expected a single IfcBuilding" << std::endl;
	}
	gp_Trsf trsf;
	kernel.convert((*buildings->begin())->ObjectPlacement(), trsf);
	double building_z = trsf.TranslationPart().Z();

	for (auto jt = iterators.begin(); jt != iterators.end(); ++jt) {
		if (!initialized[std::distance(iterators.begin(), jt)]) {
			continue;
		}

	auto& it = **jt;
	int old_progress = -1;

	for (;;) {
		TopoDS_Compound compound;
		BRep_Builder builder;
		builder.MakeCompound(compound);

		elem_t* elem = (elem_t*)it.get();
		
		const gp_Trsf& product_trsf = elem->transformation().data();
		const bool walkable = walkable_types.find(elem->type()) != walkable_types.end();
		const bool railing = elem->type() == IfcSchema::Type::ToString(IfcSchema::Type::IfcRailing);
		
		try {
		IfcSchema::IfcLocalPlacement* p = elem->product()->ObjectPlacement()->as<IfcSchema::IfcLocalPlacement>();
		if (p && kernel.is_identity_transform(p->RelativePlacement())) {
			auto c0 = p->PlacementRelTo()->as<IfcSchema::IfcLocalPlacement>()->RelativePlacement()->as<IfcSchema::IfcAxis2Placement3D>()->Location()->Coordinates();
			auto c1 = p->PlacementRelTo()->as<IfcSchema::IfcLocalPlacement>()->PlacementRelTo()->as<IfcSchema::IfcLocalPlacement>()->RelativePlacement()->as<IfcSchema::IfcAxis2Placement3D>()->Location()->Coordinates();
			if (c0[2] > 50000 && c1[2] < -50000) {
				goto skip;
			}
		}
		} catch(...) {}

		/* if (elem->product()->GlobalId() != "1nOs6Hg0v9fR$sLR1LjIyX") {
			goto skip;
		} */

		if (boost::to_lower_copy(elem->name()).find("nulpunt") != std::string::npos) {
			goto skip;
		}

		// Doors are specifically excluded
		// if (elem->type() == "IfcDoor") goto skip;
		if (elem->type() == "IfcOpeningElement") {
			IfcSchema::IfcOpeningElement* opening = (IfcSchema::IfcOpeningElement*) it.getFile()->entityById(elem->id());
			IfcSchema::IfcRelFillsElement::list::ptr fills = opening->HasFillings();
			bool has_door = false;
			for (auto it = fills->begin(); it != fills->end(); ++it) {
				if ((*it)->RelatedBuildingElement()->is(IfcSchema::Type::IfcDoor)) {
					has_door = true;
				}
			}
			if (!has_door) goto skip;
		}

		for (IfcGeom::IfcRepresentationShapeItems::const_iterator it = elem->geometry().begin(); it != elem->geometry().end(); ++it) {
			bool is_transparent = it->hasStyle() && it->Style().Transparency() && (*it->Style().Transparency()) > 0.;
				
			const TopoDS_Shape& s = it->Shape();
			gp_GTrsf trsf = it->Placement();
			// const TopoDS_Shape s2 = IfcGeom::Kernel::apply_transformation(s, trsf);
			// const TopoDS_Shape moved_shape = IfcGeom::Kernel::apply_transformation(s2, elem->transformation().data());
			const TopoDS_Shape moved_shape_ = IfcGeom::Kernel::apply_transformation(s, trsf);
			const TopoDS_Shape moved_shape = IfcGeom::Kernel::apply_transformation(moved_shape_, product_trsf);
			// const TopoDS_Shape moved_shape = BRepBuilderAPI_Transform(s, trsf, true);
			BRepBndLib::Add(moved_shape, global_bounds);

			/* if (elem->product()->as<IfcSchema::IfcBuildingElement>()->Tag() != "143534") {
				continue;
			} */
			
			if (get_geometry_enum<geometry_t>() == TopAbs_SHAPE) {
				builder.Add(compound, moved_shape);
			} else {
				TopExp_Explorer exp(moved_shape, get_geometry_enum<geometry_t>());
				for (; exp.More(); exp.Next()) {
					geometry_t part = cast_geometry<geometry_t>(exp.Current());
					builder.Add(compound, part);
				}
			}
		}

		BRepMesh_IncrementalMesh(compound, 0.001);

		if (elem->type() == "IfcSpace") {
			spaces.push_back(std::make_pair(elem->id(), compound));
		} else if (elem->type() == "IfcOpeningElement") {
			openings.push_back(std::make_pair(elem->id(), compound));
		} else {
			int code = 0;
			if (walkable) code += 1;
			if (railing) code += 2;
			geometries.push_back(std::make_pair(code, compound));
		}

		if (elem->type() == "IfcSite") {
			sites.push_back(std::make_pair(elem->id(), compound));
		}

	skip:

		if (old_progress != it.progress()) {
			std::cerr << "\rgeometry " << it.progress();
			old_progress = it.progress();
		}
		
		if (!it.next()) {
			std::cerr << std::endl;
			break;
		}
	}
	}

	// delete ifc_file;
	for (auto it = iterators.begin(); it != iterators.end(); ++it) {
		// delete *it;
	}

	// Elements of similar type often appear consecutively. Randomize to have threading behave better.
	std::random_shuffle(geometries.begin(), geometries.end());

	double x1, y1, z1, x2, y2, z2;
	global_bounds.Get(x1, y1, z1, x2, y2, z2);
	int nx = (int) ceil((x2 - x1) / d);
	int ny = (int) ceil((y2 - y1) / d);
	int nz = (int) ceil((z2 - z1) / d);

#ifdef PADDING
	x1 -= d * PADDING;
	y1 -= d * PADDING;
	z1 -= d * PADDING;
	nx += PADDING * 2;
	ny += PADDING * 2;
	nz += PADDING * 2;
#endif

	std::cerr << nx << " x " << ny << " x " << nz << std::endl;

	factory fac;
	if (chunk) {
		fac.chunk_size(*chunk);
	}
	if (mmap) {
		fac.mmap(output_filename);
	}

	auto get_voxelizer = [&threads, &chunk](double x1, double y1, double z1, double d, size_t nx, size_t ny, size_t nz, progress_writer& progress) {
		if (threads) {
			return (abstract_processor*) new threaded_processor(x1, y1, z1, d, nx, ny, nz, chunk.get_value_or(0), *threads, progress);
		} else {
			return (abstract_processor*) new processor(x1, y1, z1, d, nx, ny, nz, chunk.get_value_or(0), progress);
		}
	};

	progress_writer progress_1("fusing surfaces");
	auto fused_surfaces = get_voxelizer(x1, y1, z1, d, nx, ny, nz, progress_1);
	fused_surfaces->process(geometries.begin(), geometries.end(), SURFACE(), output(MERGED(), output_filename)
		.post(threaded_post_process<fill_gaps>(threads.get_value_or(1)))
		.post(threaded_post_process< offset<> >(threads.get_value_or(1)))
		.post(threaded_post_process<keep_outmost>(1)));

	// 'Undo' the offset.
	auto offset1 = fused_surfaces->voxels();
	auto offset02 = threaded_post_process< offset<> >(threads.get_value_or(1))((regular_voxel_storage*)offset1);
	auto offset2 = threaded_post_process<keep_outmost>(1)(offset02);
	offset02->boolean_subtraction(offset2);
	auto final_result = offset02;

	/*
	// Some stuff for creating a thicker wall for 3d printing.
	for (size_t i = 0; i < 10; ++i) {
		auto offset02 = threaded_post_process< offset<> >(threads.get_value_or(1))(final_result);
		auto offset2 = threaded_post_process<keep_outmost>(1)(offset02);
		offset02->boolean_subtraction(offset2);
		final_result->boolean_union(offset02);
		std::cout << "final_result.count " << final_result->count() << std::endl;
	}
	*/

	auto bottom = final_result->empty_copy();
	size_t building_z_voxel;
	bottom->GetVoxelZ(building_z, building_z_voxel);
	for (size_t i = 0; i < bottom->extents().get<0>(); ++i) {
		for (size_t j = 0; j < bottom->extents().get<1>(); ++j) {
			for (size_t k = 0; k <= building_z_voxel; ++k) {
				bottom->Set({ i, j, k });
			}
		}
	}

	final_result->boolean_subtraction(bottom);

	std::cout << final_result->count() * (d*d) << "m2";
	
	voxel_writer writer;
	writer.SetVoxels(final_result);
	writer.Write(output_filename + "-final.vox");
	
	// std::ofstream obj_fs(output_filename_obj.c_str());
	// ((regular_voxel_storage*)final_result)->obj_export(obj_fs);

	delete fused_surfaces;
	
	/*
	{
		progress_writer progress("doors");
		voxelizer_t filler(x1, y1, z1, d, nx, ny, nz, progress);
		filler.process(openings.begin(), openings.end(), processor::VOLUME(), processor::output(processor::SEPARATE(), door_pattern));
	}
		
	{
		progress_writer progress("site");
		voxelizer_t filler(x1, y1, z1, d, nx, ny, nz, progress);
		filler.process(sites.begin(), sites.end(), processor::SURFACE(), processor::output(processor::MERGED(), site_surfaces_name));
	}

	{
		progress_writer progress_1("fusing surfaces");
		voxelizer_t fused_surfaces(x1, y1, z1, d, nx, ny, nz, progress_1);
		fused_surfaces.process(geometries.begin(), geometries.end(), processor::SURFACE(), processor::output(processor::MERGED(), slab_surfaces_name));
		progress_1.end();

		progress_writer progress_2("spaces");
		voxelizer_t spaces_volumes(x1, y1, z1, d, nx, ny, nz, progress_2);
		// spaces_volumes.process(spaces.begin(), spaces.end(), processor::VOLUME(), processor::output(processor::SEPARATE_MINUS(), fused_surfaces.voxels(), space_pattern));
		spaces_volumes.process(spaces.begin(), spaces.end(), processor::VOLUME(), processor::output(processor::SEPARATE(), space_pattern));
	}

	{
		progress_writer progress("walkable surfaces");
		voxelizer_t walkable_surfaces(x1, y1, z1, d, nx, ny, nz, progress);
		geometry_collection_t walkable;
		for (geometry_collection_t::const_iterator it = geometries.begin(); it != geometries.end(); ++it) {
			if ((it->first & 1) == 1) {
				TopoDS_Compound compound;
				BRep_Builder builder;
				builder.MakeCompound(compound);
				TopExp_Explorer exp(it->second, TopAbs_FACE);
				for (; exp.More(); exp.Next()) {
					const TopoDS_Face& face = TopoDS::Face(exp.Current());
					double u0, u1, v0, v1;
					BRepTools::UVBounds(face, u0, u1, v0, v1);
					gp_Pnt p; gp_Vec v;
					BRepGProp_Face prop(face);
					prop.Normal((u0 + u1) / 2., (v0 + v1) / 2., p, v);
					if (v.Z() > 0.8) {
						builder.Add(compound, face);
					}
				}
				walkable.push_back(std::make_pair(0, compound));
			}
		}
		walkable_surfaces.process(walkable.begin(), walkable.end(), processor::SURFACE(), processor::output(processor::MERGED(), walkable_surfaces_name));
	}
	*/
}
