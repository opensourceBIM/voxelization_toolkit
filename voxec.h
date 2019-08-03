#ifndef VOXEC_H
#define VOXEC_H

#include "voxelfile.h"
#include "processor.h"
#include "storage.h"
#include "offset.h"
#include "fill_gaps.h"
#include "traversal.h"

#ifdef WITH_IFC
#include <ifcparse/IfcFile.h>
#ifdef IFCOPENSHELL_05
#include <ifcgeom/IfcGeomIterator.h>
#else
#include <ifcgeom_schema_agnostic/IfcGeomIterator.h>
#endif

#include <boost/filesystem.hpp>
#else
namespace IfcParse {
	class IfcFile {};
}
#endif

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>

#include <set>
#include <map>
#include <exception>
#include <functional>

#include <boost/variant/apply_visitor.hpp>
#include <boost/tokenizer.hpp>
#include <boost/range/iterator_range.hpp>


#ifdef WIN32
#define DIRSEP "\\"
#else
#define DIRSEP "/"
#endif

typedef boost::variant<boost::blank, std::vector<IfcParse::IfcFile*>, geometry_collection_t*, abstract_voxel_storage*, function_arg_value_type> symbol_value;

class voxel_operation;

class scope_map : public std::map<std::string, symbol_value> {
protected:
	template <typename T>
	typename std::enable_if<boost::mpl::contains<symbol_value::types, T>::type::value, const T&>::type
		get_value_(const symbol_value& v) const {
		return boost::get<T>(v);
	}

	template <typename T>
	typename std::enable_if<boost::mpl::contains<function_arg_value_type::types, T>::type::value, const T&>::type
		get_value_(const symbol_value& v) const {
		return boost::get<T>(boost::get<function_arg_value_type>(v));
	}

public:
	class not_in_scope : public std::runtime_error {
		using std::runtime_error::runtime_error;
	};

	template <typename T>
	const T& get_value_or(const std::string& symbol, const T& default_value) const {
		auto it = find(symbol);
		if (it == end()) {
			return default_value;
		}
		return get_value_<T>(it->second);
	}

	template <typename T>
	const T& get_value(const std::string& symbol) const {
		auto it = find(symbol);
		if (it == end()) {
			throw not_in_scope("Not in scope " + symbol);
		}
		return get_value_<T>(it->second);
	}

	const int get_length(const std::string& symbol) const {
		auto it = find(symbol);
		if (it == end()) {
			throw not_in_scope("Not in scope " + symbol);
		}
		try {
			return get_value_<int>(it->second);
		} catch (boost::bad_get&) {
			return std::ceil(get_value_<double>(it->second) / get_value<double>("VOXELSIZE"));
		}
	}

	bool has(const std::string& symbol) const {
		auto it = find(symbol);
		return it != end();
	}
};

struct voxel_operation_map {
public:
	typedef std::map<std::string, std::function<voxel_operation*()> > map_t;

	static map_t& map();

	static voxel_operation* create(const std::string& s) {
		auto it = map().find(s);
		if (it == map().end()) {
			throw std::runtime_error("No operation named " + s);
		}
		return it->second();
	}
};

struct argument_spec {
	bool required;
	std::string name;
	std::string type;
};

class voxel_operation {
public:
	virtual const std::vector<argument_spec>& arg_names() const = 0;
	virtual symbol_value invoke(const scope_map& scope) const = 0;
};

#ifdef WITH_IFC
class op_parse_ifc_file : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "string"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		using namespace boost::filesystem;

		const std::string& pattern = scope.get_value<std::string>("input");
		boost::regex regex = IfcGeom::wildcard_filter::wildcard_string_to_regex(pattern);
		
		std::vector<IfcParse::IfcFile*> files;

		for (auto& entry : boost::make_iterator_range(recursive_directory_iterator("."), {})) {
			// strip off ./
			std::string filename = entry.path().string().substr(2);
			if (boost::regex_match(filename, regex)) {
				std::string filename = entry.path().string();
				{
					std::ifstream fs(filename.c_str());
					if (!fs.good()) {
						throw std::runtime_error("Unable to open file " + filename);
					}
				}
#ifdef IFCOPENSHELL_05
				IfcParse::IfcFile* f = new IfcParse::IfcFile();
				if (!f->Init()) {
#else
				IfcParse::IfcFile* f = new IfcParse::IfcFile(filename);
				if (!f->good()) {
#endif
					throw std::runtime_error("Unable to open file " + filename);
				}

				files.push_back(f);
			}
		}
		
		if (files.empty()) {
			throw std::runtime_error("Not a single file matched pattern");
		}

		return files;
	}
};

class op_create_geometry : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "ifcfile" }, { false, "include", "sequence"}, { false, "exclude", "sequence"} };
		return nm_;
	}
		
	symbol_value invoke(const scope_map& scope) const {

		IfcGeom::entity_filter ef;

#ifdef IFCOPENSHELL_05
		auto ifc_roof = IfcSchema::Type::IfcRoof;
		auto ifc_slab = IfcSchema::Type::IfcSlab;
		auto ifc_space = IfcSchema::Type::IfcSpace;
		auto ifc_opening = IfcSchema::Type::IfcOpeningElement;
		auto ifc_furnishing = IfcSchema::Type::IfcFurnishingElement;
		auto& ef_elements = ef_elements;
#else
		// From IfcOpenShell v0.6.0 onwards, there is support for multiple
		// schemas at runtime so type identification is based on strings.
		std::string ifc_roof = "IfcRoof";
		std::string ifc_slab = "IfcSlab";
		std::string ifc_space = "IfcSpace";
		std::string ifc_opening = "IfcOpeningElement";
		std::string ifc_furnishing = "IfcFurnishingElement";
		auto& ef_elements = ef.entity_names;
#endif

		const std::vector<IfcParse::IfcFile*>& ifc_files = scope.get_value<std::vector<IfcParse::IfcFile*>>("input");

		IfcGeom::IteratorSettings settings_surface;
		settings_surface.set(IfcGeom::IteratorSettings::DISABLE_TRIANGULATION, true);
		settings_surface.set(IfcGeom::IteratorSettings::USE_WORLD_COORDS, true);

		boost::optional<bool> include, roof_slabs;
		std::vector<std::string> entities;
		if (scope.has("include")) {
			entities = scope.get_value<std::vector<std::string> >("include");
			include = true;
		}
		if (scope.has("exclude")) {
			if (!entities.empty()) {
				throw std::runtime_error("include and exclude cannot be specified together");
			}
			entities = scope.get_value<std::vector<std::string> >("exclude");
			include = false;
		}

		if (include) {
			ef.include = *include;
			ef.traverse = true;

			std::vector<std::string> entities_without_quotes;
			std::transform(entities.begin(), entities.end(), std::back_inserter(entities_without_quotes), [](const std::string& v) {
				return v.substr(1, v.size() - 2);
			});

#ifdef IFCOPENSHELL_05
			std::transform(entities_without_quotes.begin(), entities_without_quotes.end(), std::inserter(ef_elements, ef_elements.begin()), [](const std::string& v) {
				return IfcSchema::Type::FromString(boost::to_upper_copy(v));
			});
#else
			ef_elements.insert(entities_without_quotes.begin(), entities_without_quotes.end());
#endif

			if (*include) {
				if (ef_elements.find(ifc_roof) != ef_elements.end()) {
					// Including { IfcRoof, ... } then we need to include Slabs (as they are potentially roof slabs)
					ef_elements.insert(ifc_slab);
					roof_slabs = true;
				}
			} else {
				if (ef_elements.find(ifc_roof) == ef_elements.end()) {
					// Excluding NOT IfcRoof then we need to exclude Slabs as well (as they are potentially roof slabs)
					if (ef_elements.erase(ifc_slab) > 0) {
						roof_slabs = false;
					}
				}
			}
		} else {
			ef.include = false;
			ef.traverse = false;
			ef_elements = { ifc_space, ifc_opening, ifc_furnishing };
		}

		geometry_collection_t* geometries = new geometry_collection_t;

		auto filters_surface = std::vector<IfcGeom::filter_t>({ ef });

		bool at_least_one_succesful = false;

		for (auto ifc_file : ifc_files) {

			IfcGeom::Iterator<double> iterator(settings_surface, ifc_file, filters_surface);
			
			// For debugging geometry creation from IfcOpenShell
			// Logger::SetOutput(&std::cout, &std::cout);

			if (!iterator.initialize()) {
				continue;
			}

			at_least_one_succesful = true;

			int old_progress = -1;

			for (;;) {
				elem_t* elem = (elem_t*)iterator.get();
				bool process = true;

				if (boost::to_lower_copy(elem->name()).find("nulpunt") != std::string::npos) {
					process = false;
				}

#ifdef IFCOPENSHELL_05
				if (roof_slabs && elem->product()->as<IfcSchema::IfcSlab>()) {
					auto pdt = elem->product()->as<IfcSchema::IfcSlab>()->PredefinedType();
					process = (pdt == IfcSchema::IfcSlabTypeEnum::IfcSlabType_ROOF) == *roof_slabs;
				}
#else
				if (roof_slabs && elem->product()->declaration().is("IfcSlab")) {
					std::string pdt = *elem->product()->get("PredefinedType");
					process = (pdt == "ROOF") == *roof_slabs;
				}
#endif

				if (process) {
					TopoDS_Compound compound = elem->geometry().as_compound();
					BRepMesh_IncrementalMesh(compound, 0.001);
					geometries->push_back(std::make_pair(elem->id(), compound));
				}

				if (old_progress != iterator.progress()) {
					std::cerr << "\rparsing geometry " << iterator.progress();
					old_progress = iterator.progress();
				}

				if (!iterator.next()) {
					std::cerr << std::endl;
					break;
				}
			}

		}

		if (!at_least_one_succesful) {
			throw std::runtime_error("Failed to generate geometry");
		}

		std::random_shuffle(geometries->begin(), geometries->end());

		return geometries;
	}
};
#endif

namespace {
	abstract_voxel_storage* voxelize(geometry_collection_t* surfaces, double vsize, int chunksize, const boost::optional<int>& threads) {
		double x1, y1, z1, x2, y2, z2;
		int nx, ny, nz;

		Bnd_Box global_bounds;
		for (auto& p : *surfaces) {
			BRepBndLib::Add(p.second, global_bounds);
		}

		const size_t PADDING = 32U;

		global_bounds.Get(x1, y1, z1, x2, y2, z2);
		nx = (int)ceil((x2 - x1) / vsize);
		ny = (int)ceil((y2 - y1) / vsize);
		nz = (int)ceil((z2 - z1) / vsize);

		x1 -= vsize * PADDING;
		y1 -= vsize * PADDING;
		z1 -= vsize * PADDING;
		nx += PADDING * 2;
		ny += PADDING * 2;
		nz += PADDING * 2;

		if (threads) {
			progress_writer progress("voxelize");
			threaded_processor p(x1, y1, z1, vsize, nx, ny, nz, chunksize, *threads, progress);
			p.process(surfaces->begin(), surfaces->end(), SURFACE(), output(MERGED()));
			return p.voxels();
		} else {
			progress_writer progress;
			auto voxels = factory().chunk_size(chunksize).create(x1, y1, z1, vsize, nx, ny, nz);
			// nb: the other constructor would tell the constructor to delete the created voxels
			processor p(voxels, progress);
			p.process(surfaces->begin(), surfaces->end(), SURFACE(), output(MERGED()));
			return voxels;
		}
	}
}

class op_voxelize : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "surfaceset" }, {false, "VOXELSIZE", "real"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		geometry_collection_t* surfaces = scope.get_value<geometry_collection_t*>("input");

		double vsize = scope.get_value<double>("VOXELSIZE");
		int cs = scope.get_value<int>("CHUNKSIZE");
		int t = scope.get_value<int>("THREADS");

		return voxelize(surfaces, vsize, cs, t);
	}
};

class op_print_components : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		connected_components((regular_voxel_storage*) voxels, [](regular_voxel_storage* c) {
			std::cout << "Component " << c->count() << std::endl;
		});
		symbol_value v;
		return v;
	}
};

namespace {
	void dump_info(abstract_voxel_storage* voxels) {
		if (dynamic_cast<abstract_chunked_voxel_storage*>(voxels)) {
			auto left = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->grid_offset();
			auto nc = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->num_chunks();
			auto right = (left + nc.as<long>()) - (decltype(left)::element_type)1;
			auto sz = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->voxel_size();
			auto szl = (long)dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->chunk_size();
			auto left_world = ((voxels->bounds()[0].as<long>() + left * szl).as<double>() * sz);
			auto right_world = ((voxels->bounds()[1].as<long>() + left * szl).as<double>() * sz);
			std::cerr << "voxels: "
				<< left.format() << " - " << right.format()
				<< " ; count: " << voxels->count()
				<< " ; bounds: " << voxels->bounds()[0].format()
				<< " - " << voxels->bounds()[1].format() << std::endl
				<< " ; world: " << left_world.format()
				<< " - " << right_world.format() << std::endl;
		}
	}
}

class op_dump_surfaces : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input_voxels", "voxels" }, { true, "input_surfaces", "surfaceset" }, { true, "output_path", "string" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input_voxels");
		
		auto scope_copy = scope;
		geometry_collection_t* surfaces = scope.get_value<geometry_collection_t*>("input_surfaces");

		const std::string output_path = scope.get_value<std::string>("output_path");
		
		double vsize = scope.get_value<double>("VOXELSIZE");
		int cs = scope.get_value<int>("CHUNKSIZE");

		BRep_Builder B;

		TopoDS_Compound face_subset_all_elem;
		B.MakeCompound(face_subset_all_elem);

		for (auto& pair : *surfaces) {
			TopoDS_Compound face_subset;
			B.MakeCompound(face_subset);

			bool any = false;

			TopExp_Explorer exp(pair.second, TopAbs_FACE);
			
			for (; exp.More(); exp.Next()) {
				TopoDS_Compound C;
				B.MakeCompound(C);
				B.Add(C, exp.Current());
				geometry_collection_t single = { { pair.first, C} };				

				auto vs = voxelize(&single, vsize, cs, boost::none);
				auto x = vs->boolean_intersection(voxels);

				// std::cout << "#" << pair.first << std::endl;
				// std::cout << "vs ";
				// dump_info(vs);
				// std::cout << "x  ";
				// dump_info(x);

				if (vs->count() > 0 && x->count() * 2 >= vs->count()) {
					// @todo I thought that a valid non-null face would result in at least once voxel.
					//       there might be differences here with the scanline algorithm and box intersect
					//       algorithm.
					B.Add(face_subset, exp.Current());
					B.Add(face_subset_all_elem, exp.Current());
					any = true;
				}

				delete vs;
				delete x;
			}

			if (any) {
				std::string fn = output_path + DIRSEP + std::to_string(pair.first) + ".brep";
				std::ofstream fs(fn.c_str());
				BRepTools::Write(face_subset, fs);
			}
		}

		std::string fn = output_path + DIRSEP + "all.brep";
		std::ofstream fs(fn.c_str());
		BRepTools::Write(face_subset_all_elem , fs);

		symbol_value v;
		return v;
	}
};

class op_fill_gaps : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		return threaded_post_process<fill_gaps>(scope.get_value<int>("THREADS"))((regular_voxel_storage*) voxels);
	}
};

class op_offset : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		return threaded_post_process< offset<> >(scope.get_value<int>("THREADS"))((regular_voxel_storage*)voxels);
	}
};

class op_offset_xy : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "d", "integer|real" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		int d = scope.get_length("d");
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");

#if 0
		auto current = (regular_voxel_storage*)voxels;
		while (d) {
			auto ofs = threaded_post_process< offset<1, 2> >(scope.get_value<int>("THREADS"))(current);
			if (current != voxels) {
				// don't delete the voxels in program scope
				delete current;
			}
			current = ofs;
			d--;
		}
#else
		auto current = (regular_voxel_storage*) voxels->copy();
		while (d) {
			auto ofs = threaded_post_process< offset<1, 2> >(scope.get_value<int>("THREADS"))(current);
			auto next = (regular_voxel_storage*) current->boolean_union(ofs);
			delete ofs;
			delete current;
			current = next;
			d--;
		}
#endif

		return current;
	}
};


class op_outmost : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		return threaded_post_process<keep_outmost>(1)((regular_voxel_storage*)voxels);
	}
};

class op_count : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		function_arg_value_type var = (int) voxels->count();
		return var;
	}
};

class op_volume : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		return threaded_post_process<traversal_voxel_filler_separate_components>(1)((regular_voxel_storage*)voxels);
	}
};

class op_volume_2 : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		return threaded_post_process<traversal_voxel_filler_inverse>(1)((regular_voxel_storage*)voxels);
	}
};

class op_volume_inverted : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		return threaded_post_process<traversal_voxel_filler_inverted>(1)((regular_voxel_storage*)voxels);
	}
};

class op_invert : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		return voxels->inverted();
	}
};

class op_fill_volume : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		abstract_voxel_storage* volume = threaded_post_process<traversal_voxel_filler_separate_components>(1)((regular_voxel_storage*)voxels);
		abstract_voxel_storage* combined = volume->boolean_union(voxels);
		delete volume;
		return combined;
	}
};

template <abstract_voxel_storage*(abstract_voxel_storage::*Fn)(const abstract_voxel_storage*)>
class op_boolean : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "a", "voxels" }, { true, "b", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* a = scope.get_value<abstract_voxel_storage*>("a");
		abstract_voxel_storage* b = scope.get_value<abstract_voxel_storage*>("b");
		return (a->*Fn)(b);
	}
};

#include "resample.h"

class op_resample : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "factor", "integer"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		int value = scope.get_value<int>("factor");
		return resampler(value)((regular_voxel_storage*)voxels);
	}
};

class op_union : public op_boolean<&abstract_voxel_storage::boolean_union> {};
class op_subtract : public op_boolean<&abstract_voxel_storage::boolean_subtraction> {};
class op_intersect : public op_boolean<&abstract_voxel_storage::boolean_intersection> {};

#include "shift.h"
#include "sweep.h"
#include "collapse.h"

template <typename T>
class op_geom : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "dx", "integer|real" },{ true, "dy", "integer|real" },{ true, "dz", "integer|real" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");

		int dx = scope.get_length("dx");
		int dy = scope.get_length("dy");
		int dz = scope.get_length("dz");

		T s;
		return s(voxels, dx, dy, dz);
	}
};

class op_shift: public op_geom<shift> {};
class op_sweep : public op_geom<sweep> {};
class op_collapse : public op_geom<collapse> {};

class op_traverse : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "seed", "voxels" }, { false, "depth", "integer|real" }, { false, "connectedness", "integer" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		regular_voxel_storage* voxels = (regular_voxel_storage*) scope.get_value<abstract_voxel_storage*>("input");
		regular_voxel_storage* seed = (regular_voxel_storage*) scope.get_value<abstract_voxel_storage*>("seed");

		regular_voxel_storage* output = (regular_voxel_storage*)voxels->empty_copy();

		auto callback = [this, output](const tagged_index& pos) {
			if (pos.which == tagged_index::VOXEL) {
				output->Set(pos.pos);
			} else {
				((abstract_chunked_voxel_storage*)output)->create_constant(pos.pos, 1U);
			}
		};

		boost::optional<double> max_depth;
		try {
			max_depth = scope.get_length("depth");
		} catch (scope_map::not_in_scope&) {
			// traversal with unconstrained depth
		}

		// Type erasure to get a runtime argument into a template arg.
		std::function<void(decltype(callback), decltype(voxels), decltype(seed))> run_visitor;

		int C = 6;
		try {
			C = scope.get_value<int>("connectedness");
		} catch (scope_map::not_in_scope&) {
			// default 6 connectedness
		}

		if (C != 6 && C != 26) {
			throw std::runtime_error("Connectedness should be 6 or 26");
		}
		
		if (C == 6) {
			visitor<6> v;
			v.max_depth = max_depth;
			run_visitor = v;
		} else {
			visitor<26> v;
			v.max_depth = max_depth;
			run_visitor = v;
		}

		run_visitor(callback, voxels, seed);

		return output;
	}
};

class op_constant_like : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "value", "integer" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		regular_voxel_storage* voxels = (regular_voxel_storage*)scope.get_value<abstract_voxel_storage*>("input");
		int value = scope.get_value<int>("value");
		abstract_chunked_voxel_storage* output = (abstract_chunked_voxel_storage*)voxels->empty_copy();
		if (value == 1) {
			auto i = make_vec<size_t>(0U, 0U, 0U);
			auto j = output->num_chunks();
			BEGIN_LOOP2(i, j)
				output->create_constant(ijk, 1);
			END_LOOP;
		}
		return output;
	}
};

template <typename T>
T* instantiate() {
	return new T();
}

class symbol_value_assignment_visitor {
private:
	symbol_value* v_;
	std::set<std::string> type_;
	const scope_map& context_;

	void assert_type(const std::string& t) const {
		if (type_.find(t) == type_.end()) {
			std::stringstream ss;
			ss << "Expected value of type ";
			bool first = true;
			for (auto& v : type_) {
				if (!first) {
					ss << " or ";
				}
				first = false;
				ss << v;
			}
			ss << ", got " << t;
			throw std::runtime_error(ss.str());
		}
	}

public:
	typedef void result_type;

	symbol_value_assignment_visitor(symbol_value* v, const std::string& type, const scope_map& context)
		: v_(v)
		, context_(context)
	{
		boost::tokenizer<> t(type);
		for (auto& v: t) {
			type_.insert(v);
		}
	}

void operator()(std::vector<IfcParse::IfcFile*> const v) const {
	assert_type("ifcfile");
	(*v_) = v;
}

void operator()(geometry_collection_t* const v) const {
	assert_type("surfaceset");
	(*v_) = v;
}

void operator()(abstract_voxel_storage* const v) const {
	assert_type("voxels");
	(*v_) = v;
}

void operator()(int v) const {
	assert_type("integer");
	(*v_) = v;
}

void operator()(double v) const {
	assert_type("real");
	(*v_) = v;
}

void operator()(const std::string& v) const {
	if (v.at(0) == '"') {
		assert_type("string");
		(*v_) = v.substr(1, v.size() - 2);
	} else {
		auto it = context_.find(boost::get<std::string>(v));
		if (it == context_.end()) {
			throw std::runtime_error("Not defined: " + boost::get<std::string>(v));
		}
		it->second.apply_visitor(*this);
	}
}

void operator()(const std::vector<std::string>& v) const {
	assert_type("sequence");
	(*v_) = v;
}

void operator()(const function_arg_value_type& v) const {
	v.apply_visitor(*this);
}

void operator()(const boost::blank&) const {
	// throw error?
}
};

class voxel_operation_runner {
public:
	const statement_type& statement_;

	voxel_operation_runner(const statement_type& statement) :
		statement_(statement) {}

	symbol_value run(const statement_type& st, scope_map& context) {
		voxel_operation* op = voxel_operation_map::create(statement_.call().name());

		bool has_keyword = false;
		std::map<std::string, function_arg_value_type> function_values;
		size_t position = 0;

		for (auto& arg : statement_.call().args()) {
			if (has_keyword && !arg.has_keyword()) {
				throw std::runtime_error("Keyword argument supplied after positional argument");
			}
			has_keyword = arg.has_keyword();
			if (has_keyword) {
				if (function_values.find(*arg.keyword()) != function_values.end()) {
					throw std::runtime_error("Value for " + *arg.keyword() + "supplied multiple times");
				}
				function_values[*arg.keyword()] = arg.value();
			} else {
				if (position >= op->arg_names().size()) {
					throw std::runtime_error("Too many positional arguments");
				}
				function_values[op->arg_names()[position].name] = arg.value();
			}
			position++;
		}

		scope_map function_value_symbols;

		for (auto& p : op->arg_names()) {
			if (function_values.find(p.name) == function_values.end()) {
				if (p.required) {
					throw std::runtime_error("No value supplied for " + p.name);
				} else {
					continue;
				}
			}

			symbol_value_assignment_visitor v(&function_value_symbols[p.name], p.type, context);
			function_values[p.name].apply_visitor(v);
		}

		for (auto& p : context) {
			if (function_value_symbols.find(p.first) == function_value_symbols.end()) {
				function_value_symbols.insert(p);
			}
		}

		auto r = op->invoke(function_value_symbols);

		delete op;

		return r;
	}
};

scope_map run(const std::vector<statement_type>& statements, double size, size_t threads = 0, size_t chunk_size = 128, bool with_mesh = false);

#endif
