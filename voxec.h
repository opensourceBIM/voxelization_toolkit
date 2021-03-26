#ifndef VOXEC_H
#define VOXEC_H

#include "voxelfile.h"
#include "processor.h"
#include "storage.h"
#include "offset.h"
#include "fill_gaps.h"
#include "traversal.h"
#include "json_logger.h"

#ifdef WITH_IFC
#include <ifcparse/IfcFile.h>
#ifdef IFCOPENSHELL_05
#include <ifcgeom/IfcGeomIterator.h>
#else
#include <ifcgeom_schema_agnostic/IfcGeomIterator.h>
#endif

#include <boost/filesystem.hpp>

struct instance_filter_t {
	virtual bool operator()(const IfcUtil::IfcBaseEntity*) const = 0;
};

struct filtered_files_t {
	std::vector<IfcParse::IfcFile*> files;
	instance_filter_t* filter = nullptr;
};
#else
namespace IfcParse {
	class IfcFile {};
}

struct filtered_files_t {};
#endif

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <ProjLib.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

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

size_t get_padding();
void set_padding(size_t);

typedef boost::variant<boost::blank, filtered_files_t, geometry_collection_t*, abstract_voxel_storage*, function_arg_value_type> symbol_value;

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
	boost::optional<std::function<void(float)>> application_progress_callback;
	bool silent = false;

	virtual const std::vector<argument_spec>& arg_names() const = 0;
	virtual symbol_value invoke(const scope_map& scope) const = 0;
	virtual bool catch_all() const {
		return false;
	}
	virtual bool only_local() const {
		return false;
	}
	virtual ~voxel_operation() {}
};

namespace {
	json_logger::meta_data dump_info(abstract_voxel_storage* voxels) {
		if (dynamic_cast<abstract_chunked_voxel_storage*>(voxels)) {
			auto left = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->grid_offset();
			auto nc = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->num_chunks();
			auto right = (left + nc.as<long>()) - (decltype(left)::element_type)1;
			auto sz = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->voxel_size();
			auto szl = (long)dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->chunk_size();
			auto left_world = ((voxels->bounds()[0].as<long>() + left * szl).as<double>() * sz);
			auto right_world = ((voxels->bounds()[1].as<long>() + left * szl).as<double>() * sz);

			return {
				{"count", (long)voxels->count()},
				{"grid", left.format() + " - " + right.format()},
				{"bounds", voxels->bounds()[0].format() + " - " + voxels->bounds()[1].format()},
				{"world", left_world.format() + " - " + right_world.format()},
				{"bits", (long)voxels->value_bits()}
			};
		}
		return {};
	}
}

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

				auto projects = f->instances_by_type("IfcProject");
				// @nb: a copy has to be made, because instances_by_type() returns a reference
				//      from the live map of the file which is updated upon removeEntity()
				std::vector<IfcUtil::IfcBaseClass*> projects_copy(projects->begin(), projects->end());
				if (projects->size() > 1) {
					for (auto it = projects_copy.begin() + 1; it != projects_copy.end(); ++it) {
						auto inverses = f->getInverse((*it)->data().id(), nullptr, -1);
						
						f->removeEntity(*it);

						for (auto& inv : *inverses) {
							if (inv->declaration().name() == "IFCRELAGGREGATES") {
								auto attr = new IfcWrite::IfcWriteArgument;
								attr->set(projects_copy[0]);
								inv->data().setArgument(4, attr);
							}
						}
					}
				}

				files.push_back(f);
			}
		}
		
		if (files.empty()) {
			throw std::runtime_error("Not a single file matched pattern");
		}

		filtered_files_t ff;
		ff.files = files;

		return ff;
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
		// @todo what's this?
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

		const filtered_files_t& ifc_files = scope.get_value<filtered_files_t>("input");

		IfcGeom::IteratorSettings settings_surface;
		settings_surface.set(IfcGeom::IteratorSettings::DISABLE_TRIANGULATION, true);
		settings_surface.set(IfcGeom::IteratorSettings::USE_WORLD_COORDS, true);
		// Only to determine whether building element parts decompositions of slabs should be processed as roofs
		settings_surface.set(IfcGeom::IteratorSettings::SEARCH_FLOOR, true);
		

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
			
			std::vector<std::string> entities_without_quotes;
			std::transform(entities.begin(), entities.end(), std::back_inserter(entities_without_quotes), [](const std::string& v) {
				return v.substr(1, v.size() - 2);
			});

#ifdef IFCOPENSHELL_05
			std::transform(entities_without_quotes.begin(), entities_without_quotes.end(), std::inserter(ef_elements, ef_elements.begin()), [](const std::string& v) {
				return IfcSchema::Type::FromString(boost::to_upper_copy(v));
			});

			ef.traverse = ef_elements != {IfcSchema::Type::IfcSpace};
#else
			ef_elements.insert(entities_without_quotes.begin(), entities_without_quotes.end());

			// Normally we want decompositions to be included, so that a wall with IfcBuildingElement parts is processed including it's parts. For spaces we do not want that.
			static const std::set<std::string> ONLY_SPACES{ { "IfcSpace" } };
			ef.traverse = ef_elements != ONLY_SPACES;
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

		for (auto ifc_file : ifc_files.files) {

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
					IfcSchema::IfcSlabTypeEnum::IfcSlabTypeEnum pdt = IfcSchema::IfcSlabTypeEnum::NOTDEFINED;
					if (elem->product()->as<IfcSchema::IfcSlab>()->hasPredefinedType()) {
						pdt = elem->product()->as<IfcSchema::IfcSlab>()->PredefinedType();
					}
					process = process && (pdt == IfcSchema::IfcSlabTypeEnum::IfcSlabType_ROOF) == *roof_slabs;
				}
#else
				auto elem_product = elem->product();
				if (elem->product()->declaration().is("IfcBuildingElementPart")) {
					auto parents = elem->parents();
					if (parents.size()) {
						// Assume the first parent of a building element part is
						// the element that got it included by the hierarchical
						// processing of element filters.
						elem_product = parents.back()->product();
					}
				}
				if (roof_slabs && elem_product->declaration().is("IfcSlab")) {
					auto attr_value = elem_product->get("PredefinedType");
					std::string pdt = attr_value->isNull() ? std::string("") : (std::string)(*attr_value);
					process = process && (pdt == "ROOF") == *roof_slabs;
				}
#endif

				if (ifc_files.filter) {
					try {
						process = process && (*ifc_files.filter)(elem_product);
					} catch (...) {
#ifdef IFCOPENSHELL_05
						throw std::runtime_error("Error evaluating filter on " + elem_product->toString());
#else
						throw std::runtime_error("Error evaluating filter on " + elem_product->data().toString());
#endif
					}
				}

				if (process) {
					TopoDS_Compound compound = elem->geometry().as_compound();
					BRepMesh_IncrementalMesh(compound, 0.001);
					geometries->push_back(std::make_pair(elem->id(), compound));
				}

				if (old_progress != iterator.progress()) {
					old_progress = iterator.progress();
					if (application_progress_callback) {
						(*application_progress_callback)(old_progress / 100.f);
					}
				}

				if (!iterator.next()) {
					break;
				}
			}

		}		

		if (!at_least_one_succesful) {
			json_logger::message(json_logger::LOG_FATAL, "Failed to generate geometry");
			abort();
		}

		std::random_shuffle(geometries->begin(), geometries->end());

		return geometries;
	}
};
#endif

namespace {
	abstract_voxel_storage* voxelize_2(abstract_voxel_storage* voxels, geometry_collection_t* surfaces) {
		progress_writer progress;
		processor p(voxels, progress);
		p.process(surfaces->begin(), surfaces->end(), SURFACE(), output(MERGED()));
		return voxels;
	}

	template <typename V = bit_t>
	abstract_voxel_storage* voxelize(geometry_collection_t* surfaces, double vsize, int chunksize, const boost::optional<int>& threads, bool use_volume, bool silent = false) {
		double x1, y1, z1, x2, y2, z2;
		int nx, ny, nz;

		Bnd_Box global_bounds;
		for (auto& p : *surfaces) {
			BRepBndLib::Add(p.second, global_bounds);
		}

		global_bounds.Get(x1, y1, z1, x2, y2, z2);
		nx = (int)ceil((x2 - x1) / vsize);
		ny = (int)ceil((y2 - y1) / vsize);
		nz = (int)ceil((z2 - z1) / vsize);

		x1 -= vsize * get_padding();
		y1 -= vsize * get_padding();
		z1 -= vsize * get_padding();
		nx += get_padding() * 2;
		ny += get_padding() * 2;
		nz += get_padding() * 2;

		std::unique_ptr<fill_volume_t> method;
		if (use_volume) {
			method = std::make_unique<VOLUME>();
		} else {
			method = std::make_unique<SURFACE>();
		}

		if (std::is_same<V, voxel_uint32_t>::value) {
			chunked_voxel_storage<voxel_uint32_t>* storage = new chunked_voxel_storage<voxel_uint32_t>(x1, y1, z1, vsize, nx, ny, nz, chunksize);
			progress_writer progress("voxelize");
			processor pr(storage, progress);
			pr.use_scanline() = false;
			// @todo, uint32 defaults to VOLUME_PRODUCT_ID, make this explicit
			pr.process(surfaces->begin(), surfaces->end(), VOLUME_PRODUCT_ID(), output(MERGED()));
			return storage;
		} else {
			if (threads) {
				progress_writer progress("voxelize", silent);
				threaded_processor p(x1, y1, z1, vsize, nx, ny, nz, chunksize, *threads, progress);
				p.process(surfaces->begin(), surfaces->end(), *method, output(MERGED()));
				return p.voxels();
			} else {
				progress_writer progress;
				auto voxels = factory().chunk_size(chunksize).create(x1, y1, z1, vsize, nx, ny, nz);
				// nb: the other constructor would tell the constructor to delete the created voxels
				processor p(voxels, progress);
				p.process(surfaces->begin(), surfaces->end(), *method, output(MERGED()));
				return voxels;
			}
		}
	}
}

class op_voxelize : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "surfaceset" }, {false, "VOXELSIZE", "real"}, {false, "type", "string"}, {false, "method", "string"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		geometry_collection_t* surfaces = scope.get_value<geometry_collection_t*>("input");

		double vsize = scope.get_value<double>("VOXELSIZE");
		int cs = scope.get_value<int>("CHUNKSIZE");
		int t = scope.get_value<int>("THREADS");
		std::string ty = scope.get_value_or<std::string>("type", "bit");
		std::string method = scope.get_value_or<std::string>("method", "surface");

		if (method != "surface" && method != "volume") {
			throw std::runtime_error("Unsupported method " + method);
		}

		bool use_volume = method == "volume";

		if (surfaces->size() == 0) {
			// Just some arbitrary empty region
			if (ty == "bit") {
				return (abstract_voxel_storage*) new chunked_voxel_storage<bit_t>(make_vec<long>(0, 0, 0), vsize, cs, make_vec<size_t>(1U, 1U, 1U));
			} else {
				return (abstract_voxel_storage*) new chunked_voxel_storage<voxel_uint32_t>(make_vec<long>(0, 0, 0), vsize, cs, make_vec<size_t>(1U, 1U, 1U));
			}
		} else {
			if (ty == "bit") {
				return voxelize<bit_t>(surfaces, vsize, cs, t, use_volume, silent);
			} else {
				return voxelize<voxel_uint32_t>(surfaces, vsize, cs, t, use_volume, silent);
			}
		}
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

class op_describe_components : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "output_path", "string" }, { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		const std::string output_path = scope.get_value<std::string>("output_path");
		std::ofstream ofs(output_path.c_str());
		ofs << "[";
		bool first = true;
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		connected_components((regular_voxel_storage*)voxels, [&ofs, &first](regular_voxel_storage* c) {
			if (!first) {
				ofs << ",";
			}
			auto info = dump_info(c);
			ofs << json_logger::to_json_string(info);
			first = false;
		});
		ofs << "]";
		symbol_value v;
		return v;
	}
};

class op_describe_group_by : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "output_path", "string" }, { true, "input", "voxels" }, { true, "groups", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		const std::string output_path = scope.get_value<std::string>("output_path");
		std::ofstream ofs(output_path.c_str());
		ofs << "[";
		bool first = true;
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		
		auto groups = (regular_voxel_storage*) scope.get_value<abstract_voxel_storage*>("groups");

		if (groups->value_bits() != 32) {
			throw std::runtime_error("Expected a uint stored dataset for groups");
		}

		uint32_t v;
		std::set<uint32_t> vs;
		for (auto& ijk : *groups) {
			groups->Get(ijk, &v);
			vs.insert(v);
		}

		static bit_t desc_bits;

		for (auto& id : vs) {
			if (!first) {
				ofs << ",";
			}

			// A {0,1} dataset of `groups`==`id`
			auto where_id = groups->empty_copy_as(&desc_bits);
			for (auto& ijk : *groups) {
				groups->Get(ijk, &v);
				if (v == id) {
					where_id->Set(ijk);
				}
			}

			auto c = voxels->boolean_intersection(where_id);

			auto info = dump_info(c);
			info["id"] = static_cast<long>(id);
			ofs << json_logger::to_json_string(info);

			delete c;
			delete where_id;

			first = false;
		}

		ofs << "]";

		symbol_value v_null;
		return v_null;
	}
};

class op_keep_components : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "min_size", "integer" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		auto result = voxels->empty_copy();
		size_t min_size = (size_t) scope.get_value<int>("min_size");

		connected_components((regular_voxel_storage*)voxels, [&result, &min_size](regular_voxel_storage* c) {
			if (c->count() >= min_size) {
				result->boolean_union_inplace(c);
			}
		});

		return result;
	}
};

namespace {
	template <typename Fn, typename Fn2>
	void revoxelize_and_check_overlap(abstract_voxel_storage* voxels, const geometry_collection_t& surfaces, Fn fn, Fn2 fn2) {
		BRep_Builder B;

		TopoDS_Compound face_subset_all_elem;
		B.MakeCompound(face_subset_all_elem);

		for (auto& pair : surfaces) {
			TopoDS_Compound face_subset;
			B.MakeCompound(face_subset);

			bool any = false;

			TopExp_Explorer exp(pair.second, TopAbs_FACE);

			for (; exp.More(); exp.Next()) {
				TopoDS_Compound C;
				B.MakeCompound(C);
				B.Add(C, exp.Current());
				geometry_collection_t single = { { pair.first, C} };

				auto vs = voxels->empty_copy();
				voxelize_2(vs, &single);
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
				fn(pair.first, face_subset);
			}
		}

		fn2(face_subset_all_elem);
	}

}

namespace {
	class json_visitor {
	public:
		typedef std::string result_type;

		std::string operator()(const boost::blank&) const {
			return "null";
		}

		std::string operator()(int v) const {
			return boost::lexical_cast<std::string>(v);
		}

		std::string operator()(double v) const {
			return boost::lexical_cast<std::string>(v);
		}

		std::string operator()(const std::string& v) const {
			// assume properly escaped.
			return "\"" + v + "\"";
		}

		std::string operator()(const function_arg_value_type& v) const {
			return v.apply_visitor(*this);
		}

		std::string operator()(filtered_files_t const v) const {
#ifdef WITH_IFC
			return "{\"num_files\":" + boost::lexical_cast<std::string>(v.files.size()) + "}";
#else
			return "{}";
#endif
		}

		std::string operator()(geometry_collection_t* const v) const {
			return "{\"num_geometries\":" + boost::lexical_cast<std::string>(v->size()) + "}";
		}

		std::string operator()(abstract_voxel_storage* const v) const {
			return "{\"count\":" + boost::lexical_cast<std::string>(v->count()) + "}";
		}
	};
}

class op_json_stats : public voxel_operation {
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "output_path", "string" }, { false, "variables", "sequence"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {		
		const std::string output_path = scope.get_value<std::string>("output_path");
		std::vector<std::string> vars;
		if (scope.has("variables")) {
			vars = scope.get_value<std::vector<std::string> >("variables");
		} else {
			for (auto& p : scope) {
				vars.push_back(p.first);
			}
		}

		std::ofstream json(output_path.c_str());
		json << "{";

		int n = 0;

		for (auto& k : vars) {
			auto it = scope.find(k);
			if (it != scope.end()) {
				if (n) {
					json << ",";
				}
				json_visitor v;
				json << "\"" << k << "\":" << it->second.apply_visitor(v);
				++n;
			}			
		}

		json << "}";

		symbol_value v;
		return v;
	}
};


#ifdef WITH_IFC

class op_export_elements : public voxel_operation {
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "ifcfile" }, { true, "input_voxels", "voxels" }, { true, "input_surfaces", "surfaceset" }, { true, "output_path", "string" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		const filtered_files_t& ifc_files = scope.get_value<filtered_files_t>("input");
		if (ifc_files.files.size() != 1) {
			throw std::runtime_error("Only single file inputs supported for this operation");
		}
		auto f0 = ifc_files.files[0];

		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input_voxels");
		geometry_collection_t* surfaces = scope.get_value<geometry_collection_t*>("input_surfaces");
		const std::string output_path = scope.get_value<std::string>("output_path");

		std::ofstream json(output_path.c_str());
		json << "[\n";
		int n = 0;

		revoxelize_and_check_overlap(voxels, *surfaces, [&f0, &json, &n](int iden, const TopoDS_Compound& face_subset) {
			TopExp_Explorer exp(face_subset, TopAbs_FACE);
			int num_faces = 0;
			for (; exp.More(); exp.Next()) {
				num_faces++;
			}

			// @todo arbitary value alert
			if (num_faces > 3) {
				std::string guid = *((IfcUtil::IfcBaseEntity*)f0->instance_by_id(iden))->get("GlobalId");
				if (n) {
					json << ",\n";
				}
				n += 1;
				json << "{\"id\":" << iden << ",\"guid\":\"" << guid << "\"}";
			}
		}, [&output_path](const TopoDS_Compound& face_subset_all_elem) {
		});

		json << "\n]";

		symbol_value v;
		return v;
	}
};

class op_export_ifc : public voxel_operation  {
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "ifcfile" }, { true, "input_voxels", "voxels" }, { true, "input_surfaces", "surfaceset" }, { true, "output_path", "string" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		const filtered_files_t& ifc_files = scope.get_value<filtered_files_t>("input");
		if (ifc_files.files.size() != 1) {
			throw std::runtime_error("Only single file inputs supported for this operation");
		}
		auto f0 = ifc_files.files[0];
#ifndef IFCOPENSHELL_05
#endif
		IfcParse::IfcFile new_file(f0->schema());
		auto ps = f0->instances_by_type("IfcProject");
		for (auto& p : *ps) {
			new_file.addEntity(p);
		}
		
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input_voxels");
		geometry_collection_t* surfaces = scope.get_value<geometry_collection_t*>("input_surfaces");
		const std::string output_path = scope.get_value<std::string>("output_path");

		revoxelize_and_check_overlap(voxels, *surfaces, [&f0, &new_file](int iden, const TopoDS_Compound& face_subset) {
			TopExp_Explorer exp(face_subset, TopAbs_FACE);
			int num_faces = 0;
			for (; exp.More(); exp.Next()) {
				num_faces++;
			}

			// @todo arbitary value alert
			if (num_faces >= 1) {
				auto inst = f0->instance_by_id(iden);
				new_file.addEntity(inst);
				auto refs = f0->traverse(inst);
				for (auto& r : *refs) {
					if (r->declaration().is("IfcRepresentationItem")) {
						auto styles = ((IfcUtil::IfcBaseEntity*)r)->get_inverse("StyledByItem");
						for (auto& s : *styles) {
							new_file.addEntity(s);
						}
					}
				}
				auto asses = ((IfcUtil::IfcBaseEntity*) inst)->get_inverse("HasAssociations");
				for (auto& rel : *asses) {
					auto relrefs = f0->traverse(rel);
					for (auto& r : *relrefs) {
						if (r->declaration().is("IfcMaterial")) {
							new_file.addEntity(rel);
							auto styles = ((IfcUtil::IfcBaseEntity*)r)->get_inverse("HasRepresentation");
							for (auto& s : *styles) {
								new_file.addEntity(s);
							}
						}
					}
				}
				auto ops = ((IfcUtil::IfcBaseEntity*) inst)->get_inverse("HasOpenings");
				for (auto& rel : *ops) {
					new_file.addEntity(rel);
				}
			}
		}, [&output_path](const TopoDS_Compound& face_subset_all_elem) {
		});

		{
			std::ofstream fs(output_path.c_str());
			fs << new_file;
		}

		symbol_value v;
		return v;
	}
};

#endif

class op_dump_surfaces : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input_voxels", "voxels" }, { true, "input_surfaces", "surfaceset" }, { true, "output_path", "string" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input_voxels");
		geometry_collection_t* surfaces = scope.get_value<geometry_collection_t*>("input_surfaces");
		const std::string output_path = scope.get_value<std::string>("output_path");

		revoxelize_and_check_overlap(voxels, *surfaces, [&output_path](int iden, const TopoDS_Compound& face_subset) {
			std::string fn = output_path + DIRSEP + std::to_string(iden) + ".brep";
			std::ofstream fs(fn.c_str());
			BRepTools::Write(face_subset, fs);
		}, [&output_path](const TopoDS_Compound& face_subset_all_elem) {
			std::string fn = output_path + DIRSEP + "all.brep";
			std::ofstream fs(fn.c_str());
			BRepTools::Write(face_subset_all_elem, fs);
		});		

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
		threaded_post_process<fill_gaps> tpp(scope.get_value<int>("THREADS"));
		tpp.set_progress_callback(application_progress_callback);
		tpp.silent = silent;
		return tpp((regular_voxel_storage*) voxels);
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
		threaded_post_process< offset<> > tpp(scope.get_value<int>("THREADS"));
		tpp.set_progress_callback(application_progress_callback);
		tpp.silent = silent;
		return tpp((regular_voxel_storage*)voxels);
	}
};

class op_zeros : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		// @todo also implement empty_copy_as() here.
		return voxels->empty_copy();
	}
};

class op_plane : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, {true, "a", "real"}, {true, "b", "real"}, {true, "c", "real"}, {true, "d", "real"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* v = scope.get_value<abstract_voxel_storage*>("input");
		
		auto voxels = dynamic_cast<abstract_chunked_voxel_storage*>(v);
		if (voxels == nullptr) {
			throw std::runtime_error("expected chunked storage");
		}

		auto result = voxels->empty_copy();

		if (voxels->count() == 0) {
			return result;
		}
		
		gp_Pln pln(
			scope.get_value<double>("a"),
			scope.get_value<double>("b"),
			scope.get_value<double>("c"),
			scope.get_value<double>("d")
		);
		
		auto left = voxels->grid_offset();
		auto sz = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->voxel_size();
		auto szl = (long)dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->chunk_size();
		auto left_world = ((voxels->bounds()[0].as<long>() + left * szl).as<double>() * sz);
		auto right_world = ((voxels->bounds()[1].as<long>() + left * szl).as<double>() * sz);

		BRepPrimAPI_MakeBox mb(gp_Pnt(
			left_world.get<0>(),
			left_world.get<1>(),
			left_world.get<2>()
		), gp_Pnt(
			right_world.get<0>(),
			right_world.get<1>(),
			right_world.get<2>()
		));

		auto box = mb.Solid();
		static double inf = std::numeric_limits<double>::infinity();
		std::array<std::array<double, 2>, 2> uv_min_max = {{ {{+inf, +inf}}, {{-inf,-inf}} }};

		TopExp_Explorer exp(box, TopAbs_VERTEX);
		for (; exp.More(); exp.Next()) {
			auto p = BRep_Tool::Pnt(TopoDS::Vertex(exp.Current()));
			auto p2d = ProjLib::Project(pln, p);
			for (int i = 0; i < 2; ++i) {
				auto v = p2d.ChangeCoord().ChangeCoord(i + 1);
				auto& min_v = uv_min_max[0][i];
				auto& max_v = uv_min_max[1][i];
				if (v < min_v) min_v = v;
				if (v > max_v) max_v = v;
			}
		}

		auto face = BRepBuilderAPI_MakeFace(pln,
			uv_min_max[0][0], uv_min_max[1][0],
			uv_min_max[0][1], uv_min_max[1][1]
		);

		auto face_inside = BRepAlgoAPI_Common(face, box).Shape();
		TopoDS_Compound C;
		if (face_inside.ShapeType() == TopAbs_COMPOUND) {
			C = TopoDS::Compound(face_inside);
		} else {
			BRep_Builder B;
			B.MakeCompound(C);
			B.Add(C, face_inside);
		}

		BRepMesh_IncrementalMesh(C, 0.001);
		
		geometry_collection_t* single = new geometry_collection_t{ { 0, C} };
		return single;
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
			threaded_post_process< offset<1, 2> > tpp(scope.get_value<int>("THREADS"));
			// @todo progress not propagated as it's within a loop
			tpp.silent = silent;
			auto ofs = tpp(current);
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
		threaded_post_process<keep_outmost> tpp(1);
		tpp.set_progress_callback(application_progress_callback);
		tpp.silent = silent;
		return tpp((regular_voxel_storage*)voxels);
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
		threaded_post_process<traversal_voxel_filler_separate_components> tpp(1);
		tpp.set_progress_callback(application_progress_callback);
		tpp.silent = silent;
		return tpp((regular_voxel_storage*)voxels);
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
		if (voxels->count() == 0) {
			return voxels->empty_copy();
		}
		threaded_post_process<traversal_voxel_filler_inverse> tpp(1);
		tpp.set_progress_callback(application_progress_callback);
		tpp.silent = silent;
		return ((regular_voxel_storage*)voxels);
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
		threaded_post_process<traversal_voxel_filler_inverted> tpp(1);
		tpp.set_progress_callback(application_progress_callback);
		tpp.silent = silent;
		return tpp((regular_voxel_storage*)voxels);
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
class op_collapse_count : public op_geom<collapse_count> {};

template <typename Pred>
class op_compare : public voxel_operation {
	const std::vector<argument_spec>& arg_names() const {
		// @todo this needs to be expanded to other types for rhs eventually
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "rhs", "integer"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		auto* voxels = (regular_voxel_storage*) scope.get_value<abstract_voxel_storage*>("input");
		auto rhs = (uint32_t)scope.get_value<int>("rhs");

		auto result = voxels->empty_copy();
		uint32_t val;

		for (auto& pos : *voxels) {
			voxels->Get(pos, &val);
			if (Pred()(val, rhs)) {
				result->Set(pos, &val);
			}
		}

		return result;
	}
};

class op_greater : public op_compare<std::greater<size_t>> {};
class op_less : public op_compare<std::less<size_t>> {};

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
		
		visitor<6> v6;
		visitor<26> v26;
		v6.max_depth = max_depth;
		v26.max_depth = max_depth;

		if (C == 6) {	
			run_visitor = std::ref(v6);
		} else {
			run_visitor = std::ref(v26);
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

#ifdef WITH_IFC

namespace {
	class instance_by_attribute_map_filter : public instance_filter_t {
	public:
		typedef std::vector<std::pair<std::string, std::string> > values_t;
	private:
		values_t attr_pattern_;
	public:
		instance_by_attribute_map_filter(const values_t& pattern)
			: attr_pattern_(pattern)
		{}

		bool operator()(const IfcUtil::IfcBaseEntity* inst) const {
#ifdef IFCOPENSHELL_05
			throw std::runtime_error("Not implemented");
#else
			for (auto& p : attr_pattern_) {
				auto idx = inst->declaration().as_entity()->attribute_index(p.first);
				if (idx == -1) {
					throw std::runtime_error(inst->declaration().name() + " has no attribute " + p.first);
				}
				auto attr = inst->data().getArgument(idx);
				if (!attr || attr->isNull()) {
					return false;
				}
				if (attr->type() == IfcUtil::Argument_DOUBLE) {
					auto op = p.second.at(0);
					auto v = p.second.substr(1);
					auto attr_type = inst->declaration().attribute_by_index(idx);
					double d0 = *attr;
					if (attr_type->type_of_attribute()->is("IfcLengthMeasure")) {
						d0 *= inst->data().file->getUnit("LENGTHUNIT").second;
					}
					auto d1 = boost::lexical_cast<double>(v);
					if (op == '<') {
						if (!(d0 < d1)) {
							return false;
						}
					} else if (op == '>') {
						if (!(d0 > d1)) {
							return false;
						}
					} else if (op == '=') {
						if (!(d0 == d1)) {
							return false;
						}
					} else {
						throw std::runtime_error("Invalid binary predicate " + p.second);
					}
				} else {
					throw std::runtime_error("Not implemented");
				}
			}

			return true;
#endif			
		}
	};
}

class op_create_attr_filter : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "ifcfile" } };
		return nm_;
	}
	bool catch_all() const {
		return true;
	}
	bool only_local() const {
		return true;
	}
	symbol_value invoke(const scope_map& scope) const {
		static std::set<std::string> to_exclude{ "input" };
		filtered_files_t ifc_files_copy = scope.get_value<filtered_files_t>("input");
		instance_by_attribute_map_filter::values_t vs;
		for (auto& p : scope) {
			if (to_exclude.find(p.first) == to_exclude.end()) {
				auto s = boost::get<std::string>(boost::get<function_arg_value_type>(p.second));
				vs.push_back({ p.first, s });
			}
		}
		ifc_files_copy.filter = new instance_by_attribute_map_filter(vs);
		return ifc_files_copy;
	}
};

class op_create_prop_filter : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "ifcfile" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		throw std::runtime_error("Not implemented");
	}
};

#endif

class op_mesh : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "filename", "string"}, {false, "use_value", "integer"}, {false, "with_components", "integer"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		auto voxels = scope.get_value<abstract_voxel_storage*>("input");
		auto use_value = scope.get_value_or<int>("use_value", -1);
		auto filename = scope.get_value<std::string>("filename");
		std::ofstream ofs(filename.c_str());
		if (voxels->value_bits() == 1) {
			auto with_components = scope.get_value_or<int>("with_components", -1);
			((regular_voxel_storage*)voxels)->obj_export(ofs, with_components != 0);
		} else {
			((regular_voxel_storage*)voxels)->obj_export(ofs, use_value != 1, use_value == 1);
		}
		symbol_value v;
		return v;
	}
};

template <typename T>
T* instantiate() {
	return new T();
}

class symbol_value_assignment_visitor {
private:
	bool all_;
	symbol_value* v_;
	std::set<std::string> type_;
	const scope_map& context_;

	void assert_type(const std::string& t) const {
		if (all_) {
			return;
		} else if (type_.find(t) == type_.end()) {
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
		: all_(false)
		, v_(v)
		, context_(context)
	{
		if (type == "*") {
			all_ = true;
		} else {
			boost::tokenizer<> t(type);
			for (auto& v : t) {
				type_.insert(v);
			}
		}
	}

void operator()(filtered_files_t const v) const {
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
	boost::optional<std::function<void(float)>> application_progress_callback;
	bool silent = false;

	voxel_operation_runner(const statement_type& statement) :
		statement_(statement) {}

	symbol_value run(const statement_type& st, scope_map& context) {
		voxel_operation* op = voxel_operation_map::create(statement_.call().name());
		op->application_progress_callback = application_progress_callback;
		op->silent = silent;

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

		for (auto& arg : statement_.call().args()) {
			if (arg.has_keyword() && function_value_symbols.find(*arg.keyword()) == function_value_symbols.end()) {
				if (op->catch_all()) {
					symbol_value_assignment_visitor v(&function_value_symbols[*arg.keyword()], "*", context);
					function_values[*arg.keyword()].apply_visitor(v);
				} else {
					throw std::runtime_error("Keyword argument " + *arg.keyword() + " unused");
				}
			}
		}

		if (!op->only_local()) {
			for (auto& p : context) {
				if (function_value_symbols.find(p.first) == function_value_symbols.end()) {
					function_value_symbols.insert(p);
				}
			}
		}

		auto r = op->invoke(function_value_symbols);

		delete op;

		return r;
	}
};

scope_map run(const std::vector<statement_type>& statements, double size, size_t threads = 0, size_t chunk_size = 128, bool with_mesh = false, bool with_progress_on_cout = false);

#endif
