#ifndef VOXEC_H
#define VOXEC_H

#ifdef WITH_IFC
#ifdef IFCOPENSHELL_05
#include <ifcgeom/IfcGeomIterator.h>
#else
#include <ifcgeom_schema_agnostic/IfcGeomIterator.h>
#endif

#include <ifcparse/IfcFile.h>
#include <boost/filesystem.hpp>

#include <Eigen/Dense>

// #define OLD_GROUP_BY

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

#include "voxelfile.h"
#include "processor.h"
#include "storage.h"
#include "offset.h"
#include "fill_gaps.h"
#include "traversal.h"
#include "json_logger.h"

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
#include <BRepPrimAPI_MakeHalfSpace.hxx>

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

class assertion_error : public std::runtime_error {
	using std::runtime_error::runtime_error;
};

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
	const std::map<std::string, function_def_type>* functions;

	class not_in_scope : public std::runtime_error {
		using std::runtime_error::runtime_error;
	};

	class value_error : public std::runtime_error {
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
			throw not_in_scope("Undefined variable " + symbol);
		}
		try {
			return get_value_<T>(it->second);
		} catch (boost::bad_get&) {
			throw value_error(std::string("Expected ") + typeid(T).name() + " got type index " + std::to_string(it->second.which()));
		}
	}

	const int get_length(const std::string& symbol) const {
		auto it = find(symbol);
		if (it == end()) {
			throw not_in_scope("Undefined variable " + symbol);
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

void invoke_function_by_name(scope_map& context, const std::string& function_name);

namespace {
	json_logger::meta_data dump_info(abstract_voxel_storage* voxels) {
		if (dynamic_cast<abstract_chunked_voxel_storage*>(voxels)) {
			auto csize = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->chunk_size();
			auto left = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->grid_offset();
			auto nc = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->num_chunks();
			auto right = (left + nc.as<long>()) - (decltype(left)::element_type)1;
			auto sz = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->voxel_size();
			auto szl = (long)dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->chunk_size();
			auto left_world = ((voxels->bounds()[0].as<long>() + left * szl).as<double>() * sz);
			auto right_world = ((voxels->bounds()[1].as<long>() + left * szl).as<double>() * sz);

			json_logger::meta_data md = {
				{"count", (long)voxels->count()},
				{"grid", left.format() + " - " + right.format()},
				{"bounds", voxels->bounds()[0].format() + " - " + voxels->bounds()[1].format()},
				{"world", left_world.format() + " - " + right_world.format()},
				{"bits", (long)voxels->value_bits()},
				{"chunk_size", (long) csize}
			};

			if (voxels->value_bits() == 32) {
				uint32_t v, mi = std::numeric_limits<uint32_t>::max(), ma = std::numeric_limits<uint32_t>::min();
				for (auto& ijk : *(regular_voxel_storage*)voxels) {
					voxels->Get(ijk, &v);
					if (v < mi) {
						mi = v;
					}
					if (v > ma) {
						ma = v;
					}
				}
				md.insert({ "min_value", (long)mi });
				md.insert({ "max_value", (long)ma });
			}

			return md;
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
				
				if (projects) {
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

#ifdef WITH_IFC
#ifdef IFCOPENSHELL_07
typedef IfcGeom::Iterator iterator_t;
typedef aggregate_of_instance instance_list_t;
#else
typedef IfcGeom::Iterator<double> iterator_t;
typedef IfcEntityList instance_list_t;
#endif
#endif

class op_create_geometry : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "ifcfile" }, { false, "include", "sequence"}, { false, "exclude", "sequence"}, { false, "optional", "integer"}, { false, "only_transparent", "integer"}, { false, "only_opaque", "integer"} };
		return nm_;
	}
		
	symbol_value invoke(const scope_map& scope) const {

		IfcGeom::entity_filter ef;

		boost::optional<size_t> threads;
		if (scope.has("THREADS")) {
			int t = scope.get_value<int>("THREADS");
			if (t > 1) {
				threads = (size_t)t;
			}
		}

		bool only_transparent = scope.get_value_or<int>("only_transparent", 0) == 1;
		bool only_opaque = scope.get_value_or<int>("only_opaque", 0) == 1;

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
		// settings_surface.set(IfcGeom::IteratorSettings::USE_WORLD_COORDS, true);
		// Only to determine whether building element parts decompositions of slabs should be processed as roofs
#ifdef IFCOPENSHELL_07
		settings_surface.set(IfcGeom::IteratorSettings::ELEMENT_HIERARCHY, true);
#else
		settings_surface.set(IfcGeom::IteratorSettings::SEARCH_FLOOR, true);
#endif
		
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

			std::unique_ptr<iterator_t> iterator;

#ifdef IFCOPENSHELL_05
			iterator.reset(iterator_t(settings_surface, ifc_file, filters_surface));
#else
			if (threads) {
				iterator.reset(new iterator_t(settings_surface, ifc_file, filters_surface, *threads));
			} else {
				iterator.reset(new iterator_t(settings_surface, ifc_file, filters_surface));
			}
#endif
			
			
			// For debugging geometry creation from IfcOpenShell
			// Logger::SetOutput(&std::cout, &std::cout);

			if (!iterator->initialize()) {
				continue;
			}

			at_least_one_succesful = true;

			int old_progress = -1;

			for (;;) {
				elem_t* elem = (elem_t*)iterator->get();
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
					
					bool filtered_non_empty = true;
					if (only_transparent || only_opaque) {
						filtered_non_empty = false;
						TopoDS_Compound filtered;
						BRep_Builder B;
						B.MakeCompound(filtered);

						auto it = elem->geometry().begin();
						for (TopoDS_Iterator jt(compound); jt.More(); ++it, jt.Next()) {
							bool is_transparent = it->hasStyle() && it->Style().Transparency().get_value_or(0.0) > 1.e-9;
							if (only_transparent == is_transparent) {
								B.Add(filtered, jt.Value());
								filtered_non_empty = true;
							}
						}

						std::swap(compound, filtered);
					}

					if (filtered_non_empty) {
						compound.Move(elem->transformation().data());

						BRepMesh_IncrementalMesh(compound, 0.001);
						geometries->push_back(std::make_pair(std::pair<void*, int>(ifc_file, elem->id()), compound));
					}
				}

				if (old_progress != iterator->progress()) {
					old_progress = iterator->progress();
					if (application_progress_callback) {
						(*application_progress_callback)(old_progress / 100.f);
					}
				}

				if (!iterator->next()) {
					break;
				}
			}

		}		

		if (scope.get_value_or<int>("optional", 0) == 0 && !at_least_one_succesful) {
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

		if (global_bounds.IsVoid()) {
			return (abstract_voxel_storage*) new chunked_voxel_storage<V>(make_vec<long>(0, 0, 0), vsize, chunksize, make_vec<size_t>(1U, 1U, 1U));
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
			
			// @todo, uint32 defaults to VOLUME_PRODUCT_ID, make this explicit
			// @why
			// pr.use_scanline() = false;

			chunked_voxel_storage<voxel_uint32_t>* storage = new chunked_voxel_storage<voxel_uint32_t>(x1, y1, z1, vsize, nx, ny, nz, chunksize);

			if (threads && *threads != 1) {
				progress_writer progress("voxelize", silent);
				threaded_processor p(storage, *threads, progress);
				p.process(surfaces->begin(), surfaces->end(), VOLUME_PRODUCT_ID(), output(MERGED()));
				// @todo what actually happens to storage?
				delete storage;
				return p.voxels();
			} else {
				progress_writer progress("voxelize");
				processor pr(storage, progress);
				pr.process(surfaces->begin(), surfaces->end(), VOLUME_PRODUCT_ID(), output(MERGED()));
				return storage;
			}			

		} else {
			if (threads && *threads != 1) {
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


class op_count_components : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		int cnt = 0;
		connected_components((regular_voxel_storage*)voxels, [&cnt](regular_voxel_storage* c) {
			++cnt;
		});
		symbol_value v = cnt;
		return v;
	}
};

class op_print_values : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		auto voxels = (regular_voxel_storage*) scope.get_value<abstract_voxel_storage*>("input");
		uint32_t v;
		std::set<uint32_t> vs;
		for (auto& ijk : *voxels) {
			voxels->Get(ijk, &v);
			vs.insert(v);
		}
		bool first = true;
		for (auto& x : vs) {
			if (!first) {
				std::cout << " ";
			}
			first = false;
			std::cout << x;
		}
		symbol_value vv;
		return vv;
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

#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>
#include <thread>
#include <mutex>

namespace {
	template <typename T>
	class mt_list : public std::list<T> {
		std::mutex m;
	public:
		void push_back(const T& t) {
			std::lock_guard<std::mutex> lock{ m };
			std::list<T>::push_back(t);
		}
	};

#ifdef OLD_GROUP_BY
	template <typename Fn>
	void group_by(regular_voxel_storage* groups, abstract_voxel_storage* voxels, Fn fn, int threads=1) {

		uint32_t v;
		std::set<uint32_t> vs;
		for (auto& ijk : *groups) {
			groups->Get(ijk, &v);
			vs.insert(v);
		}

		bit_t desc_bits;

		mt_list<std::pair<uint32_t, abstract_voxel_storage*>> results;
		boost::asio::thread_pool pool(threads); // 4 threads
		
		auto process = [&fn, &groups, &desc_bits, &voxels, &results, &threads](uint32_t id) {
			uint32_t vv;
			// A {0,1} dataset of `groups`==`id`
			auto where_id = groups->empty_copy_as(&desc_bits);
			for (auto& ijk : *groups) {
				groups->Get(ijk, &vv);
				if (vv == id) {
					where_id->Set(ijk);
				}
			}

			auto c = voxels->boolean_intersection(where_id);

			delete where_id;

			if (threads <= 1) {
				fn(id, c);
			} else {
				results.push_back({ id, c });
			}
		};

		for (auto& id : vs) {
			if (threads > 1) {
				boost::asio::post(pool, std::bind(process, id));
			} else {
				process(id);
			}
		}

		if (threads > 1) {
			
			pool.join();
			for (auto& r : results) {
				fn(r.first, r.second);
			}
		}
	}
#else
	template <typename Fn>
	void group_by(regular_voxel_storage* groups, abstract_voxel_storage* voxels, Fn fn, std::map<uint32_t, size_t>& counts, bool use_bits=true, bool conserve_memory=true, bool only_counts=false) {

		uint32_t v;
		
		// conserve_memory=false
		std::map<uint32_t, abstract_voxel_storage*> map;
		
		// conserve_memory=true
		uint32_t target;
		std::set<uint32_t> vs;
		std::set<uint32_t>::const_iterator vs_it;
		abstract_voxel_storage* single;

		{
			auto acvs_voxels = dynamic_cast<abstract_chunked_voxel_storage*>(voxels);
			auto acvs_groups = dynamic_cast<abstract_chunked_voxel_storage*>(groups);

			if (!acvs_voxels || !acvs_groups) {
				throw std::runtime_error("Group operations are not supported on non-chunked storage");
			}

			if (!(acvs_voxels->grid_offset() == acvs_groups->grid_offset()).all()) {
				throw std::runtime_error("Group operations on unaligned voxel grids are not supported");
			}
		}

		if (conserve_memory) {
			for (auto& ijk : *(regular_voxel_storage*)voxels) {
				groups->Get(ijk, &v);
				vs.insert(v);
			}
			vs_it = vs.begin();
		} 
		
	repeat:

		if (conserve_memory) {
			if (vs_it == vs.end()) {
				return;
			}

			target = *vs_it;

			if (use_bits) {
				static bit_t fmt;
				single = voxels->empty_copy_as(&fmt);
			} else {
				single = voxels->empty_copy();
			}
		}

		// @todo use regions for multi threading
		for (auto& ijk : *(regular_voxel_storage*)voxels) {
			groups->Get(ijk, &v);

			if (conserve_memory) {
				if (v != target) {
					continue;
				}
			} else {
				if (v == 0) {
					continue;
				}
			}

			if (only_counts) {
				counts[v] ++;
				continue;
			}

			abstract_voxel_storage* r;

			if (conserve_memory) {
				r = single;
			} else {
				auto it = map.find(v);
				if (it == map.end()) {
					if (use_bits) {
						static bit_t fmt;
						map.insert({ v, r = voxels->empty_copy_as(&fmt) });
					} else {
						map.insert({ v, r = voxels->empty_copy() });
					}
				} else {
					r = it->second;
				}
			}

			if (use_bits) {
				r->Set(ijk);
			} else {
				voxels->Get(ijk, &v);
				r->Set(ijk, &v);
			}
		}

		if (conserve_memory) {
			fn(target, single);
			delete single;
			vs_it++;
			goto repeat;
		}

		for (auto& r : map) {
			fn(r.first, r.second);

			// @todo double check, untested
			delete r.second;
		}
	}
#endif
}

class op_describe_group_by : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "output_path", "string" }, { true, "input", "voxels" }, { true, "groups", "voxels" }, { false, "use_bits", "integer" }, { false, "conserve_memory", "integer" }, { false, "only_counts", "integer" } };
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

		bool use_bits = scope.get_value_or<int>("use_bits", 1) == 1;
		bool conserve_memory = scope.get_value_or<int>("conserve_memory", 0) == 1;
		bool only_counts = scope.get_value_or<int>("only_counts", 0) == 1;

		std::map<uint32_t, size_t> counts;

		group_by(groups, voxels, [&ofs, &first](uint32_t id, abstract_voxel_storage* c) {
			if (!first) {
				ofs << ",";
			}
			auto info = dump_info(c);
			info["id"] = static_cast<long>(id);
			ofs << json_logger::to_json_string(info);
			first = false;
			// @nb `c` is deleted automatically after this function body exits
			// @todo use unique_ptr&&
		}
#ifdef OLD_GROUP_BY
		,scope.get_value_or<int>("THREADS", 1)
#else
		, counts
		, use_bits
		, conserve_memory
		, only_counts
#endif
		);

		if (only_counts) {
			for (auto& p : counts) {
				if (!first) {
					ofs << ",";
				}
				json_logger::meta_data info = {
					{"id", (long)p.first},
					{"count", (long)p.second}
				};
				ofs << json_logger::to_json_string(info);
				first = false;
			}
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
	void revoxelize_and_check_overlap(abstract_voxel_storage* voxels, const geometry_collection_t& surfaces, bool indiv_face, int factor, Fn fn, Fn2 fn2) {
		BRep_Builder B;

		TopoDS_Compound face_subset_all_elem;
		B.MakeCompound(face_subset_all_elem);

		for (auto& pair : surfaces) {
			TopoDS_Compound face_subset;
			B.MakeCompound(face_subset);

			bool any = false;

			int face_idx = 0;

			std::list<TopoDS_Compound> items;

			if (indiv_face) {
				TopExp_Explorer exp(pair.second, TopAbs_FACE);
				for (; exp.More(); exp.Next(), ++face_idx) {
					TopoDS_Compound C;
					B.MakeCompound(C);
					B.Add(C, exp.Current());
					items.emplace_back(C);
				}
			} else {
				items.push_back(pair.second);
			}

			for (auto& item : items) {
				geometry_collection_t single = { { pair.first, item} };

				auto vs = voxels->empty_copy();
				voxelize_2(vs, &single);
				auto x = vs->boolean_intersection(voxels);

				json_logger::message(
					json_logger::LOG_DEBUG,
					std::string("#") + std::to_string(pair.first.second) + ": " + std::to_string(x->count()) + "/" + std::to_string(vs->count())
				);

				// std::cout << "#" << pair.first << std::endl;
				// std::cout << "vs ";
				// dump_info(vs);
				// std::cout << "x  ";
				// dump_info(x);

				if (vs->count() > 0 && x->count() * factor >= vs->count()) {
					// @todo I thought that a valid non-null face would result in at least once voxel.
					//       there might be differences here with the scanline algorithm and box intersect
					//       algorithm.
					if (indiv_face) {
						auto face = TopoDS_Iterator(item).Value();
						B.Add(face_subset, face);
						B.Add(face_subset_all_elem, face);
					}
					any = true;
				}

				delete vs;
				delete x;

				face_idx++;
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
		static std::vector<argument_spec> nm_ = { { true, "input", "ifcfile" }, { true, "input_voxels", "voxels" }, { true, "input_surfaces", "surfaceset" }, { true, "output_path", "string" }, {false, "individual_faces", "integer"}, {false, "factor", "integer"}, {false, "face_count", "integer"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		const filtered_files_t& ifc_files = scope.get_value<filtered_files_t>("input");
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input_voxels");
		geometry_collection_t* surfaces = scope.get_value<geometry_collection_t*>("input_surfaces");
		const std::string output_path = scope.get_value<std::string>("output_path");
		const bool individual_faces = scope.get_value_or<int>("individual_faces", 1) == 1;
		const int factor = scope.get_value_or<int>("factor", 2);
		const int face_count = scope.get_value_or<int>("face_count", 3);

		std::ofstream json(output_path.c_str());
		json << "[\n";
		int n = 0;

		revoxelize_and_check_overlap(voxels, *surfaces, individual_faces, factor, [&json, &n, &individual_faces, &face_count](std::pair<void*, int> iden, const TopoDS_Compound& face_subset) {
			bool include = !individual_faces;
			if (individual_faces) {
				TopExp_Explorer exp(face_subset, TopAbs_FACE);
				int num_faces = 0;
				for (; exp.More(); exp.Next()) {
					num_faces++;
				}

				// @todo arbitrary value alert
				if (num_faces > face_count) {
					include = true;
				}
			}
			if (include) {
				std::string guid = *((IfcUtil::IfcBaseEntity*)((IfcParse::IfcFile*)iden.first)->instance_by_id(iden.second))->get("GlobalId");
				if (n++) {
					json << ",\n";
				}
				json << "{\"id\":" << iden.second << ",\"guid\":\"" << guid << "\"}";
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

		revoxelize_and_check_overlap(voxels, *surfaces, true, 2, [&f0, &new_file](std::pair<void*, int> iden, const TopoDS_Compound& face_subset) {
			TopExp_Explorer exp(face_subset, TopAbs_FACE);
			int num_faces = 0;
			for (; exp.More(); exp.Next()) {
				num_faces++;
			}

			// @todo arbitrary value alert
			if (num_faces >= 1) {
				auto inst = f0->instance_by_id(iden.second);
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

		revoxelize_and_check_overlap(voxels, *surfaces, true, 2, [&output_path](std::pair<void*, int> iden, const TopoDS_Compound& face_subset) {
			std::string fn = output_path + DIRSEP + std::to_string(iden.second) + ".brep";
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

template <int plane_or_halfspace=0>
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
		).Face();

		TopoDS_Shape bool_result;

		if (plane_or_halfspace == 0) {
			bool_result = BRepAlgoAPI_Common(face, box).Shape();
		} else {
			BRepGProp_Face prop(face);
			double u1, u2, v1, v2;
			prop.Bounds(u1, u2, v1, v2);
			gp_Pnt p;
			gp_Vec v;
			prop.Normal((u1 + u2) / 2., (v1 + v2) / 2, p, v);
			// mass opposite of normal
			BRepPrimAPI_MakeHalfSpace mhs(face, p.XYZ() - v.XYZ());
			bool_result = BRepAlgoAPI_Common(box, mhs.Solid()).Shape();
		}

		TopoDS_Compound C;
		if (bool_result.ShapeType() == TopAbs_COMPOUND) {
			C = TopoDS::Compound(bool_result);
		} else {
			BRep_Builder B;
			B.MakeCompound(C);
			B.Add(C, bool_result);
		}

		BRepMesh_IncrementalMesh(C, 0.001);
		
		geometry_collection_t* single = new geometry_collection_t{ { {nullptr, 0}, C} };
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


class op_free: public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		delete voxels;
		symbol_value v;
		return v;
	}
};

class op_assert : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels|integer" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		try {
			auto voxels = scope.get_value<abstract_voxel_storage*>("input");
			if (voxels->count() == 0) {
				throw assertion_error("Failed assert");
			}
		} catch (scope_map::value_error&) {
			auto v = scope.get_value<int>("input");
			if (v == 0) {
				throw assertion_error("Failed assert");
			}
		}
		symbol_value v;
		return v;
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
		static std::vector<argument_spec> nm_ = { { true, "a", "voxels" }, { true, "b", "voxels" }, {false, "if_non_empty", "integer"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* a = scope.get_value<abstract_voxel_storage*>("a");
		abstract_voxel_storage* b = scope.get_value<abstract_voxel_storage*>("b");
		if (scope.get_value_or<int>("if_non_empty", 0) == 1 && b->count() == 0) {
			return a;
		}
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
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "dx", "integer|real" },{ true, "dy", "integer|real" },{ true, "dz", "integer|real" }, { false, "until", "voxels" }, { false, "max", "integer" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");

		int dx = scope.get_length("dx");
		int dy = scope.get_length("dy");
		int dz = scope.get_length("dz");

		abstract_voxel_storage* until = nullptr;
		try {
			until = scope.get_value<abstract_voxel_storage*>("until");
		} catch (scope_map::not_in_scope&) { }

		boost::optional<int> max_depth;
		try {
			max_depth = scope.get_value<int>("max");
		} catch (scope_map::not_in_scope&) {}

		T s;
		s.until = until;
		s.max_depth = max_depth;
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
		
		bit_t fmt;
		auto result = voxels->empty_copy_as(&fmt);
		uint32_t val;

		for (auto& pos : *voxels) {
			voxels->Get(pos, &val);
			if (Pred()(val, rhs)) {
				result->Set(pos);
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
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "seed", "voxels" }, { false, "depth", "integer|real" }, { false, "connectedness", "integer" }, {false, "type", "string"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		regular_voxel_storage* voxels = (regular_voxel_storage*) scope.get_value<abstract_voxel_storage*>("input");
		regular_voxel_storage* seed = (regular_voxel_storage*) scope.get_value<abstract_voxel_storage*>("seed");

		{
			auto acvs_voxels = dynamic_cast<abstract_chunked_voxel_storage*>(voxels);
			auto acvs_seed = dynamic_cast<abstract_chunked_voxel_storage*>(seed);

			if (!acvs_voxels || !acvs_seed) {
				throw std::runtime_error("Traversal operations are not supported on non-chunked storage");
			}

			if (!(acvs_voxels->grid_offset() == acvs_seed->grid_offset()).all()) {
				throw std::runtime_error("Traversal operations on unaligned voxel grids are not supported");
			}
		}

		boost::optional<double> max_depth;
		try {
			max_depth = scope.get_length("depth");
		} catch (scope_map::not_in_scope&) {
			// traversal with unconstrained depth
		}

		bool use_value = scope.has("type") && scope.get_value<std::string>("type") == "uint";

		int C = 6;
		try {
			C = scope.get_value<int>("connectedness");
		} catch (scope_map::not_in_scope&) {
			// default 6 connectedness
		}

		if (C != 6 && C != 26) {
			throw std::runtime_error("Connectedness should be 6 or 26");
		}

		if (use_value && C != 26) {
			throw std::runtime_error("Connectedness should be 26 when using value");
		}

		regular_voxel_storage* output;
		if (use_value) {
			voxel_uint32_t fmt;
			output = (regular_voxel_storage*)voxels->empty_copy_as(&fmt);
		} else {
			output = (regular_voxel_storage*)voxels->empty_copy();
		}
		
		visitor<6> v6;
		visitor<26> v26;
		v6.max_depth = max_depth;
		v26.max_depth = max_depth;

		uint32_t val;

		auto callback = [this, output, &v26, &val, &use_value](const tagged_index& pos) {
			if (use_value) {
				val = v26.depth * 10. + 1;
				output->Set(pos.pos, &val);
			} else if (pos.which == tagged_index::VOXEL) {
				output->Set(pos.pos);
			} else {
				((abstract_chunked_voxel_storage*)output)->create_constant(pos.pos, 1U);
			}
		};

		// Type erasure to get a runtime argument into a template arg.
		std::function<void(decltype(callback), decltype(voxels), decltype(seed))> run_visitor;

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
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "value", "integer" }, { false, "type", "string"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		regular_voxel_storage* voxels = (regular_voxel_storage*)scope.get_value<abstract_voxel_storage*>("input");
		int value = scope.get_value<int>("value");

		abstract_chunked_voxel_storage* output;

		if (scope.has("type")) {
			if (scope.get_value<std::string>("type") == "uint") {
				voxel_uint32_t fmt;
				output = (abstract_chunked_voxel_storage*)voxels->empty_copy_as(&fmt);
			} else if (scope.get_value<std::string>("type") == "bit") {
				bit_t fmt;
				output = (abstract_chunked_voxel_storage*)voxels->empty_copy_as(&fmt);
			} else {
				throw std::runtime_error("not implemented");
			}			
		} else {
			output = (abstract_chunked_voxel_storage*)voxels->empty_copy();
		}

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

class op_copy : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { false, "type", "string"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		regular_voxel_storage* voxels = (regular_voxel_storage*)scope.get_value<abstract_voxel_storage*>("input");

		abstract_chunked_voxel_storage* output;

		if (scope.has("type")) {
			if (scope.get_value<std::string>("type") == "uint") {
				voxel_uint32_t fmt;
				output = (abstract_chunked_voxel_storage*)voxels->copy_as(&fmt);
			} else if (scope.get_value<std::string>("type") == "bit") {
				bit_t fmt;
				output = (abstract_chunked_voxel_storage*)voxels->copy_as(&fmt);
			} else {
				throw std::runtime_error("not implemented");
			}
		} else {
			output = (abstract_chunked_voxel_storage*)voxels->copy();
		}

		return output;
	}
};

class op_repeat_slice : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, {true, "axis", "integer"}, {true, "location", "real"}, {true, "repetitions", "integer"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		auto voxels = (chunked_voxel_storage<bit_t>*)scope.get_value<abstract_voxel_storage*>("input");
		
		auto grid_offset = voxels->grid_offset();
		auto num_chunks = voxels->num_chunks();

		auto axis = scope.get_value<int>("axis");
		auto location = scope.get_value<double>("location");
		auto repetitions = scope.get_value<int>("repetitions");
		size_t location_i = (size_t) std::floor((location - voxels->origin().get(axis)) / voxels->voxel_size());

		vec_n<3, size_t> offset = make_vec<size_t>(0, 0, 0);
		vec_n<3, size_t> offset_after_repeat = make_vec<size_t>(0, 0, 0);
		offset_after_repeat.get(axis) = (size_t)std::abs(repetitions);

		if (repetitions < 0) {
			size_t d = integer_ceil_div((size_t)-repetitions, voxels->chunk_size());
			grid_offset.get(axis) -= d;
			num_chunks.get(axis) += d;
			offset.get(axis) = d * voxels->chunk_size();
		} else if (repetitions > 0) {
			num_chunks.get(axis) += integer_ceil_div((size_t)repetitions, voxels->chunk_size());
		}

		abstract_chunked_voxel_storage* output = new chunked_voxel_storage<bit_t>(
			grid_offset,
			voxels->voxel_size(),
			voxels->chunk_size(),
			num_chunks
		);

		for (auto it = voxels->begin(); it != voxels->end(); ++it) {
			if ((*it).get(axis) < location_i) {
				if (repetitions < 0) {
					output->Set(offset + (*it) - offset_after_repeat);
				} else {
					output->Set(offset + (*it));
				}
			} else if ((*it).get(axis) == location_i) {
				auto ijk = offset + (*it);
				if (repetitions < 0) {
					ijk -= offset_after_repeat;
				}
				for (size_t i = 0; i <= (size_t) std::abs(repetitions); ++i) {
					auto ijk2 = ijk;
					ijk2.get(axis) += i;
					output->Set(ijk2);
				}
			} else { // greater than
				if (repetitions < 0) {
					output->Set(offset + (*it));
				} else {
					output->Set(offset + (*it) + offset_after_repeat);
				}
			}
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

	// @todo this is a very minimal implementation for specific cases
	class instance_by_property_map_filter : public instance_filter_t {
	public:
		typedef std::vector<std::pair<std::string, function_arg_value_type> > values_t;
	private:
		values_t attr_pattern_;
	public:
		instance_by_property_map_filter(const values_t& pattern)
			: attr_pattern_(pattern)
		{}

		bool operator()(const IfcUtil::IfcBaseEntity* inst) const {
#ifdef IFCOPENSHELL_05
			throw std::runtime_error("Not implemented");
#else
			for (auto& p : attr_pattern_) {
				auto rels = inst->get_inverse("IsDefinedBy");
				bool has_match = false;
				if (rels) {
					for (auto& rel : *rels) {
						if (rel->declaration().is("IfcRelDefinesByProperties")) {
							IfcUtil::IfcBaseClass* pset = *((IfcUtil::IfcBaseEntity*)rel)->get("RelatingPropertyDefinition");
							if (pset->declaration().is("IfcPropertySet")) {
								instance_list_t::ptr props = *((IfcUtil::IfcBaseEntity*)pset)->get("HasProperties");
								for (auto& prop : *props) {
									if (prop->declaration().is("IfcPropertySingleValue")) {
										auto name = (std::string) *((IfcUtil::IfcBaseEntity*)prop)->get("Name");
										
										/*
										// In the voxelfile grammer we can also have keywords starting with an alpha character
										// so for the string comparison we need to trim off any others.
										auto it = std::find_if(name.begin(), name.end(), [](char c) { return std::isalpha(c); });
										if (it == name.end()) {
											return false;
										}
										name = name.substr(std::distance(name.begin(), it));
										*/

										std::replace_if(name.begin(), name.end(), [](char c) { return std::isdigit(c); }, 'n');

										if (name == p.first) {
											has_match = true;
											IfcUtil::IfcBaseClass* val = *((IfcUtil::IfcBaseEntity*)prop)->get("NominalValue");
											auto val_attr = val->data().getArgument(0);
											if (val_attr->type() == IfcUtil::Argument_BOOL) {
												auto v_ifc = (bool)*val_attr;
												int v_filter = 0;
												try {
													v_filter = boost::get<int>(p.second);
												} catch (boost::bad_get&) {
													return false;
												}
												bool match = (v_ifc ? 1 : 0) == v_filter;
												if (!match) {
													return false;
												}
											} else if (val_attr->type() == IfcUtil::Argument_STRING) {
												auto v_ifc = (std::string)*val_attr;
												std::string v_filter;
												try {
													v_filter = boost::get<std::string>(p.second);
												} catch (boost::bad_get&) {
													return false;
												}
												bool match = v_ifc == v_filter;
												if (!match) {
													return false;
												}
											} else {
												return false;
											}
										}
									}
								}
							}
						}
					}
				}

				if (!has_match) {
					return false;
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
	bool catch_all() const {
		return true;
	}
	bool only_local() const {
		return true;
	}
	symbol_value invoke(const scope_map& scope) const {
		static std::set<std::string> to_exclude{ "input" };
		filtered_files_t ifc_files_copy = scope.get_value<filtered_files_t>("input");
		instance_by_property_map_filter::values_t vs;
		for (auto& p : scope) {
			if (to_exclude.find(p.first) == to_exclude.end()) {
				auto s = boost::get<function_arg_value_type>(p.second);
				vs.push_back({ p.first, s });
			}
		}
		ifc_files_copy.filter = new instance_by_property_map_filter(vs);
		return ifc_files_copy;
	}
};

#endif

class op_mesh : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "filename", "string"}, {false, "use_value", "integer"}, {false, "with_components", "integer"}, { false, "groups", "voxels" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		auto voxels = scope.get_value<abstract_voxel_storage*>("input");
		auto use_value = scope.get_value_or<int>("use_value", -1);
		auto with_components = scope.get_value_or<int>("with_components", -1);
		auto filename = scope.get_value<std::string>("filename");

		auto groups = (regular_voxel_storage*) scope.get_value_or<abstract_voxel_storage*>("groups", nullptr);

		std::ofstream ofs(filename.c_str());

		if (groups) {
			obj_export_helper helper(ofs);
			std::map<uint32_t, size_t> counts;
			group_by(groups, voxels, [&helper, &ofs](uint32_t id, abstract_voxel_storage* c) {
				ofs << "g id-" << id << "\n";
				((regular_voxel_storage*)c)->obj_export(helper, false, false);
			}
#ifdef OLD_GROUP_BY
			, scope.get_value_or<int>("THREADS", 1)
#else
			, counts
#endif
			);
		} else {

			if (voxels->value_bits() == 1) {
				((regular_voxel_storage*)voxels)->obj_export(ofs, with_components != 0);
			} else {
				((regular_voxel_storage*)voxels)->obj_export(ofs, with_components != 0 && use_value != 1, use_value == 1);
			}
		}
		symbol_value v;
		return v;
	}
};

template <int sweep_or_move>
class op_local_sweep : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "surfaceset" }, { true, "dx", "real"}, { true, "dy", "real"}, { true, "dz", "real"}, {false, "VOXELSIZE", "real"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		geometry_collection_t* surfaces = scope.get_value<geometry_collection_t*>("input");
		auto dx = scope.get_value<double>("dx");
		auto dy = scope.get_value<double>("dy");
		auto dz = scope.get_value<double>("dz");
		auto vs = scope.get_value<double>("VOXELSIZE");

		auto l = gp_Vec(dx, dy, dz).Magnitude();
		size_t n = (size_t)std::ceil(l / vs);
		double stp = l / n;
		
		auto copy = new geometry_collection_t;
		// note n inclusive
		for (int i = sweep_or_move == 0 ? 0 : n; i <= n; ++i) {
			auto s = copy->size();
			copy->insert(copy->end(), surfaces->begin(), surfaces->end());
			if (i == 0) {
				// no sense in applying the zero length translation
				continue;
			}
			for (auto it = copy->begin() + s; it != copy->end(); ++it) {
				gp_Trsf trsf;
				trsf.SetTranslation(gp_Vec(dx * stp * i, dy * stp * i, dz * stp * i));

				it->second.Location(
					it->second.Location().Transformation() * trsf
				);
			}
		}
		return copy;
	}
};

namespace {
	void export_csv_or_obj(int mode, regular_voxel_storage* voxels, const std::string& filename) {
		std::ofstream ofs(filename.c_str());

		bool use_value = mode == 0 && (voxels->value_bits() == 32 || voxels->value_bits() == 64);

		auto sz = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->voxel_size();
		auto szl = (long)dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->chunk_size();
		auto left = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->grid_offset();

		uint32_t v;
		normal_and_curvature<int16_t> v2;

		std::string prefix = mode == 0 ? "" : "v ";
		std::string sep = mode == 0 ? "," : " ";

		for (auto ijk : *voxels) {
			if (use_value) {
				voxels->Get(ijk, voxels->value_bits() == 32 ? (void*)&v : (void*)&v2);
				if (v == 0) {
					// @todo why is this needed?
					continue;
				}
			}

			auto xyz = (ijk.as<long>() + left * szl).as<double>() * sz;

			ofs << prefix
				<< xyz.get<0>() << sep
				<< xyz.get<1>() << sep
				<< xyz.get<2>();

			if (use_value) {
				if (voxels->value_bits() == 32) {
					ofs << sep << v;
				} else {
					auto v3 = v2.convert<float>();
					ofs << sep << v3.nxyz_curv[0] << sep
						<< v3.nxyz_curv[1] << sep
						<< v3.nxyz_curv[2] << sep
						<< v3.nxyz_curv[3];
				}
			}

			ofs << "\n";
		}
	}
}

template <int mode=0>
class op_export_csv : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "filename", "string"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		auto voxels = (regular_voxel_storage*) scope.get_value<abstract_voxel_storage*>("input");
		auto filename = scope.get_value<std::string>("filename");
		
		export_csv_or_obj(mode, voxels, filename);

		symbol_value v_null;
		return v_null;
	}
};

class op_component_foreach : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "function", "string"}, { true, "argument", "string"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		abstract_voxel_storage* voxels = scope.get_value<abstract_voxel_storage*>("input");
		const std::string& function_name = scope.get_value<std::string>("function");
		const std::string& argument_name = scope.get_value<std::string>("argument");

		auto fn = [&scope, &function_name, &argument_name](regular_voxel_storage* c) {
			std::cout << "Component " << c->count() << std::endl;
			auto scope_copy = scope;
			scope_copy.functions = scope.functions;
			scope_copy[argument_name] = c;

			invoke_function_by_name(scope_copy, function_name);
		};

		if (voxels->value_bits() == 1) {
			// Invoke by connected component traversal

			connected_components((regular_voxel_storage*)voxels, fn);
		} else if (voxels->value_bits() == 32) {
			// Invoke by uint32 value groups

			// @todo eliminate overlap with group_by()

			uint32_t v;
			static bit_t fmt;

			std::map<uint32_t, abstract_voxel_storage*> map;

			for (auto& ijk : *(regular_voxel_storage*)voxels) {
				voxels->Get(ijk, &v);
				if (v == 0) {
					continue;
				}
				abstract_voxel_storage* r;
				auto it = map.find(v);
				if (it == map.end()) {
					map.insert({ v, r = voxels->empty_copy_as(&fmt) });
				} else {
					r = it->second;
				}
				r->Set(ijk);
			}

			for (auto& r : map) {
				fn((regular_voxel_storage*)r.second);
				delete r.second;
			}
		}

		symbol_value v;
		return v;
	}
};

class op_normal_estimate : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "max_depth", "integer"} };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		static normal_and_curvature_t fmt;
		normal_and_curvature<int16_t> v;

		auto voxels = (regular_voxel_storage*)scope.get_value<abstract_voxel_storage*>("input");
		auto result = voxels->empty_copy_as(&fmt);

		const int max_depth = scope.get_value<int>("max_depth");
		auto box_dim = 1 + max_depth * 2;

		std::vector<float> coords;
		coords.reserve(box_dim * box_dim * box_dim);

		for (auto it = voxels->begin(); it != voxels->end(); ++it) {
			visitor<26> vis;
			vis.max_depth = max_depth;

			coords.clear();
			
			vis([&coords](const tagged_index& pos) {
				if (pos.which == tagged_index::VOXEL) {
					coords.push_back(pos.pos.get(0));
					coords.push_back(pos.pos.get(1));
					coords.push_back(pos.pos.get(2));
				} else {
					throw std::runtime_error("Unexpected");
				}
			}, voxels, *it);

			Eigen::MatrixXf points = Eigen::Map<Eigen::MatrixXf>(coords.data(), 3, coords.size() / 3).transpose();

			Eigen::MatrixXf centered = points.rowwise() - points.colwise().mean();
			Eigen::MatrixXf cov = centered.adjoint() * centered;
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);

			auto vec = eig.eigenvectors().col(0);
			auto norm = vec.normalized();
			auto curv = eig.eigenvalues().col(0).value() / eig.eigenvalues().sum();

			v.nxyz_curv = {
				(int16_t) (norm.x() * (float) (std::numeric_limits<int16_t>::max()-1)),
				(int16_t) (norm.y() * (float) (std::numeric_limits<int16_t>::max()-1)),
				(int16_t) (norm.z() * (float) (std::numeric_limits<int16_t>::max()-1)),
				(int16_t) (curv * (float) (std::numeric_limits<int16_t>::max()-1))
			};

			result->Set(*it, &v);
		}

		return result;
	}
};

namespace {
	class check_curvature_and_normal_deviation {
	private:
		regular_voxel_storage* voxels_;
		normal_and_curvature<float> seed_data_float_;
		double angular_tolerance_, max_curvature_;

	public:
		check_curvature_and_normal_deviation(regular_voxel_storage* voxels, const vec_n<3, size_t>& seed, double angular_tolerance, double max_curvature)
			: voxels_(voxels)
			, angular_tolerance_(angular_tolerance)
			, max_curvature_(max_curvature)
		{
			normal_and_curvature<int16_t> seed_data;
			voxels->Get(seed, &seed_data);
			seed_data_float_ = seed_data.convert<float>();
		}

		inline bool operator()(const vec_n<3, size_t>& pos) {
			normal_and_curvature<int16_t> data;
			voxels_->Get(pos, &data);
			auto data_float = data.convert<float>();

			if (data_float.curvature() > max_curvature_) {
				return false;
			}
			
			Eigen::Map<Eigen::Vector3f> v0(seed_data_float_.nxyz_curv.data());
			Eigen::Map<Eigen::Vector3f> v1(data_float.nxyz_curv.data());

			double angle = std::acos(std::abs(v0.dot(v1)));

			if (angle > angular_tolerance_) {
				return false;
			}

			return true;
		}
	};
}

class op_segment : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { false, "angular_tolerance", "real" }, { false, "max_curvature", "real" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		static bit_t bits;
		static voxel_uint32_t uints;
		
		auto voxels = (regular_voxel_storage*)scope.get_value<abstract_voxel_storage*>("input");
		if (voxels->value_bits() != normal_and_curvature_t::size_in_bits) {
			throw std::runtime_error("Expected normal and curvature voxel type");
		}

		const double angular_tolerance = scope.get_value_or<double>("angular_tolerance", 0.1);
		const double max_curvature = scope.get_value_or<double>("max_curvature", 0.01);
		
		auto result = (regular_voxel_storage*)voxels->empty_copy_as(&uints);

		auto voxels_bit = (regular_voxel_storage*) voxels->copy_as(&bits);

		uint32_t component_index = 0;

		while (voxels_bit->count()) {
			++component_index;

			auto seed = std::min_element(voxels_bit->begin(), voxels_bit->end(), [&voxels](const vec_n<3, size_t>& a, const vec_n<3, size_t>& b) {
				normal_and_curvature<int16_t> A, B;
				voxels->Get(a, &A);
				voxels->Get(b, &B);
				return A.curvature() < B.curvature();
			});

			normal_and_curvature<int16_t> A;
			voxels->Get(*seed, &A);

			if (A.convert<float>().curvature() > max_curvature) {
				break;
			}

			check_curvature_and_normal_deviation lookup_curv(voxels, *seed, angular_tolerance, max_curvature);

			visitor<26, DOF_XYZ, std::function<bool(const vec_n<3, size_t>&)>> vis;

			vis.set_postcondition(std::ref(lookup_curv));

			vis([&result, &component_index](const tagged_index& pos) {
				result->Set(pos.pos, &component_index);
			}, voxels_bit, *seed);

			voxels_bit->boolean_subtraction_inplace(vis.get_visited());
		}
	
		return result;
	}
};

class op_keep_neighbours : public voxel_operation {
public:
	const std::vector<argument_spec>& arg_names() const {
		static std::vector<argument_spec> nm_ = { { true, "input", "voxels" }, { true, "num_neighbours", "integer" }, { true, "connectivity", "integer" } };
		return nm_;
	}
	symbol_value invoke(const scope_map& scope) const {
		auto voxels = (regular_voxel_storage*)scope.get_value<abstract_voxel_storage*>("input");
		auto result = (regular_voxel_storage*)voxels->empty_copy();
		const auto num_neighbours = (size_t) scope.get_value<int>("num_neighbours");
		const auto connectivity = scope.get_value<int>("connectivity");
		auto extents = voxels->extents().as<long>();

		for (auto it = voxels->begin(); it != voxels->end(); ++it) {
			size_t num = 0;

			if (connectivity == 6) {

				for (size_t f = 0; f < 6; ++f) {
					vec_n<3, long> n;
					size_t normal = f / 2;
					size_t o0 = (normal + 1) % 3;
					size_t o1 = (normal + 2) % 3;
					size_t side = f % 2;
					n.get(normal) = side ? 1 : -1;
					if (it.neighbour(n)) {
						++num;
					}
				}

			} else {

				for (long i = -1; i <= 1; ++i) {
					for (long j = -1; j <= 1; ++j) {
						for (long k = -1; k <= 1; ++k) {
							if (i == 0 && j == 0 && k == 0) {
								continue;
							}
							auto ijk2 = (*it).as<long>() + make_vec<long>(i, j, k);
							if ((ijk2 >= 0).all() && (ijk2 < extents).all()) {
								if (voxels->Get(ijk2.as<size_t>())) {
									++num;
								}
							}
						}
					}
				}

			}

			if (num >= num_neighbours) {
				result->Set(*it);
			}
		}

		return result;
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

		if (statement_.call().name() == "free") {
			// @todo prevent double free, can this implementation be a bit better?
			// currently the call evaluation doesn't seem to have access to global scope
			context.erase(boost::get<std::string>(statement_.call().args().front().value()));
		}

		delete op;

		return r;
	}
};

scope_map run(const std::vector<statement_or_function_def>& statements, double size, size_t threads = 0, size_t chunk_size = 128, bool with_mesh = false, bool with_progress_on_cout = false, bool no_vox = false, bool with_point_cloud = false);

#endif
