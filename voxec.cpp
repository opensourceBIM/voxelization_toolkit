// @todo erode for angle slabs
// @todo finish memory mapped chunked storage and make default

#include "voxec.h"
#include "progress.h"
#include "json_logger.h"

voxel_operation_map::map_t& voxel_operation_map::map() {
	static voxel_operation_map::map_t m;
	static bool initialized = false;
	if (!initialized) {
#ifdef WITH_IFC
		m.insert(std::make_pair(std::string("parse"), &instantiate<op_parse_ifc_file>));
		m.insert(std::make_pair(std::string("create_geometry"), &instantiate<op_create_geometry>));
#endif
		m.insert(std::make_pair(std::string("voxelize"), &instantiate<op_voxelize>));
		// m.insert(std::make_pair(std::string("voxelize_ids"), &instantiate<op_voxelize_ids>));
		m.insert(std::make_pair(std::string("fill_gaps"), &instantiate<op_fill_gaps>));
		m.insert(std::make_pair(std::string("offset"), &instantiate<op_offset>));
		m.insert(std::make_pair(std::string("offset_xy"), &instantiate<op_offset_xy>));
		m.insert(std::make_pair(std::string("outmost"), &instantiate<op_outmost>));
		m.insert(std::make_pair(std::string("count"), &instantiate<op_count>));
		m.insert(std::make_pair(std::string("volume"), &instantiate<op_volume>));
		m.insert(std::make_pair(std::string("volume2"), &instantiate<op_volume_2>));
		m.insert(std::make_pair(std::string("exterior"), &instantiate<op_volume_inverted>));
		m.insert(std::make_pair(std::string("invert"), &instantiate<op_invert>));
		m.insert(std::make_pair(std::string("fill_volume"), &instantiate<op_fill_volume>));
		m.insert(std::make_pair(std::string("union"), &instantiate<op_union>));
		m.insert(std::make_pair(std::string("subtract"), &instantiate<op_subtract>));
		m.insert(std::make_pair(std::string("intersect"), &instantiate<op_intersect>));
		m.insert(std::make_pair(std::string("shift"), &instantiate<op_shift>)); 
		m.insert(std::make_pair(std::string("sweep"), &instantiate<op_sweep>));
		m.insert(std::make_pair(std::string("traverse"), &instantiate<op_traverse>));
		m.insert(std::make_pair(std::string("constant_like"), &instantiate<op_constant_like>));
		m.insert(std::make_pair(std::string("resample"), &instantiate<op_resample>));
		m.insert(std::make_pair(std::string("collapse"), &instantiate<op_collapse>));
		m.insert(std::make_pair(std::string("collapse_count"), &instantiate<op_collapse_count>));
		m.insert(std::make_pair(std::string("print_components"), &instantiate<op_print_components>));
		m.insert(std::make_pair(std::string("describe_components"), &instantiate<op_describe_components>));
		m.insert(std::make_pair(std::string("describe_group_by"), &instantiate<op_describe_group_by>));
		m.insert(std::make_pair(std::string("keep_components"), &instantiate<op_keep_components>));
		m.insert(std::make_pair(std::string("dump_surfaces"), &instantiate<op_dump_surfaces>));
		m.insert(std::make_pair(std::string("json_stats"), &instantiate<op_json_stats>));
		m.insert(std::make_pair(std::string("greater_than"), &instantiate<op_greater>));
		m.insert(std::make_pair(std::string("less_than"), &instantiate<op_less>));
#ifdef WITH_IFC
		m.insert(std::make_pair(std::string("export_ifc"), &instantiate<op_export_ifc>));
		m.insert(std::make_pair(std::string("export_json"), &instantiate<op_export_elements>));
		m.insert(std::make_pair(std::string("filter_attributes"), &instantiate<op_create_attr_filter>));
		m.insert(std::make_pair(std::string("filter_properties"), &instantiate<op_create_prop_filter>));
#endif
		m.insert(std::make_pair(std::string("zeros"), &instantiate<op_zeros>));
		m.insert(std::make_pair(std::string("plane"), &instantiate<op_plane>));
		m.insert(std::make_pair(std::string("mesh"), &instantiate<op_mesh>));
		initialized = true;
	}
	return m;
}

std::ostream& operator<<(std::ostream& a, const std::vector<std::string>& b) {
	a << "{";
	bool first = true;
	for (auto& v : b) {
		if (!first) {
			a << ",";
		}
		first = false;
		a << v;
	}
	a << "}";
	return a;
}

std::ostream& operator<<(std::ostream& a, const boost::variant<double, int, std::string, std::vector<std::string>>& b) {
	// @todo must be a better way
	switch (b.which()) {
	case 0:
		a << boost::get<double>(b);
		break;
	case 1:
		a << boost::get<int>(b);
		break;
	case 2:
		a << boost::get<std::string>(b);
		break;
	case 3:
		a << boost::get<std::vector<std::string>>(b);
		break;
	default:
		break;
	}
	return a;
}

std::ostream& operator<<(std::ostream& a, const function_call_type& b) {
	a << boost::fusion::at_c<0>(b) << "(";
	bool first = true;
	for (auto& arg : boost::fusion::at_c<1>(b)) {
		if (!first) {
			a << ",";
		}
		first = false;
		if (arg.has_keyword()) {
			a << *boost::fusion::at_c<0>(arg) << "=";
		}
		a << boost::fusion::at_c<1>(arg);
	}
	a << ")";
	return a;
}

std::ostream& operator<<(std::ostream& a, const statement_type& b) {
	a << boost::fusion::at_c<0>(b) << " = " << boost::fusion::at_c<1>(b);
	return a;
}

class log_visitor {
private:
	json_logger::meta_value v_;
public:
	const json_logger::meta_value& value() const { return v_; }

	typedef void result_type;

	void operator()(const boost::blank&) {
		v_ = std::string("void");
	}

	void operator()(int v) {
		v_ = (long) v;
	}

	void operator()(double v) {
		v_ = v;
	}

	void operator()(const std::string& v) {
		if (v.at(0) == '"') {
			v_ = v.substr(1, v.size() - 2);
		}
		else {
			v_ = v;
		}
	}

	void operator()(const function_arg_value_type& v) {
		v.apply_visitor(*this);
	}

	void operator()(filtered_files_t const v) {	}

	void operator()(geometry_collection_t* const v) { }

	void operator()(abstract_voxel_storage* const v) { }
};


scope_map run(const std::vector<statement_type>& statements, double size, size_t threads, size_t chunk_size, bool with_mesh, bool with_progress_on_cout) {
	scope_map context;

	std::unique_ptr<progress_bar> pb;
	std::unique_ptr<application_progress> ap;
	std::function<void(float)> pfn = [&pb](float f) { (*pb)(f); };
	std::function<void(float)> apfn = [&ap](float f) { (*ap)(f); };
	if (with_progress_on_cout) {
		pb = std::make_unique<progress_bar>(std::cout, progress_bar::DOTS);
	}
	else {
		pb = std::make_unique<progress_bar>(std::cout);
	}

	std::vector<float> estimates(statements.size(), 1.f);
	ap = std::make_unique<application_progress>(estimates, pfn);

	function_arg_value_type threads_ = (int)threads;
	context["THREADS"] = threads_;

	function_arg_value_type chunk_size_ = (int)chunk_size;
	context["CHUNKSIZE"] = chunk_size_;

	function_arg_value_type vsize_ = size;
	context["VOXELSIZE"] = vsize_;

	size_t n = 0;

	long HAS_VOXELS = boost::mpl::distance<
		boost::mpl::begin<symbol_value::types>::type,
		boost::mpl::find<symbol_value::types, abstract_voxel_storage*>::type
	>::type::value;

	for (auto it = statements.begin(); it != statements.end(); ++it) {
		auto& st = *it;
		const bool is_last = it == --statements.end();

		auto statement_string = boost::lexical_cast<std::string>(st);

		json_logger::message(json_logger::LOG_NOTICE, "executing {statement}", { 
			{"statement", {{"text", statement_string}}}
		});

		voxel_operation_runner op(st);
		op.application_progress_callback = apfn;
		op.silent = with_progress_on_cout;
		context[st.assignee()] = op.run(st, context);

		if (context[st.assignee()].which() == HAS_VOXELS) {
			auto voxels = boost::get<abstract_voxel_storage*>(context[st.assignee()]);
			voxel_writer w;
			w.SetVoxels(voxels);
			w.Write(boost::lexical_cast<std::string>(n) + ".vox");

			if (with_mesh && is_last) {
				std::string ofn = boost::lexical_cast<std::string>(n) + ".obj";
				std::ofstream ofs(ofn.c_str());
				((regular_voxel_storage*)voxels)->obj_export(ofs);
			}

			if (dynamic_cast<abstract_chunked_voxel_storage*>(voxels)) {
				auto left = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->grid_offset();
				auto nc = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->num_chunks();
				auto right = (left + nc.as<long>()) - (decltype(left)::element_type)1;
				auto sz = dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->voxel_size();
				auto szl = (long)dynamic_cast<abstract_chunked_voxel_storage*>(voxels)->chunk_size();
				auto left_world = ((voxels->bounds()[0].as<long>() + left * szl).as<double>() * sz);
				auto right_world = ((voxels->bounds()[1].as<long>() + left * szl).as<double>() * sz);
				
				json_logger::message(json_logger::LOG_NOTICE, "storing {value} in {variable}", {
					{"variable", {{"name", st.assignee()}}},
					{"value", dump_info(voxels)},
				});
			}
		}

		if (ap) {
			ap->finished();
		}

		n++;
	}

	if (statements.size()) {
		log_visitor v;
		context[statements.back().assignee()].apply_visitor(v);

		json_logger::message(json_logger::LOG_NOTICE, "scripted finished with {variable}", {
				{"variable", {{"value", v.value()}}},
		});
	} else {
		json_logger::message(json_logger::LOG_WARNING, "no operations in input", {});
	}

	return context;
}

size_t padding_ = 32;
size_t get_padding() {
	return padding_;
}
void set_padding(size_t v) {
	padding_ = v;
}