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
		m.insert(std::make_pair(std::string("copy"), &instantiate<op_copy>));
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
		m.insert(std::make_pair(std::string("count_components"), &instantiate<op_count_components>));
		m.insert(std::make_pair(std::string("print_values"), &instantiate<op_print_values>));
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
		m.insert(std::make_pair(std::string("plane"), &instantiate<op_plane<>>));
		m.insert(std::make_pair(std::string("halfspace"), &instantiate<op_plane<1>>));
		m.insert(std::make_pair(std::string("mesh"), &instantiate<op_mesh>));
		m.insert(std::make_pair(std::string("export_csv"), &instantiate<op_export_csv<>>));
		m.insert(std::make_pair(std::string("export_point_cloud"), &instantiate<op_export_csv<1>>));
		m.insert(std::make_pair(std::string("local_sweep"), &instantiate<op_local_sweep<0>>));
		m.insert(std::make_pair(std::string("local_move"), &instantiate<op_local_sweep<1>>));
		m.insert(std::make_pair(std::string("repeat_slice"), &instantiate<op_repeat_slice>));
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
	std::string assignee = boost::fusion::at_c<0>(b);
	if (assignee.size()) {
		a << assignee << " = ";
	}
	a << boost::fusion::at_c<1>(b);
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

template <typename T, typename U, typename Ts, typename Us>
class assigner {
	Ts& ts_;
	Us& us_;

public:
	assigner(Ts& ts, Us& us)
		: ts_(ts)
		, us_(us)
	{}

	void operator()(const T& t) {
		ts_.push_back(t);
	}

	void operator()(const U& u) {
		// @nb overwritten on purpose
		us_[u.name()] = u;
	}
};

void map_arguments(const std::vector<std::string> parameters, scope_map& context, const std::vector<function_arg_type>& args) {

	// @todo revisit the similarities with voxel_operation_runner

	bool has_keyword = false;
	std::map<std::string, function_arg_value_type> function_values;
	size_t position = 0;

	for (auto& arg : args) {
		if (has_keyword && !arg.has_keyword()) {
			throw std::runtime_error("Keyword argument supplied after positional argument");
		}
		has_keyword = arg.has_keyword();
		if (has_keyword) {
			if (function_values.find(*arg.keyword()) != function_values.end()) {
				throw std::runtime_error("Value for " + *arg.keyword() + "supplied multiple times");
			}
			function_values[*arg.keyword()] = arg.value();
		}
		else {
			if (position >= parameters.size()) {
				throw std::runtime_error("Too many positional arguments");
			}
			function_values[parameters[position]] = arg.value();
		}
		position++;
	}

	for (auto& pname : parameters) {
		if (function_values.find(pname) == function_values.end()) {
			throw std::runtime_error("No value supplied for " + pname);
		}

		symbol_value_assignment_visitor v(&context[pname], "*", context);
		function_values[pname].apply_visitor(v);
	}

	for (auto& arg : args) {
		if (arg.has_keyword() && context.find(*arg.keyword()) == context.end()) {
			throw std::runtime_error("Keyword argument " + *arg.keyword() + " unused");
		}
	}
}

void execute_statements(scope_map& context, const std::map<std::string, function_def_type>& functions, const std::vector<statement_type>& statements, bool silent, std::unique_ptr<application_progress>& ap, std::function<void(float)> apfn, std::string prefix="") {
	size_t n = 0;

	long HAS_VOXELS = boost::mpl::distance<
		boost::mpl::begin<symbol_value::types>::type,
		boost::mpl::find<symbol_value::types, abstract_voxel_storage*>::type
	>::type::value;

	for (auto it = statements.begin(); it != statements.end(); ++it) {
		auto& st = *it;
		// const bool is_last = it == --statements.end();

		auto statement_string = boost::lexical_cast<std::string>(st);

		json_logger::message(json_logger::LOG_NOTICE, "executing {statement}", {
			{"statement", {{"text", statement_string}}}
			});

		auto fnit = functions.find(st.call().name());
		if (fnit != functions.end()) {
			auto context_copy = context;
			map_arguments(
				fnit->second.args(),
				context_copy,
				st.call().args()
			);
			std::unique_ptr<application_progress> ap_unused;
			execute_statements(
				context_copy,
				functions,
				fnit->second.statements(),
				silent,
				ap_unused,
				[](float) {},
				boost::lexical_cast<std::string>(n) + "."
			);
			if (context_copy.find(fnit->second.result_identifier()) == context_copy.end()) {
				throw scope_map::not_in_scope("Undefined variable " + fnit->second.result_identifier());
			}
			context[st.assignee()] = context_copy[fnit->second.result_identifier()];
		} else {
			voxel_operation_runner op(st);
			op.application_progress_callback = apfn;
			op.silent = silent;
			context[st.assignee()] = op.run(st, context);
		}	

		if (context[st.assignee()].which() == HAS_VOXELS) {
			auto voxels = boost::get<abstract_voxel_storage*>(context[st.assignee()]);
			voxel_writer w;
			w.SetVoxels(voxels);
			w.Write(prefix + boost::lexical_cast<std::string>(n) + ".vox");

			/*
			// @nb automatic meshing of last operand output no longer supported
			if (with_mesh && is_last) {
				std::string ofn = boost::lexical_cast<std::string>(n) + ".obj";
				std::ofstream ofs(ofn.c_str());
				((regular_voxel_storage*)voxels)->obj_export(ofs);
			}
			*/

			if (dynamic_cast<abstract_chunked_voxel_storage*>(voxels)) {
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

		std::string msg = prefix.empty()
			? std::string("script finished with {variable}")
			: std::string("function finished with {variable}");

		json_logger::message(json_logger::LOG_NOTICE, msg, {
				{"variable", {{"value", v.value()}}},
			});
	}
	else {
		json_logger::message(json_logger::LOG_WARNING, "no operations in input", {});
	}
}

scope_map run(const std::vector<statement_or_function_def>& statements_and_functions, double size, size_t threads, size_t chunk_size, bool with_mesh, bool with_progress_on_cout) {
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

	// split into statements and function calls
	std::vector<statement_type> statements;
	std::map<std::string, function_def_type> functions;

	assigner<
		statement_type,
		function_def_type,
		decltype(statements),
		decltype(functions)> vis(
			statements,
			functions
		);

	std::for_each(statements_and_functions.begin(), statements_and_functions.end(), boost::apply_visitor(vis));

	std::vector<float> estimates(statements.size(), 1.f);
	ap = std::make_unique<application_progress>(estimates, pfn);

	function_arg_value_type threads_ = (int)threads;
	context["THREADS"] = threads_;

	function_arg_value_type chunk_size_ = (int)chunk_size;
	context["CHUNKSIZE"] = chunk_size_;

	function_arg_value_type vsize_ = size;
	context["VOXELSIZE"] = vsize_;

	execute_statements(context, functions, statements, with_progress_on_cout, ap, apfn);

	return context;
}

size_t padding_ = 32;
size_t get_padding() {
	return padding_;
}
void set_padding(size_t v) {
	padding_ = v;
}