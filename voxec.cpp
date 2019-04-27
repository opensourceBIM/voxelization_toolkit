// @todo erode for angle slabs
// @todo finish memory mapped chunked storage and make default

#include "voxec.h"

voxel_operation_map::map_t& voxel_operation_map::map() {
	static voxel_operation_map::map_t m;
	static bool initialized = false;
	if (!initialized) {
#ifdef WITH_IFC
		m.insert(std::make_pair(std::string("parse"), &instantiate<op_parse_ifc_file>));
		m.insert(std::make_pair(std::string("create_geometry"), &instantiate<op_create_geometry>));
#endif
		m.insert(std::make_pair(std::string("voxelize"), &instantiate<op_voxelize>));
		m.insert(std::make_pair(std::string("fill_gaps"), &instantiate<op_fill_gaps>));
		m.insert(std::make_pair(std::string("offset"), &instantiate<op_offset>));
		m.insert(std::make_pair(std::string("offset_xy"), &instantiate<op_offset_xy>));
		m.insert(std::make_pair(std::string("outmost"), &instantiate<op_outmost>));
		m.insert(std::make_pair(std::string("count"), &instantiate<op_count>));
		m.insert(std::make_pair(std::string("volume"), &instantiate<op_volume>));
		m.insert(std::make_pair(std::string("volume2"), &instantiate<op_volume_2>));
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

class print_visitor {
public:
	typedef void result_type;

	void operator()(int v) const {
		std::cerr << "(integer:)" << v << std::endl;
	}

	void operator()(double v) const {
		std::cerr << "(real:)" << v << std::endl;
	}

	void operator()(const std::string& v) const {
		if (v.at(0) == '"') {
			std::cerr << "(string:)" << v.substr(1, v.size() - 2) << std::endl;
		}
	}

	void operator()(const function_arg_value_type& v) const {
		v.apply_visitor(*this);
	}

	void operator()(std::vector<IfcParse::IfcFile*> const v) const {	}

	void operator()(geometry_collection_t* const v) const {	}

	void operator()(abstract_voxel_storage* const v) const { }
};


scope_map run(const std::vector<statement_type>& statements, double size, size_t threads, size_t chunk_size, bool with_mesh) {
	scope_map context;

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

		std::cerr << "> " << st << std::endl;
		voxel_operation_runner op(st);
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
				std::cerr << "@" << n << " ; chunked voxels: "
					<< left.format() << " - " << right.format() 
					<< " ; count: " << voxels->count() 
					<< " ; bounds: " << voxels->bounds()[0].format()
						<< " - " << voxels->bounds()[1].format() << std::endl
					<< " ; world: " << left_world.format()
						<< " - " << right_world.format() << std::endl;
			}
		}

		n++;

		print_visitor v;
		context[statements.back().assignee()].apply_visitor(v);
	}

	return context;
}
