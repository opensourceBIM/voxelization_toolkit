%include "std_string.i"
%include "exception.i"

%{
#include <Python.h>
#include <vector>
#include <BRepTools_ShapeSet.hxx>
#include "voxec.h"
%}

%exception {
	try {
		$action
	} catch(const std::runtime_error& e) {
		SWIG_exception(SWIG_RuntimeError, e.what());
	} catch(...) {
		SWIG_exception(SWIG_RuntimeError, "An unknown error occurred");
	}
}

%typemap(out) std::array< vec_n<3, long>, 2 > {
    $result = PyTuple_New(2);
	for (unsigned j = 0; j < 2; ++j) {
		auto coord = PyTuple_New(3);
		PyTuple_SetItem($result, j, coord);
		for (unsigned i = 0; i < 3; ++i) {
			PyTuple_SetItem(coord, i, PyLong_FromLong($1[j].get(i)));
		}
	}
}

%typemap(out) std::vector<abstract_voxel_storage*> {
    $result = PyTuple_New($1.size());
	for (unsigned i = 0; i < $1.size(); ++i) {
		PyTuple_SetItem($result, i, SWIG_NewPointerObj(SWIG_as_voidptr(((std::vector<abstract_voxel_storage*>&)result)[i]), SWIGTYPE_p_abstract_voxel_storage, 0));
	}
}

%typemap(out) std::pair<void*, size_t> {
    $result = PyBytes_FromStringAndSize((char* const)$1.first, $1.second);
}

%extend abstract_voxel_storage {
	std::vector<abstract_voxel_storage*> components() const {
		std::vector<abstract_voxel_storage*> comps;
		connected_components((regular_voxel_storage*)$self, [&comps](regular_voxel_storage* c) {
			comps.push_back(c->copy());
		});
		return comps;
	}
	PyObject* get(long i, long j, long k) const {
		auto acvs_voxels = dynamic_cast<abstract_chunked_voxel_storage const*>($self);
		if (!acvs_voxels) {
			throw std::runtime_error("Unsupported storage");
		}
		auto ijk_long = (make_vec<long>(i, j, k) - (acvs_voxels->grid_offset() * acvs_voxels->chunk_size()).as<long>());
		auto ijk = ijk_long.as<size_t>();
		if (!(ijk_long > 0).all() || !(ijk < $self->extents()).all()) {
			if ($self->value_bits() == 1) {
				return PyBool_FromLong(0);			
			} else if ($self->value_bits() == 32) {
				return PyLong_FromLong(0);
			} else {
				Py_INCREF(Py_None);
				return Py_None;
			}
		}
		if ($self->value_bits() == 1) {
			return PyBool_FromLong(self->Get(ijk));
		} else if ($self->value_bits() == 8) {
			uint8_t v;
			$self->Get(ijk, &v);
			return PyLong_FromLong(v);
		} else if ($self->value_bits() == 32) {
			uint32_t v;
			$self->Get(ijk, &v);
			return PyLong_FromLong(v);
		} else if ($self->value_bits() == sizeof(normal_and_curvature<int16_t>) * 8) {
			normal_and_curvature_t::storage_type v;
			$self->Get(ijk, &v);
			if (!v) {
				return SWIG_Py_Void();
			} else {
				auto vf = v.convert<float>();
				auto tup = PyTuple_New(4);
				for (size_t i = 0; i < 4; ++i) {
					PyTuple_SetItem(tup, i, PyFloat_FromDouble(vf.nxyz_curv[i]));
				}
				return tup;
			}
		} else {
			throw std::runtime_error("Unsupported data type, size: " + std::to_string($self->value_bits()));
		}
	}
	std::pair<void*, size_t> get_domain_buffer() const {
		auto acvs_voxels = dynamic_cast<abstract_chunked_voxel_storage const*>($self);
		if (!acvs_voxels) {
			throw std::runtime_error("Unsupported");
		}
		auto offs = acvs_voxels->grid_offset() * acvs_voxels->chunk_size();
		size_t oi, oj, ok;
		offs.tie(oi, oj, ok);
		auto a = $self->bounds();
		auto extents = a[1] - a[0] + 1;
		auto num_elements = extents.prod();

		size_t elem_size_bytes;
		if ($self->value_bits() == 1) {
			elem_size_bytes = 1;
		} else if ($self->value_bits() == 8) {
			elem_size_bytes = 1;
		} else if ($self->value_bits() == 32) {
			elem_size_bytes = 4;
		} else if ($self->value_bits() == sizeof(normal_and_curvature<int16_t>) * 8) {
			elem_size_bytes = 16;
		} else {
			throw std::runtime_error("Unsupported data type, size: " + std::to_string($self->value_bits()));
		}
		
		// @nb we choose float as that has the highest alignment requirements most likely 
		void* data = new float[num_elements * elem_size_bytes / sizeof(float)];
		
		size_t index = 0;
        BEGIN_LOOP_I2(a[0], a[1])
			if (self->value_bits() == 1) {
				((uint8_t*) (data))[index++] = self->Get(ijk) ? 1 : 0;
			} else if (self->value_bits() == 8) {
				uint8_t v;
				self->Get(ijk, &v);
				((uint8_t*) (data))[index++] = v;
			} else if (self->value_bits() == 32) {
				uint32_t v;
				self->Get(ijk, &v);
				((uint32_t*) (data))[index++] = v;
            } else if (self->value_bits() == sizeof(normal_and_curvature<int16_t>) * 8) {
                normal_and_curvature_t::storage_type v;
                self->Get(ijk, &v);
                auto vf = v.convert<float>();
                for (size_t l = 0; l < 4; ++l) {
                    ((float*)(data))[index++] = vf.nxyz_curv[l];
                }
            }
        END_LOOP;
        return { data, num_elements * elem_size_bytes };
	}
	PyObject* get_domain() const {
		auto acvs_voxels = dynamic_cast<abstract_chunked_voxel_storage const*>($self);
		if (!acvs_voxels) {
			throw std::runtime_error("Unsupported");
		}
		auto offs = acvs_voxels->grid_offset() * acvs_voxels->chunk_size();
		auto a = self->bounds();
        long oi, oj, ok;
        (offs + a[0].as<long>()).tie(oi, oj, ok);
        auto extents = a[1] - a[0] + 1;
		PyObject* idim = PyTuple_New(extents.get<0>());
		for (long i = 0; i < extents.get<0>(); ++i) {
			PyObject* jdim = PyTuple_New(extents.get<1>());
			PyTuple_SetItem(idim, i, jdim);
			for (long j = 0; j < extents.get<1>(); ++j) {
				PyObject* kdim = PyTuple_New(extents.get<2>());
				PyTuple_SetItem(jdim, j, kdim);
				for (long k = 0; k < extents.get<2>(); ++k) {
					PyTuple_SetItem(kdim, k, abstract_voxel_storage_get__SWIG_0($self, i + oi, j + oj, k + ok));
				}
			}
		}
		return idim;
	}
	bool set(long i, long j, long k, PyObject* v) {
		auto acvs_voxels = dynamic_cast<abstract_chunked_voxel_storage const*>($self);
		if (!acvs_voxels) {
			throw std::runtime_error("Unsupported storage");
		}
		auto ijk_long = (make_vec<long>(i, j, k) - (acvs_voxels->grid_offset() * acvs_voxels->chunk_size()).as<long>());
		auto ijk = ijk_long.as<size_t>();
		if (!(ijk_long > 0).all() || !(ijk < $self->extents()).all()) {
			return false;
		}
		if ($self->value_bits() == 1) {
			bool b = PyObject_IsTrue(v) == 1;
			$self->Set(ijk, &b);
			return true;
		}
		return false;
	}
	PyObject* get(double x, double y, double z) const {
		auto acvs_voxels = dynamic_cast<abstract_chunked_voxel_storage const*>($self);
		if (!acvs_voxels) {
			throw std::runtime_error("Unsupported");
		}
		auto offs = acvs_voxels->grid_offset() * acvs_voxels->chunk_size();
		size_t i, j, k;
		$self->GetVoxelX(x, i);
		$self->GetVoxelX(y, j);
		$self->GetVoxelX(z, k);
		return abstract_voxel_storage_get__SWIG_0($self, i + offs.get<0>(), j + offs.get<1>(), k + offs.get<2>());
	}
	std::array< vec_n<3, long>, 2 > world_bounds() const {
		auto acvs_voxels = dynamic_cast<abstract_chunked_voxel_storage const*>($self);
		if (!acvs_voxels) {
			throw std::runtime_error("Unsupported");
		}
		auto offs = acvs_voxels->grid_offset() * acvs_voxels->chunk_size();
		auto a = $self->bounds();
		return {a[0].as<long>() + offs, a[1].as<long>() + offs};
	}
}

%newobject abstract_voxel_storage::boolean_union;
%newobject abstract_voxel_storage::boolean_subtraction;
%newobject abstract_voxel_storage::boolean_intersection;

%include "storage.h"

%{
void populate_scope(scope_map& scope, const argument_spec& spec, PyObject* obj) {
	std::set<std::string> type_opts;
	std::vector<std::string> tokens;
	boost::split(tokens, spec.type, boost::is_any_of("|"));
	type_opts.insert(tokens.begin(), tokens.end());

	// Check if the argument is a Python float
	if (PyFloat_Check(obj) && type_opts.find("real") != type_opts.end()) {
		auto value = PyFloat_AsDouble(obj);
		scope[spec.name] = value;
	} else if (PyLong_Check(obj) && type_opts.find("integer") != type_opts.end()) {
		int value = (int) PyLong_AsLong(obj);
		scope[spec.name] = value;
	} else if (PyUnicode_Check(obj) && type_opts.find("string") != type_opts.end()) {
		auto value = std::string(PyUnicode_AsUTF8(obj));
		scope[spec.name] = value;
	} else if (PyUnicode_Check(obj) && type_opts.find("surfaceset") != type_opts.end()) {
		auto str = std::string(PyUnicode_AsUTF8(obj));
		std::stringstream stream(str);
		BRepTools_ShapeSet shapes;
		shapes.Read(stream);
		const TopoDS_Shape& shp = shapes.Shape(shapes.NbShapes());
		BRepMesh_IncrementalMesh(shp, 0.001);
		TopoDS_Compound comp;
		if (shp.ShapeType() == TopAbs_COMPOUND) {
			comp = TopoDS::Compound(shp);
		} else {
			BRep_Builder BB;
			BB.MakeCompound(comp);
			BB.Add(comp, shp);
		}
		scope[spec.name] = new geometry_collection_t{ {{nullptr, 0}, comp} };
	} else if (type_opts.find("voxels") != type_opts.end()) {
		void* argp1;
		auto res1 = SWIG_ConvertPtr(obj, &argp1, SWIGTYPE_p_abstract_voxel_storage, 0);
		if (!SWIG_IsOK(res1)) {
			throw std::runtime_error("Argument type mismatch for argument '" + spec.name + "' of type '" + spec.type + "'");
		}
		auto value = reinterpret_cast<abstract_voxel_storage*>(argp1);
		scope[spec.name] = value;
	} else if (type_opts.find("sequence") != type_opts.end()) {
		throw std::runtime_error("Not supported");
	} else if (type_opts.find("ifcfile") != type_opts.end()) {
		throw std::runtime_error("Not supported");
	} else {
		throw std::runtime_error("Argument type mismatch for argument '" + spec.name + "' of type '" + spec.type + "'");
	}
}
%}

%typemap(out) symbol_value {
    if (const int* val = get_value_opt_<int>($1)) {
        $result = PyLong_FromLong(*val);
    } else if (const double* val = get_value_opt_<double>($1)) {
        $result = PyFloat_FromDouble(*val);
    } else if (const std::string* val = get_value_opt_<std::string>($1)) {
        $result = PyUnicode_FromString(val->c_str());
    } else if (abstract_voxel_storage*const* val = get_value_opt_<abstract_voxel_storage*>($1)) {
		$result = SWIG_NewPointerObj(SWIG_as_voidptr(*val), SWIGTYPE_p_abstract_voxel_storage, SWIG_POINTER_OWN);
	} else if ($1.which() == 0) {
		Py_INCREF(Py_None);
		$result = Py_None;
	} else {
        PyErr_SetString(PyExc_TypeError, "Unsupported type for symbol_value");
        SWIG_fail;
    }
}

%inline %{
struct context {
	scope_map scope;
	context() {}

	void set(const std::string& name, int v) {
		scope[name] = v;
	}
	void set(const std::string& name, double v) {
		scope[name] = v;
	}
};

symbol_value run_(const std::string& name, PyObject *args, PyObject *kwargs, context* ctx = nullptr, bool silent=false) {
	scope_map scope;
	if (ctx) {
		scope = ctx->scope;
	}

	voxel_operation* op = voxel_operation_map::create(name);
	op->silent = silent;

	// Process positional arguments
	auto it = op->arg_names().begin();
	for (Py_ssize_t i = 0; i < PyTuple_Size(args) && it < op->arg_names().end(); ++i, ++it) {
		PyObject* obj = PyTuple_GetItem(args, i);
		populate_scope(scope, *it, obj);
	}

	// Process keyword arguments
	PyObject *key, *value;
	Py_ssize_t pos = 0;
	while (PyDict_Next(kwargs, &pos, &key, &value)) {
		if (PyUnicode_Check(key)) {
			auto keyname = std::string(PyUnicode_AsUTF8(key));
			auto jt = std::find_if(it, op->arg_names().end(), [&keyname](const auto& s) { return s.name == keyname; });
			if (jt == op->arg_names().end()) {
				throw std::runtime_error("Argument named '" + keyname + "' not found or already specified as positional argument");
			}
			populate_scope(scope, *jt, value);
		} else {
			throw std::runtime_error("Non-string keyword argument");
		}
	}

	auto r = op->invoke(scope);
	delete op;
	return r;
}

%}

%module voxec %{
%}