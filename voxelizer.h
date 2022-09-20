#ifndef VOXELIZER_H
#define VOXELIZER_H

#include "storage.h"
#include "polyfill.h"

#include <TopoDS_Face.hxx>
#include <TopLoc_Location.hxx>
#include <Poly_Triangulation.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Line.hxx>
#include <TopoDS.hxx>
#include <gp_Pln.hxx>
#include <BRepTools_WireExplorer.hxx>

#include <algorithm>
#include <exception>

class voxelizer {
private:
	const TopoDS_Shape& shape_;
	regular_voxel_storage* ds_;
	double eps_;

	bool generate_primitives_, use_scanline_;
public:
	voxelizer(const TopoDS_Shape& shape, regular_voxel_storage* ds, bool generate_primitives = true, bool use_scanline = true)
		: shape_(shape), ds_(ds), eps_(0.), generate_primitives_(generate_primitives), use_scanline_(use_scanline) {}

	double& epsilon() { return eps_; }

	void process_triangulated(TopoDS_Face& face) {
		TopLoc_Location loc;
		Handle_Poly_Triangulation tri = BRep_Tool::Triangulation(face, loc);

		if (tri.IsNull()) {
			// @todo the reason we don't triangulate here is for multithreading (I think)
			// this should be properly documented or changed.
			return;
			throw std::runtime_error("No triangulation!");
		} else {
			std::vector<gp_XYZ> xyzs;
			xyzs.reserve(tri->NbNodes());
			for (int i = 1; i <= tri->NbNodes(); ++i) {
				gp_XYZ xyz = tri->Node(i).Transformed(loc).XYZ();
				xyzs.push_back(xyz);
			}

			const Poly_Array1OfTriangle& triangles = tri->Triangles();
			for (int i = 1; i <= triangles.Length(); ++i) {

				double boxcenter[3];
				double boxhalfsize[3] = { ds_->voxel_size() / 2. + eps_ ,  ds_->voxel_size() / 2. + eps_ ,  ds_->voxel_size() / 2. + eps_ };
				double triverts[3][3];

				int n123[3];
				if (face.Orientation() == TopAbs_REVERSED)
					triangles(i).Get(n123[2], n123[1], n123[0]);
				else triangles(i).Get(n123[0], n123[1], n123[2]);

				typedef std::array<double, 3> ijk_t;
				ijk_t ijks[3];

				for (int i = 0; i < 3; ++i) {
					const gp_XYZ& p = xyzs[n123[i] - 1];

					ijks[i][0] = p.X();
					ijks[i][1] = p.Y();
					ijks[i][2] = p.Z();

					triverts[i][0] = p.X();
					triverts[i][1] = p.Y();
					triverts[i][2] = p.Z();
				}

				ijk_t minmax[2];
				minmax[0].fill(+std::numeric_limits<double>::infinity());
				minmax[1].fill(-std::numeric_limits<double>::infinity());

				for (int i = 0; i < 3; ++i) {
					for (int j = 0; j < 3; ++j) {
						if (ijks[i][j] < minmax[0][j]) {
							minmax[0][j] = ijks[i][j];
						}
						if (ijks[i][j] > minmax[1][j]) {
							minmax[1][j] = ijks[i][j];
						}
					}
				}

				size_t bounds[2][3];

				for (int i = 0; i < 2; ++i) {
					ds_->GetVoxelX(minmax[i][0], bounds[i][0]);
					ds_->GetVoxelY(minmax[i][1], bounds[i][1]);
					ds_->GetVoxelZ(minmax[i][2], bounds[i][2]);
				}

				for (int i = 0; i < 2; ++i) {
					const int d = i == 0 ? -1 : 1;
					for (int j = 0; j < 3; ++j) {
						bounds[i][j] += d;
					}
				}

				if (bounds[0][0] < 0) bounds[0][0] = 0;
				if (bounds[0][1] < 0) bounds[0][1] = 0;
				if (bounds[0][2] < 0) bounds[0][2] = 0;
				if (bounds[1][0] > ds_->GetNbX()) bounds[1][0] = ds_->GetNbX();
				if (bounds[1][1] > ds_->GetNbY()) bounds[1][1] = ds_->GetNbY();
				if (bounds[1][2] > ds_->GetNbZ()) bounds[1][2] = ds_->GetNbZ();

				BEGIN_LOOP(bounds[0][0], bounds[1][0], bounds[0][1], bounds[1][1], bounds[0][2], bounds[1][2])
					ds_->GetCenter(ijk.get<0>(), ijk.get<1>(), ijk.get<2>(), boxcenter[0], boxcenter[1], boxcenter[2]);
					if (ds_->Get(ijk)) {
						continue;
					}
					if (triBoxOverlap(boxcenter, boxhalfsize, triverts)) {
						ds_->Set(ijk);
					}
				END_LOOP;
			}
		}
	}

	bool is_ortho_polygon(TopoDS_Face& face) {
		// Single wire
		{
			TopExp_Explorer exp(face, TopAbs_WIRE);
			int wire_count = 0;
			for (; exp.More(); exp.Next(), ++wire_count) {}
			if (wire_count != 1) {
				return false;
			}
		}

		// Planar surface
		auto surf = BRep_Tool::Surface(face);
		if (surf->DynamicType() != STANDARD_TYPE(Geom_Plane)) {
			return false;
		}

		// Orthogonal direction
		gp_Dir d = opencascade::handle<Geom_Plane>::DownCast(surf)->Pln().Axis().Direction();
		double c = std::max({ std::abs(d.X()), std::abs(d.Y()), std::abs(d.Z()) });
		if (c < (1. - 1.e-7)) {
			return false;
		}

		// Linear edges
		{
			TopExp_Explorer exp(face, TopAbs_EDGE);
			for (; exp.More(); exp.Next()) {
				double _, __;
				auto curve = BRep_Tool::Curve(TopoDS::Edge(exp.Current()), _, __);
				if (curve->DynamicType() != STANDARD_TYPE(Geom_Line)) {
					return false;
				}
			}
		}

		// @todo Eliminate this requirement. See test_covering
		// Max four edges
		{
			TopExp_Explorer exp(face, TopAbs_EDGE);
			int edge_count = 0;
			for (; exp.More(); exp.Next(), ++edge_count) {}
			if (edge_count > 4) {
				return false;
			}
		}

		return true;
	}

	void process_scanline(TopoDS_Face& face) {
		auto surf = BRep_Tool::Surface(face);
		gp_Dir d = opencascade::handle<Geom_Plane>::DownCast(surf)->Pln().Axis().Direction();
		std::array<double, 3> abs_d = { d.X(), d.Y(), d.Z() };
		int d_idx = std::find_if(abs_d.begin(), abs_d.end(), [](double d) {
			return (std::abs(d) >= (1. - 1.e-7));
		}) - abs_d.begin();

		std::vector< std::array<int, 2> > points;
		TopoDS_Iterator it(face);
		const TopoDS_Wire& w = TopoDS::Wire(it.Value());
		BRepTools_WireExplorer exp(w, face);

		std::vector<point_t> verts;
		size_t fixed;

		auto ds_extents = ds_->extents();
		std::vector<size_t> extents{ ds_extents.get<0>(), ds_extents.get<1>(), ds_extents.get<2>() };
		extents.erase(extents.begin() + d_idx);

		for (; exp.More(); exp.Next()) {
			gp_Pnt p = BRep_Tool::Pnt(exp.CurrentVertex());
			double x, y, z;
			std::vector<size_t> ijk(3);
			std::tie(x, y, z) = std::make_tuple(p.X(), p.Y(), p.Z());
			ds_->GetVoxelX(x, ijk[0]);
			ds_->GetVoxelY(y, ijk[1]);
			ds_->GetVoxelZ(z, ijk[2]);

			if (ijk[0] > ds_extents.get<0>() || ijk[1] > ds_extents.get<1>() || ijk[2] > ds_extents.get<2>()) {
				std::cout << 1;
			}

			fixed = ijk[d_idx];

			ijk.erase(ijk.begin() + d_idx);
			std::array<int, 2> ij;
			std::copy_n(ijk.begin(), 2, ij.begin());

			verts.push_back(point_t(ij[0], ij[1]));
		}

		typedef std::pair<int, int> xspan;

		struct span_collection {
			int y;
			size_t num_xs;
			xspan* xs;
		};

		int vminy = std::numeric_limits<int>::max();
		int vmaxy = std::numeric_limits<int>::min();
		std::for_each(verts.begin(), verts.end(), [&vminy, &vmaxy](const auto& v) {
			const int& y = v.template get<1>();

			if (y < vminy) vminy = y;
			if (y > vmaxy) vmaxy = y;
		});

		const size_t num_y_spans = vmaxy - vminy + 1;
		std::vector<span_collection>** spans = new std::vector<span_collection>*[num_y_spans];
		memset(spans, 0, sizeof(std::vector<span_collection>*) * num_y_spans);

		struct fill_data {
			abstract_voxel_storage* voxels;
			std::vector<span_collection>** spans;
			size_t xmin, xmax, ymin, ymax, ymin2;
			int fixed_dimension;
			size_t fixed;
		};

		fill_data fd{ ds_, spans, (size_t)-1, 0, (size_t)-1, 0, (size_t)vminy, d_idx, fixed };

		// std::cerr << "Fixed: " << fixed << std::endl;

		fill_poly(verts, [](int y_, int npairs, int* xses, void* data) {
			size_t y = (size_t)y_;
			fill_data* fd = (fill_data*)data;

			auto xss = new xspan[npairs];
			if (fd->spans[y - fd->ymin2] == nullptr) {
				fd->spans[y - fd->ymin2] = new std::vector<span_collection>;
			}
			fd->spans[y - fd->ymin2]->push_back({ (int)y, (size_t)npairs, xss });

			for (int i = 0; i < npairs; ++i) {
				xss[i].first = xses[2 * i];
				xss[i].second = xses[2 * i + 1];
			}

			const size_t smallest_x = (size_t) xses[0];
			const size_t largest_x = (size_t) xses[npairs * 2 - 1];

			if (smallest_x < fd->xmin) {
				fd->xmin = smallest_x;
			}

			if (largest_x > fd->xmax) {
				fd->xmax = largest_x;
			}

			if (y < fd->ymin) {
				fd->ymin = y;
			}

			if (y > fd->ymax) {
				fd->ymax = y;
			}
		}, &fd);

		vec_n<3, size_t> max_cs;
		// size_t max_cs_x, max_cs_y, max_cs_z;
		bool* as_prim;

		auto stored_as_prim = [&](const vec_n<3, size_t>& c) {
			if ((c >= max_cs).any()) {
				return false;
			}
			return as_prim[c.get(0) + (max_cs.get(0) * c.get(1)) + (max_cs.get(0) * max_cs.get(1) * c.get(2))];
		};

		auto d2_to_voxel = [=](int x, int y) {
			// tfk: todo optimize: static array with fixed, and a x_index and y_index
			std::vector<size_t> args{ (size_t)x, (size_t)y, 0 };
			args.insert(args.begin() + fd.fixed_dimension, fd.fixed);
			return vec_n<3, size_t>(args[0], args[1], args[2]);
		};

		if (generate_primitives_) {

			std::array< vec_n<3, size_t>, 2 > range;
			size_t yminmax[2] = { fd.ymin, fd.ymax };
			size_t xminmax[2] = { fd.xmin, fd.xmax };
			for (size_t i = 0; i < 2; ++i) {
				size_t xy_mm[2] = { xminmax[i], yminmax[i] };
				size_t xy_mm_i = 0;
				for (size_t j = 0; j < 3; ++j) {
					if (j == d_idx) {
						range[i].get(j) = fixed;
					} else {
						range[i].get(j) = xy_mm[xy_mm_i++];
					}
				}
			}

			chunked_voxel_storage<bit_t>* cds = (chunked_voxel_storage<bit_t>*) ds_;
			auto cs = cds->empty_chunks_in(range, d_idx);
			std::array< vec_n<3, size_t>, 2 > cext;
			std::array<size_t, 2> c1_2d, c2_2d;

			for (auto& c : cs) {
				max_cs = max_cs.maximum(c + 1U);
			}

			size_t cs_amount = max_cs.get(0) * max_cs.get(1) * max_cs.get(2);
			if (cs_amount) {
				as_prim = new bool[cs_amount] {false};
			}

			auto voxel_to_2d = [=](const vec_n<3, size_t> d3) {
				std::array<size_t, 2> r;
				size_t j = 0;
				for (size_t i = 0; i < 3; ++i) {
					if (i == d_idx) {
						continue;
					}
					r[j++] = d3.get(i);
				}
				return r;
			};

			for (auto& c : cs) {
				cds->chunk_extents(c, cext);
				c1_2d = voxel_to_2d(cext[0]);
				c2_2d = voxel_to_2d(cext[1]);

				bool contained = true;

				for (size_t y = c1_2d[1]; y <= c2_2d[1]; ++y) {
					const auto& spsp = spans[y - vminy];
					bool contained_in_span = false;

					if (spsp == nullptr) {
						continue;
					}

					const auto& sps = *spsp;

					for (auto& sp : sps) {
						for (size_t i = 0; i < sp.num_xs; ++i) {
							const int& x0 = sp.xs[i].first;
							const int& x1 = sp.xs[i].second;

							if ((int)c1_2d[0] >= x0 && (int)c2_2d[0] <= x1) {
								contained_in_span = true;
							}
						}
					}

					if (!contained_in_span) {
						contained = false;
						break;
					}
				}
				if (contained) {
					if (cds->create_plane_primitive(c, d_idx, fixed % cds->chunk_size())) {
						as_prim[c.get(0) + (max_cs.get(0) * c.get(1)) + (max_cs.get(0) * max_cs.get(1) * c.get(2))] = true;
					}
				}
			}

		}

		for (size_t i = 0; i < num_y_spans; ++i) {
			const auto& spsp = spans[i];

			if (spsp == nullptr) {
				continue;
			}

			const auto& sps = *spsp;

			for (auto& sp : sps) {
				for (size_t s = 0; s < sp.num_xs; ++s) {
					// nb: the x-spans are inclusive.
					for (int x = sp.xs[s].first; x <= sp.xs[s].second; ++x) {
						bool stored = generate_primitives_;
						auto d3 = d2_to_voxel(x, sp.y);
						if (stored) {
							chunked_voxel_storage<bit_t>* cds = (chunked_voxel_storage<bit_t>*) ds_;
							auto chunk = d3 / cds->chunk_size();
							stored = stored_as_prim(chunk);
						}
						if (!stored) {
							ds_->Set(d3);
						}
					}
				}

				delete[] sp.xs;
			}

			delete spsp;
		}

		delete[] spans;
	}

	void Convert() {
		for (TopExp_Explorer exp(shape_, TopAbs_FACE); exp.More(); exp.Next()) {
			TopoDS_Face face = TopoDS::Face(exp.Current());
			if (use_scanline_ && is_ortho_polygon(face)) {
				process_scanline(face);
			} else {
				process_triangulated(face);
			}
		}
	}
};

#endif
