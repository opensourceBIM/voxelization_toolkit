// https://developer.blender.org/diffusion/B/browse/master/source/blender/blenlib/intern/math_geom.c;3b4a8f1cfa7339f3db9ddd4a7974b8cc30d7ff0b$2411

#include "polyfill.h"

#include <set>
#include <cmath>
#include <algorithm>

struct horizontal_segment {
	int y, x0, x1;
};

struct coord {
	int x, y;
};

struct intersection_node {
	int at_x;
	coord d;
};

bool operator <(const horizontal_segment& a, const horizontal_segment& b) {
	// interpret as tuple for lexicographic comparison
	return std::tie(a.y, a.x0, a.x1) < std::tie(b.y, b.x0, b.x1);
}

bool operator <(const intersection_node& a, const intersection_node& b) {
	// interpret as tuple for lexicographic comparison
	return std::tie(a.at_x, a.d.x, a.d.y) < std::tie(b.at_x, b.d.x, b.d.y);
}

int dot(const coord& a, const coord& b) {
	return a.x * b.x + a.y * b.y;
}

void get_minmax(const std::vector<point_t>& verts, int& ymin, int& ymax) {
	ymin = std::numeric_limits<int>::max();
	ymax = std::numeric_limits<int>::min();

	for (size_t i = 0; i < verts.size(); ++i) {
		if (verts[i].get<1>() < ymin) {
			ymin = verts[i].get<1>();
		}
		if (verts[i].get<1>() > ymax) {
			ymax = verts[i].get<1>();
		}
	}
}

void get_horizontal(const std::vector<point_t>& verts, std::set<horizontal_segment>& horizontal_segments) {
	size_t j = verts.size() - 1;
	for (size_t i = 0; i < verts.size(); i++) {
		const int& curx = verts[i].get<0>();
		const int& prex = verts[j].get<0>();

		const int& cury = verts[i].get<1>();
		const int& prey = verts[j].get<1>();

		if (cury == prey) {
			horizontal_segment span = { cury, curx, prex };
			if (span.x0 > span.x1) {
				std::swap(span.x1, span.x0);
			}
			horizontal_segments.insert(span);
		}

		j = i;
	}
}

void process_scanline(int pixel_y, const std::vector<point_t>& verts, const std::set<horizontal_segment>& horizontal_segments, std::vector<intersection_node>& node_x) {
	size_t j = verts.size() - 1;

	for (size_t i = 0; i < verts.size(); i++) {
		const int& curx = verts[i].get<0>();
		const int& prex = verts[j].get<0>();

		const int& cury = verts[i].get<1>();
		const int& prey = verts[j].get<1>();

		if ((cury <= pixel_y && prey >= pixel_y) ||
			(prey <= pixel_y && cury >= pixel_y)) {
			if (cury == prey) {
				// node_x.push_back({ curx,{ prex, prey } });
				// node_x.push_back({ prex,{ curx, cury } });
			} else {
				const int X = (int)std::lround(curx +
					((double)(pixel_y - cury) / (cury - prey)) *
					(curx - prex));

				node_x.push_back({ X,{ pixel_y == cury ? prex : curx, pixel_y == cury ? prey : cury } });
			}
		}

		j = i;
	}

	std::sort(node_x.begin(), node_x.end());

	/* remove duplicates for double registration of corner points */

	for (size_t i = 1; i < node_x.size(); ) {
		if (node_x[i].at_x == node_x[i - 1].at_x) {
			
			for (const auto& v : verts) {
				if (v.get<1>() == pixel_y && v.get<0>() == node_x[i].at_x) {
					// if both other ends of the segment are on the same side of the scanline, a ray would have passed twice
					if ((node_x[i].d.y > pixel_y) != (node_x[i - 1].d.y > pixel_y)) {
						node_x.erase(node_x.begin() + i);
						if (node_x.size() == i) {
							break;
						}
						continue;
					}
				}
			}

			/* if (dot(node_x[i].d, node_x[i - 1].d) > 0) {
				node_x.erase(node_x.begin() + i);
				continue;
			} */
		}
		i += 1;
	}

	/* work-around for horizontal segments */

	for (int i = 1; i < (int)node_x.size() - 2; ) {
		horizontal_segment span = { pixel_y, node_x[i].at_x, node_x[i + 1].at_x };
		if (horizontal_segments.find(span) != horizontal_segments.end()) {
			node_x.erase(node_x.begin() + i, node_x.begin() + i + 2);
		} else {
			i += 2;
		}
	}
}

#include <iostream>

void fill_poly(const std::vector<point_t>& verts, polyfill_callback callback, void *userData) {
	/* Adapted from Darel Rex Finley, 2007 */

	/*
	std::cerr << "poly: ";
	for (auto& p : verts) {
		std::cerr << p.format() << " ";
	}
	std::cerr << std::endl;
	*/

	int ymin, ymax;
	get_minmax(verts, ymin, ymax);
	
	std::vector<intersection_node> node_x;
	node_x.reserve(verts.size() * 2);

	/* Horizontal segments are treated differently */

	std::set<horizontal_segment> horizontal_segments;
	get_horizontal(verts, horizontal_segments);
	
	/* Loop through the rows of the image. */

	for (int pixel_y = ymin; pixel_y <= ymax; pixel_y++) {

		/* Build a list of nodes (intersections with scanline). */

		process_scanline(pixel_y, verts, horizontal_segments, node_x);

		/* Fill the pixels between node pairs. */

		std::vector<int> node_x_x(node_x.size());
		std::transform(node_x.begin(), node_x.end(), node_x_x.begin(), [](auto& s) {
			return s.at_x;
		});

		if (node_x.size() >= 2) {
			int nspans = node_x.size() / 2;
			callback(pixel_y, nspans, node_x_x.data(), userData);
		}

		node_x.clear();
	}
}


bool is_inside_poly(const std::vector<point_t>& verts, int x, int y) {
	int ymin, ymax;
	get_minmax(verts, ymin, ymax);

	std::vector<intersection_node> node_x;
	node_x.reserve(verts.size() * 2);

	/* Horizontal segments are treated differently */

	std::set<horizontal_segment> horizontal_segments;
	get_horizontal(verts, horizontal_segments);

	/* Build a list of nodes (intersections with scanline). */

	process_scanline(y, verts, horizontal_segments, node_x);

	bool inside = false;

	for (auto& n : node_x) {
		if (n.at_x == x) {
			// on
			return true;
		}
		if (n.at_x > x) {
			inside = !inside;
		}
	}

	return inside;
}

// Not entirely the same as Bryce Boe's
// Do not have a lot of confidence in this yet.
bool ccw(const point_t& A, const point_t& B, const point_t& C) {
	auto ac = C - A;
	auto ab = B - A;
	auto d = ac.cross(ab);
	return d > 0;
}

// https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
bool intersects(const line_t& a, const line_t& b) {
	return (ccw(a[0], b[0], b[1]) != ccw(a[1], b[0], b[1])) && (ccw(a[0], a[1], b[0]) != ccw(a[0], a[1], b[1]));
}

// https://github.com/pgkelley4/line-segments-intersect/blob/master/js/line-segments-intersect.js
bool intersects_float(const line_t& a, const line_t& b) {
	const double eps = 1.e-9;
	const double zero = -eps;
	const double one = 1. - eps;

	auto p = a[0].as<double>();
	auto p2 = a[1].as<double>();
	auto q = b[0].as<double>();
	auto q2 = b[1].as<double>();

	auto r = p2 - p;
	auto s = q2 - q;

	auto u = (q, p).cross(r);
	auto d = r.cross(s);

	if (u < eps && d < eps) {

	}

	if (d < eps) {
		return false;
	}

	auto t = u / d;
	auto v = (q - p).cross(s) / d;

	return (t >= zero) && (t <= one) && (u >= zero) && (u <= one);
}
