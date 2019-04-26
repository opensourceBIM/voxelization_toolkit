#ifndef POLYFILL_H
#define POLYFILL_H

#include <array>
#include <vector>

#include "dim3.h"

typedef void(*polyfill_callback)(int, int, int*, void *);
typedef vec_n<2, int> point_t;
typedef std::array<point_t, 2> line_t;

void fill_poly(const std::vector<point_t>& verts, polyfill_callback callback, void *userData);
bool is_inside_poly(const std::vector<point_t>& verts, int x, int y);
bool intersects(const line_t& a, const line_t& b);
int triBoxOverlap(double boxcenter[3], double boxhalfsize[3], double triverts[3][3]);

#endif
