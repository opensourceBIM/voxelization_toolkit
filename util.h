#ifndef UTIL_H
#define UTIL_H

#include <cstddef>

template <typename T = size_t>
T integer_ceil_div(T a, T b) {
	return a / b + (a % b != 0);
}

template <typename T = size_t>
T integer_floor_div(T a, T b) {
	// @todo probably can be optimized.
	return a > 0 ? (a / b) : (a / b - (a % b != 0));
}

#endif