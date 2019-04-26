#include "../polyfill.h"

#include <gtest/gtest.h>

#include <vector>
#include <array>

class PolyfillTest : public ::testing::Test {
protected:
	size_t W_, H_;
	char* data;
public:
	PolyfillTest(size_t W = 50, size_t H = 30)
		: W_(W), H_(H)
	{
		data = new char[W * H]{};
		clear();
	}

	void clear() {
		for (size_t i = 0; i < W_ * H_; ++i) {
			data[i] = ' ';
		}
	}

	void fill(const std::vector<point_t>& verts) {
		fill_poly(verts, [](int y, int npairs, int* xses, void * d) {
			PolyfillTest* self = (PolyfillTest*)d;
			for (int i = 0; i < npairs; ++i) {
				for (int x = xses[i * 2 + 0]; x <= xses[i * 2 + 1]; ++x) {
					self->get(x, y) = 'x';
				}
			}
		}, this);
	}

	char& get(int x, int y) {
		return data[W_ * y + x];
	}

	void dump() {
		std::cerr << "  ";
		for (size_t i = 0; i < W_; ++i) {
			std::cerr << (i % 10);
		}
		std::cerr << std::endl;
		std::cerr << " +";
		for (size_t i = 0; i < W_; ++i) {
			std::cerr << "-";
		}
		std::cerr << "+";
		std::cerr << std::endl;
		for (size_t i = 0; i < H_; ++i) {
			std::cerr << (i % 10) << "|";
			for (size_t j = 0; j < W_; ++j) {
				std::cerr << get(j, i);
			}
			std::cerr << "|";
			std::cerr << std::endl;
		}
		std::cerr << " +";
		for (size_t i = 0; i < W_; ++i) {
			std::cerr << "-";
		}
		std::cerr << "+";
		std::cerr << std::endl;
		std::cerr << std::endl;
	}

	size_t W() const { return W_; }
	size_t H() const { return H_; }
};

TEST_F(PolyfillTest, RectanglePolygon) {
	const std::vector<point_t> verts = { { 5, 5 },{ 5,25 },{ 45,25 },{ 30,5 } };

	fill(verts);
	dump();

	EXPECT_EQ(get(0, 0), ' ');
	EXPECT_EQ(get(5, 5), 'x');
	EXPECT_EQ(get(45, 25), 'x');
	EXPECT_EQ(get(46, 25), ' ');

	// Assert that scanline results in the same result as point containment tests
	for (size_t x = 0; x < W(); ++x) {
		for (size_t y = 0; y < H(); ++y) {
			// std::cerr << x << "," << y << std::endl;
			EXPECT_EQ(is_inside_poly(verts, x, y), get(x, y) == 'x');
		}
	}

	clear();
}


TEST_F(PolyfillTest, ConcaveOrthogonal) {
	const std::vector<point_t> verts = { { 5, 5 },{ 45, 5 },{ 45, 25 },{ 25,25 },{ 25,15 },{ 15,15 },{ 15,25 },{ 5,25 } };

	fill(verts);
	dump();

	EXPECT_EQ(get(20, 15), 'x');
	EXPECT_TRUE(is_inside_poly(verts, 20, 15));

	// Assert that scanline results in the same result as point containment tests
	for (size_t x = 0; x < W(); ++x) {
		for (size_t y = 0; y < H(); ++y) {
			EXPECT_EQ(is_inside_poly(verts, x, y), get(x, y) == 'x');
		}
	}
}


TEST_F(PolyfillTest, ConcaveNonOrtho) {
	const std::vector<point_t> verts = { { 5, 5 },{ 45, 5 },{ 45, 25 },{ 25,25 },{ 25,15 },{ 15,12 },{ 15,25 },{ 5,25 } };

	fill(verts);
	dump();

	// EXPECT_EQ(get(20, 15), ' ');
	// EXPECT_EQ(get(20, 14), 'x');

	// Assert that scanline results in the same result as point containment tests
	for (size_t x = 0; x < W(); ++x) {
		for (size_t y = 0; y < H(); ++y) {
			// std::cerr << x << " " << y << std::endl;
			EXPECT_EQ(is_inside_poly(verts, x, y), get(x, y) == 'x');
		}
	}
}

TEST_F(PolyfillTest, ConcaveNonOrtho2) {
	const std::vector<point_t> verts = { { 5, 5 },{ 45, 5 },{ 45, 25 },{ 25,25 },{ 25,12 },{ 15,15 },{ 15,25 },{ 5,25 } };

	fill(verts);
	dump();

	// EXPECT_EQ(get(20, 15), ' ');
	// EXPECT_EQ(get(20, 14), 'x');

	// Assert that scanline results in the same result as point containment tests
	for (size_t x = 0; x < W(); ++x) {
		for (size_t y = 0; y < H(); ++y) {
			EXPECT_EQ(is_inside_poly(verts, x, y), get(x, y) == 'x');
		}
	}
}

TEST(LineSegment, Intersection) {
	line_t ab{ point_t(0,0),point_t(10,0) };
	line_t cd{ point_t(5,-1),point_t(5,1) };
	ASSERT_TRUE(intersects(ab, cd));
}

TEST(LineSegment, Touching) {
	line_t ab{ point_t(0,0),point_t(10,0) };
	line_t cd{ point_t(5,0),point_t(5,5) };
	// Toughing segments need to report as non-intersecting
	ASSERT_FALSE(intersects(ab, cd));
}

TEST(LineSegment, Overlapping) {
	line_t ab{ point_t(0,0),point_t(10,0) };
	line_t cd{ point_t(3,0),point_t(6,0) };
	// Overlapping segments need to report as non-intersecting
	ASSERT_FALSE(intersects(ab, cd));
}


TEST(LineSegment, Colinear) {
	line_t ab{ point_t(0,0),point_t(10,0) };
	line_t cd{ point_t(11,0),point_t(12,0) };
	// Colinear disjoint segments need to report as non-intersecting
	ASSERT_FALSE(intersects(ab, cd));
}
