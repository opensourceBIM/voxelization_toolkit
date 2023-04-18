#ifdef WITH_IFC
#ifndef IFCOPENSHELL_05

#include "../voxec.h"

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#ifdef WIN32
#define DIRSEP "\\"
#else
#define DIRSEP "/"
#endif	

TEST(Assert, Integer) {
	std::stringstream sss;
	sss << "file = parse(\"duplex.ifc\")\n"
		<< "geom = create_geometry(file)\n"
		<< "voxels = voxelize(geom)\n"
		<< "nc = count(voxels)\n"
		<< "assert(nc)\n"
		;
	
	sss.seekg(0, sss.end);
	size_t size = sss.tellg();
	sss.seekg(0, sss.beg);
	sss >> std::noskipws;

	boost::spirit::istream_iterator first(sss), last;
	auto f = first;

	voxelfile_parser<decltype(first)> parser;

	std::vector<statement_or_function_def> tree;
	phrase_parse(first, last, parser, blank, tree);

	boost::filesystem::current_path(boost::filesystem::path("..") / "tests" / "fixtures");
	run(tree, 0.5, 1);
}

TEST(Assert, Voxels) {
	std::stringstream sss;
	sss << "file = parse(\"duplex.ifc\")\n"
		<< "geom = create_geometry(file)\n"
		<< "voxels = voxelize(geom)\n"
		<< "assert(voxels)\n"
		;

	sss.seekg(0, sss.end);
	size_t size = sss.tellg();
	sss.seekg(0, sss.beg);
	sss >> std::noskipws;

	boost::spirit::istream_iterator first(sss), last;
	auto f = first;

	voxelfile_parser<decltype(first)> parser;

	std::vector<statement_or_function_def> tree;
	phrase_parse(first, last, parser, blank, tree);

	run(tree, 0.5, 1);
}

TEST(Assert, EmptyVoxels) {
	std::stringstream sss;
	sss << "file = parse(\"duplex.ifc\")\n"
		<< "geom = create_geometry(file)\n"
		<< "voxels = voxelize(geom)\n"
		<< "empty = constant_like(voxels, 0)\n"
		<< "assert(empty)\n"
		;

	sss.seekg(0, sss.end);
	size_t size = sss.tellg();
	sss.seekg(0, sss.beg);
	sss >> std::noskipws;

	boost::spirit::istream_iterator first(sss), last;
	auto f = first;

	voxelfile_parser<decltype(first)> parser;

	std::vector<statement_or_function_def> tree;
	phrase_parse(first, last, parser, blank, tree);

	EXPECT_THROW(run(tree, 0.5, 1), assertion_error);
}

#endif
#endif
