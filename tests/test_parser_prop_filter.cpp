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

TEST(Filters, PropertyFilter) {
	std::stringstream sss;
	sss << "file = parse(\"duplex.ifc\")\n"
		<< "external = filter_properties(file, IsExternal=1)\n"
		<< "external_doors = create_geometry(external, include={\"IfcDoor\"})\n"
		<< "external_doors_voxels = voxelize(external_doors)\n"
		<< "nc = count_components(external_doors_voxels)\n"
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

	ASSERT_EQ(std::distance(f, first), size);

	boost::filesystem::current_path(boost::filesystem::path("..") / "tests" / "fixtures");

	// 0.5 sufficiently large to merge items
	auto scp = run(tree, 0.5, 1);

	ASSERT_EQ(scp.get_value<int>("nc"), 4);
}

#endif
#endif
