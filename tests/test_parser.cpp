#define BOOST_SPIRIT_DEBUG

#include "../voxec.h"
#include "../factory.h"

#include <gtest/gtest.h>

#include <fstream>

#ifdef WIN32
#define DIRSEP "\\"
#else
#define DIRSEP "/"
#endif

TEST(Voxelfile, Parser) {
	const std::string input_filename = ".." DIRSEP "tests" DIRSEP "fixtures" DIRSEP "voxelfile3.txt";

	factory::mmap("vox");

	std::ifstream ifs(input_filename.c_str(), std::ios::binary);
	ifs.seekg(0, ifs.end);
	size_t size = ifs.tellg();
	ifs.seekg(0, ifs.beg);
	ifs >> std::noskipws;

	boost::spirit::istream_iterator first(ifs), last;
	auto f = first;

	voxelfile_parser<decltype(first)> parser;
	
	std::vector<statement_or_function_def> tree;
	phrase_parse(first, last, parser, blank, tree);

	ASSERT_EQ(std::distance(f, first), size);

	// run(tree, 0.05, 1);
}
