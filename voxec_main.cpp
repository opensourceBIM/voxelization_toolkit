#include "voxec.h"

#include <boost/program_options.hpp>
#include <boost/optional.hpp>

#include <fstream>
#include <iostream>

namespace po = boost::program_options;

int main(int argc, char** argv) {
	
	po::options_description opts("Command line options");
	opts.add_options()
		("threads,t", po::value<size_t>(), "number of parallel processing threads")
		("size,d", po::value<double>(), "voxel size in meters")
		("chunk,c", po::value<size_t>(), "chunk size in number of voxels")
		("mmap,m", "use memory-mapped files instead of pure RAM")
		("mesh", "emit obj mesh for the last instruction")
		("input-file", po::value<std::string>(), "input IFC file")
		("output-file", po::value<std::string>(), "output voxel file");

	po::positional_options_description positional_options;

	positional_options.add("input-file", 1);
	positional_options.add("output-file", 1);

	po::variables_map vmap;
	try {
		po::store(po::command_line_parser(argc, argv).
			options(opts).positional(positional_options).run(), vmap);
	} catch (const std::exception& e) {
		std::cerr << "[Error] " << e.what() << "\n\n";
		return 1;
	}

	if (!vmap.count("input-file")) {
		std::cerr << "[Error] Input file not specified" << std::endl;
		return 1;
	}

	double d = 0.05;

	if (!vmap.count("size")) {
		std::cerr << "[Info] Using default size 0.05m" << std::endl;
	}

	if (vmap.count("size")) {
		d = vmap["size"].as<double>();
	}

	boost::optional<size_t> threads, chunk;

	if (vmap.count("threads")) {
		threads = vmap["threads"].as<size_t>();
	}

	if (vmap.count("chunk")) {
		chunk = vmap["chunk"].as<size_t>();
	}

	const std::string input_filename = vmap["input-file"].as<std::string>();
	const bool with_mesh = vmap.count("mesh") != 0;

	std::ifstream ifs(input_filename.c_str(), std::ios::binary);
	ifs.seekg(0, ifs.end);
	size_t size = ifs.tellg();
	ifs.seekg(0, ifs.beg);
	ifs >> std::noskipws;

	boost::spirit::istream_iterator first(ifs), last;
	auto f = first;

	voxelfile_parser<decltype(first)> parser;
	std::vector<statement_type> tree;
	phrase_parse(first, last, parser, blank, tree);

	if (std::distance(f, first) != size) {
		std::cerr << "Parse errors parsing voxelfile at " << std::distance(f, first) << std::endl;
		return 1;
	}

	try {
		run(tree, d, threads.get_value_or(1), chunk.get_value_or(128), with_mesh);
		return 0;
	} catch (const std::runtime_error& e) {
		std::cerr << "Errors while running voxelfile:" << std::endl;
		std::cerr << e.what() << std::endl;
		return 1;
	}
}