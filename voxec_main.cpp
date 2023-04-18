#include "voxec.h"
#include "json_logger.h"

#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <fstream>
#include <iostream>

namespace po = boost::program_options;

int main(int argc, char** argv) {
	std::unique_ptr<std::ofstream> log_file;
	
	po::options_description opts("Command line options");
	opts.add_options()
		("quiet,q", "limit output on stdout to progress indication")
		("log-file", po::value<std::string>(), "filename to log json message")
		("threads,t", po::value<size_t>(), "number of parallel processing threads")
		("size,d", po::value<double>(), "voxel size in meters")
		("padding,p", po::value<size_t>(), "padding around geometry bounding box (default=32)")
		("chunk,c", po::value<size_t>(), "chunk size in number of voxels")
		("mmap,m", "use memory-mapped files instead of pure RAM")
		("mesh", "emit obj mesh for the last instruction")
		("with-pointcloud", "emit point cloud for every voxel grid variable")
		("no-vox", "do not write binary dumps")
		("help,h", "emit usage information and exit")
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
		json_logger::message(json_logger::LOG_FATAL, "invalid command line {error}", {
			{"error", {{"message", std::string(e.what())}}}
		});
		return 1;
	}

	if (vmap.count("help")) {
		std::cout << "voxec: Voxelization Toolkit Execution Runtime" << std::endl;
		std::cout << opts << std::endl << std::endl;
		std::cout << "Available commands:" << std::endl << std::endl;
		auto& m = voxel_operation_map::map();
		for (auto& p : m) {
			voxel_operation* op = voxel_operation_map::create(p.first);
			std::cout << "### " << p.first << "()" << std::endl << std::endl;
			std::cout << "name|required|type" << std::endl;
			std::cout << "---|---|---" << std::endl;
			for (auto& spec : op->arg_names()) {
				std::cout << spec.name << "|" << (spec.required ? "Y" : "n") << "|" << boost::replace_all_copy(spec.type, "|", ",") << std::endl;
			}
			std::cout << std::endl << std::endl;
		}
		return 0;
	}

	if (!vmap.count("input-file")) {
		json_logger::message(json_logger::LOG_FATAL, "input file not specified");
		return 1;
	}

	double d = 0.05;

	if (!vmap.count("size")) {
		json_logger::message(json_logger::LOG_NOTICE, "using default size 0.05m");
	} else {
		d = vmap["size"].as<double>();
	}

	if (vmap.count("padding")) {
		set_padding(vmap["padding"].as<size_t>());
	}

	boost::optional<size_t> threads, chunk;

	if (vmap.count("threads")) {
		threads = vmap["threads"].as<size_t>();
	}

	if (vmap.count("chunk")) {
		chunk = vmap["chunk"].as<size_t>();
	}

	if (vmap.count("log-file")) {
		const std::string log_filename = vmap["log-file"].as<std::string>();
		log_file = std::make_unique<std::ofstream>(log_filename.c_str());
		json_logger::register_output(json_logger::FMT_JSON, &*log_file);
	}

	const std::string input_filename = vmap["input-file"].as<std::string>();
	const bool with_mesh = vmap.count("mesh") != 0;
	const bool quiet = vmap.count("quiet") != 0;
	const bool no_vox = vmap.count("no-vox") != 0;
	const bool with_pointcloud = vmap.count("with-pointcloud") != 0;

	if (!quiet) {
		json_logger::register_output(json_logger::FMT_TEXT, &std::cerr);
	}

	std::ifstream ifs(input_filename.c_str(), std::ios::binary);
	if (!ifs.good()) {
		json_logger::message(json_logger::LOG_FATAL, "unable to open file {file}", {
			{"file", {{"text", input_filename}}}
		});
		return 1;
	}
	ifs.seekg(0, ifs.end);
	size_t size = ifs.tellg();
	ifs.seekg(0, ifs.beg);
	ifs >> std::noskipws;

	boost::spirit::istream_iterator first(ifs), last;
	auto f = first;

	voxelfile_parser<decltype(first)> parser;
	std::vector<statement_or_function_def> tree;
	phrase_parse(first, last, parser, blank, tree);

	if (std::distance(f, first) != size) {
		json_logger::message(json_logger::LOG_FATAL, "parse errors in voxelfile at {offset}", {
			{"offset", {{"value", (long) std::distance(f, first)}}}
		});
		return 1;
	}

	boost::optional<std::string> error;

	try {
		run(tree, d, threads.get_value_or(1), chunk.get_value_or(128), with_mesh, quiet, no_vox, with_pointcloud);
	} catch (const std::runtime_error& e) {
		error = std::string(e.what());
	} catch (const Standard_Failure& e) {
		if (e.GetMessageString()) {
			error = std::string(e.GetMessageString());
		} else {
			error.emplace();
		}
	} catch (...) {
		error.emplace();
	}

	if (error) {
		json_logger::message(json_logger::LOG_FATAL, "encountered {error} while running voxelfile", {
				{"error", {{"message", *error}}}
		});
	}
	
	return error ? 1 : 0;
}