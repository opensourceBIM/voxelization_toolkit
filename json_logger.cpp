#include "json_logger.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/version.hpp>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <chrono>
#include <array>

namespace {
	struct ptree_writer {
		boost::property_tree::ptree& pt;
		const std::string& k;
		void operator()(long v) {
			pt.put(k, v);
		}
		void operator()(double v) {
			pt.put(k, v);
		}
		void operator()(const std::string& v) {
			pt.put(k, v);
		}
	};

	void print_tree(std::ostream& os, const boost::property_tree::ptree::value_type& pt, int indent = 0) {
		os << std::string(indent, ' ');
		if (pt.first.size()) {
			os << pt.first << ": ";
		}
		os << pt.second.data() << std::endl;
		for (auto& c : pt.second) {
			print_tree(os, c, indent + 2);
		}
	}

	void print_tree(std::ostream& os, const boost::property_tree::ptree& pt, int indent = 0) {
		print_tree(os, { "", pt });
	}
}

std::string json_logger::to_json_string(const meta_data& m) {
	boost::property_tree::ptree pt;
	for (auto& p : m) {
		ptree_writer vis{ pt, p.first };
		boost::apply_visitor(vis, p.second);
	}
	std::ostringstream oss;
	boost::property_tree::write_json(oss, pt, false);
	return oss.str();
}

void json_logger::message(severity s, const std::string & message, const std::map<std::string, meta_data>& meta)
{
	static const std::array<const std::string, 6> severity_strings = { "", "debug", "notice", "warning", "error", "fatal" };
	auto& s_str = severity_strings[static_cast<int>(s)];

	boost::property_tree::ptree pt;
	pt.put("severity", s_str);
	pt.put("message", message);

	auto now = std::chrono::system_clock::now();
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
	auto t = std::chrono::system_clock::to_time_t(now);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%Y:%m:%d %H:%M:%S");
	oss << '.' << std::setfill('0') << std::setw(3) << ms.count();

	pt.put("time", oss.str());

	for (auto& p : meta) {
		boost::property_tree::ptree pt2;
		for (auto& p2 : p.second) {
			ptree_writer vis{ pt2, p2.first };
			boost::apply_visitor(vis, p2.second);
		}
		pt.put_child(p.first, pt2);
	}

	for (auto& p : log_output) {
		if (p.first == FMT_JSON) {
			boost::property_tree::write_json(*p.second, pt, false);
		}
		else if (p.first == FMT_TEXT) {
			print_tree(*p.second, pt);
		}		
	}
}

json_logger::severity json_logger::severity_;
json_logger::severity json_logger::max_severity_ = json_logger::LOG_NONE;
std::vector<std::pair<json_logger::format, std::ostream*>> json_logger::log_output;