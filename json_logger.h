#ifndef JSON_LOGGER_H
#define JSON_LOGGER_H

#include <map>
#include <set>
#include <string>
#include <vector>

#include <boost/variant.hpp>

class json_logger {
public:
	typedef enum { LOG_NONE, LOG_DEBUG, LOG_NOTICE, LOG_WARNING, LOG_ERROR, LOG_FATAL } severity;
	typedef enum { FMT_TEXT, FMT_JSON } format;
	typedef boost::variant<long, double, std::string> meta_value;
	typedef std::map<std::string, meta_value> meta_data;
private:
	static std::vector<std::pair<format, std::ostream*>> log_output;
	static severity severity_;
	static severity max_severity_;
public:
	static void register_output(format f, std::ostream* os) { log_output.push_back({ f, os }); }
	static void set_severity(severity s) { severity_ = s; }
	static severity get_severity() { return severity_; }
	static severity get_max_severity() { return max_severity_; }
	static void message(severity s, const std::string& m, const std::map<std::string, meta_data>& meta);
	static void message(severity s, const std::string& m) { message(s, m, {}); };
};

#endif