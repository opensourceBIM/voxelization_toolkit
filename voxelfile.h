#ifndef VOXELFILE_H
#define VOXELFILE_H

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/qi_directive.hpp>
#include <boost/fusion/include/boost_tuple.hpp>
#include <boost/fusion/container/vector.hpp>

using namespace boost::spirit::qi;

using boost::spirit::int_;
using boost::spirit::qi::grammar;
using boost::spirit::qi::rule;

typedef boost::variant<double, int, std::string, std::vector<std::string> > function_arg_value_type;

class function_arg_type : public boost::fusion::vector<boost::optional<std::string>, function_arg_value_type> {
public:
	bool has_keyword() const {
		return !!boost::fusion::at_c<0>(*this);
	}
	const boost::optional<std::string>& keyword() const {
		return boost::fusion::at_c<0>(*this);
	}
	const function_arg_value_type& value() const {
		return boost::fusion::at_c<1>(*this);
	}
};

class function_call_type : public boost::fusion::vector<std::string, std::vector<function_arg_type> > {
public:
	const std::string& name() const {
		return boost::fusion::at_c<0>(*this);
	}
	const std::vector<function_arg_type>& args() const {
		return boost::fusion::at_c<1>(*this);
	}
};

class statement_type : public boost::fusion::vector<std::string, function_call_type> {
public:
	const std::string& assignee() const {
		return boost::fusion::at_c<0>(*this);
	}
	const function_call_type& call() const {
		return boost::fusion::at_c<1>(*this);
	}
};

class function_def_type : public boost::fusion::vector<std::string, std::vector<std::string>, std::vector<statement_type>, std::string> {
public:
	const std::string& name() const {
		return boost::fusion::at_c<0>(*this);
	}
	const std::vector<std::string>& args() const {
		return boost::fusion::at_c<1>(*this);
	}
	const std::vector<statement_type>& statements() const {
		return boost::fusion::at_c<2>(*this);
	}
	const std::string& result_identifier() const {
		return boost::fusion::at_c<3>(*this);
	}
};

typedef boost::variant<statement_type, function_def_type> statement_or_function_def;

std::ostream& operator<<(std::ostream& a, const function_call_type& b);

std::ostream& operator<<(std::ostream& a, const statement_type& b);

template <typename It>
struct voxelfile_parser : grammar<It, std::vector<statement_or_function_def>(), blank_type> {

	voxelfile_parser() : grammar<It, std::vector<statement_or_function_def>(), blank_type>(start) {
		iden = lexeme[(alpha >> *(alnum | char_('_')))];
		statement = -hold[iden >> '='] >> function_call >> eol >> *eol;
		quoted_string = char_('"') >> *(char_ - '"') >> char_('"');
		// @todo only sequence of strings for now
		sequence = '{' >> quoted_string % ',' >> '}';
		value = strict_double | int_ | quoted_string | iden | sequence;
		function_arg = -hold[iden >> '='] >> value;
		function_args = function_arg % ',';
		function_call = iden >> '(' >> function_args >> ')';
		argument_list = iden % ',';
		function_def = "function" >> iden >> '(' >> argument_list >> ')' >> eol >> *eol >> *statement >> "return" >> -hold[iden] >> eol >> *eol;
		start = *(statement | function_def);
	}

	real_parser<double, strict_real_policies<double>> strict_double;
	rule<It, function_arg_value_type(), blank_type> value;
	rule<It, function_arg_type(), blank_type> function_arg;
	rule<It, std::vector<std::string>(), blank_type> sequence;
	rule<It, std::vector<function_arg_type>(), blank_type> function_args;
	rule<It, function_call_type(), blank_type> function_call;
	rule<It, statement_type(), blank_type> statement;
	rule<It, std::string(), blank_type> iden, quoted_string;
	rule<It, std::vector<std::string>(), blank_type> argument_list;
	rule<It, function_def_type(), blank_type> function_def;

	typename voxelfile_parser::start_type start;
};

#endif
