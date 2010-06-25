#include "ProgramOptions.hpp"
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>


#include <boost/spirit/core.hpp>
#include <boost/spirit/phoenix.hpp>

#include <boost/foreach.hpp>

using namespace phoenix;
using namespace boost::program_options;
using namespace boost::spirit;
//using namespace rw::math;

namespace po = boost::program_options;


void validate(boost::any& v, const std::vector<std::string>& values,
		StringOptionList* target_type, int){
    // Extract the first string from 'values'. If there is more than
    // one string, it's an error, and exception will be thrown.
    const std::string& s = validators::get_single_string(values);

    // parse the values
    std::string name, value;
    bool success = parse(s.c_str(),
                         (
                           (*(anychar_p-ch_p('=')))[var(name) = construct_<std::string>(arg1,arg2)]
                           >> ch_p('=')
                           >> (*(anychar_p-ch_p('=')))[var(value) = construct_<std::string>(arg1,arg2)]
                           //      anychar_p
                         )
                         , space_p).full; // Not a full statement yet, patience...

    handleResult<std::string>(success, v, name, value);
}

void validate(boost::any& v, const std::vector<std::string>& values,
		IntOptionList* target_type, int){
    // Extract the first string from 'values'. If there is more than
    // one string, it's an error, and exception will be thrown.
    const std::string& s = validators::get_single_string(values);

    // parse the values
    std::string name;
    int value;
    bool success = parse(s.c_str(),
                         (
                           (*(anychar_p-ch_p('=')))[var(name) = construct_<std::string>(arg1,arg2)]
                           >> ch_p('=')
                           >> int_p[assign_a(value)]
                         )
                         , space_p).full; // Not a full statement yet, patience...

    handleResult<int>(success, v, name, value);
}

void validate(boost::any& v, const std::vector<std::string>& values,
		DoubleOptionList* target_type, int){
    // Extract the first string from 'values'. If there is more than
    // one string, it's an error, and exception will be thrown.
    const std::string& s = validators::get_single_string(values);

    // parse the values
    std::string name;
    double value;
    bool success = parse(s.c_str(),
                         (
                           (*(anychar_p-ch_p('=')))[var(name) = construct_<std::string>(arg1,arg2)]
                           >> ch_p('=')
                           >> real_p[assign_a(value)]
                         )
                         , space_p).full; // Not a full statement yet, patience...

    handleResult<double>(success, v, name, value);
}

void validate(boost::any& v, const std::vector<std::string>& values,
		QOptionList* target_type, int){
    // Extract the first string from 'values'. If there is more than
    // one string, it's an error, and exception will be thrown.
    const std::string& s = validators::get_single_string(values);

    // parse the values
    std::string name;
    std::vector<double> value;
    bool success = parse(s.c_str(),
                         (
                           (*(anychar_p-ch_p('=')))[var(name) = construct_<std::string>(arg1,arg2)]
                           >> ch_p('=')
                           >> ch_p('(')
                           >> *(real_p[push_back_a(value)] >> ch_p(',')) >> real_p[push_back_a(value)]
                           >> ch_p(')')
                         )
                         , space_p).full; // Not a full statement yet, patience...
    rw::math::Q q(value.size());
    for(size_t i=0;i<value.size();i++)
        q[i] = value[i];


    handleResult<rw::math::Q>(success, v, name, q);
}

