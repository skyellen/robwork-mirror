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
using namespace rw::math;

namespace po = boost::program_options;

template<class TYPE>
struct MyProperty {
public:
    MyProperty(std::string n, TYPE val): name(n),value(val){}

    std::string name;
    TYPE value;
};

void initOptions(po::options_description& desc){
    desc.add_options()
        ("help", "produce help message")
        ("version,v", "print version string")       
		("ini-file", po::value< std::string >()->default_value("RobWorkStudio.ini"), "RobWorkStudio ini-file")
        ("intproperty,i", po::value< vector<MyProperty<int> > >()->composing(),"Add a int property, name=2")
        ("doubleproperty,d", po::value< vector<MyProperty<double> > >()->composing(),"Add a double property, name=2.3")
        ("qproperty,q", po::value< vector<MyProperty<Q> > >()->composing(),"Add a Q property, name=(1.0,2,32.1,2)")
        ("property,P", po::value< vector<MyProperty<std::string> > >()->composing(),"Add a string property, name=pstring")
        ("input-file", po::value< std::string >(), "Project/Workcell/Device input file")
    ;

}


template<class TYPE>
void handleResult(bool success, boost::any& v,const string& name, const TYPE& value){
    if ( success ) {
        try {
            std::vector<MyProperty<TYPE> >& values =
                boost::any_cast< std::vector<MyProperty<TYPE> >& >(v);
            values.push_back(MyProperty<TYPE>(name,value));
        } catch (...){
            std::vector<MyProperty<TYPE> > values;
            values.push_back(MyProperty<TYPE>(name,value));
            v = boost::any( values );
        }
    } else {
        throw validation_error("invalid property value, use syntax: <name>=<value>");
    }
}

void validate(boost::any& v, const vector<string>& values, vector< MyProperty<string> >* target_type, int){
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

    handleResult<string>(success, v, name, value);
}

void validate(boost::any& v, const vector<string>& values, vector< MyProperty<int> >* target_type, int){
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

void validate(boost::any& v, const vector<string>& values, vector< MyProperty<double> >* target_type, int){
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

void validate(boost::any& v, const vector<string>& values, vector< MyProperty<Q> >* target_type, int){
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
    Q q(value.size());
    for(size_t i=0;i<value.size();i++)
        q[i] = value[i];


    handleResult<Q>(success, v, name, q);
}

