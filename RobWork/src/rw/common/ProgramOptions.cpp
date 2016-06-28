#include "ProgramOptions.hpp"

#include <rw/math/Q.hpp>

//#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>

#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/phoenix1.hpp>
#include <boost/foreach.hpp>

using namespace phoenix;
using namespace boost::program_options;
using namespace boost::spirit::classic;
using namespace rw::common;

namespace po = boost::program_options;

namespace {

	template<class TYPE>
	struct Option {
	public:
		Option(std::string n, TYPE val): name(n),value(val){}
		std::string name;
		TYPE value;
	};

	/**
	 * for each of the possible option types a validator is needed.
	 */
	typedef Option<std::string> StringOption;
	typedef Option<int> IntOption;
	typedef Option<double> DoubleOption;
	typedef Option<rw::math::Q> QOption;
	typedef std::vector< Option<std::string> > StringOptionList;
	typedef std::vector< Option<int> > IntOptionList;
	typedef std::vector< Option<double> > DoubleOptionList;
	typedef std::vector< Option<rw::math::Q> > QOptionList;

	//////////////// VALIDATORS
	/**
	 * @brief validator for properties of type string
	 */
	void validate(boost::any& v,
			const std::vector<std::string>& values,
			StringOptionList* target_type,
			int);

	/**
	 * @brief validator for properties of type int
	 */
	void validate(boost::any& v,
			const std::vector<std::string>& values,
			IntOptionList* target_type,
			int);

	/**
	 * @brief validator for properties of type double
	 */
	void validate(boost::any& v,
			const std::vector<std::string>& values,
			DoubleOptionList* target_type,
			int);

	/**
	 * @brief validator for properties of type Q
	 */
	void validate(boost::any& v,
			const std::vector<std::string>& values,
			QOptionList* target_type,
			int);

	/**
	 * @brief handles the validation error/success
	 */
	template<class TYPE>
	void handleResult(bool success, boost::any& v, const std::string& name, const TYPE& value){
		if ( success ) {
			try {
				std::vector<Option<TYPE> >& values =
					boost::any_cast< std::vector<Option<TYPE> >& >(v);
				values.push_back(Option<TYPE>(name,value));
			} catch (...){
				std::vector<Option<TYPE> > values;
				values.push_back(Option<TYPE>(name,value));
				v = boost::any( values );
			}
		} else {
			//throw boost::program_options::validation_error(
			//		boost::program_options::validation_error::invalid_option_value,
			//		"invalid property value, use syntax: <name>=<value>");
			RW_THROW("invalid property value, use syntax: <name>=<value>");
		}
	}


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

	void validate(boost::any& v, const std::vector<std::string>& values, QOptionList* target_type, int){
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

}

void ProgramOptions::initOptions(){
	_optionDesc.add_options()
        ("help", "produce help message")
        ("version,v", "print version string")
		//("ini-file", po::value< std::string >()->default_value("RobWorkStudio.ini"), "RobWorkStudio ini-file")
        ("intproperty,i", po::value< IntOptionList >()->composing(),"Add a int property, -iname=2")
        ("doubleproperty,d", po::value< DoubleOptionList >()->composing(),"Add a double property, -dname=2.3")
        ("qproperty,q", po::value< QOptionList >()->composing(),"Add a Q property, eg. -qname=\"(1.0,2,32.1,2)\"\nRemember the quotes!!!")
        ("property,P", po::value< StringOptionList >()->composing(),"Add a string property, -Pname=pstring")
        //("input-file", po::value< std::string >(), "Project/Workcell/Device input file")
    ;
}

void ProgramOptions::addStringOption(const std::string& name, const std::string& defval, const std::string& desc){
	if(defval!=""){
		_optionDesc.add_options()
				(name.c_str(), po::value< std::string >()->default_value(defval.c_str()), desc.c_str());
	} else {
		_optionDesc.add_options()
				(name.c_str(), po::value< std::string >(), desc.c_str());
	}
	_additionalStringOptions.push_back(name);
}

void ProgramOptions::setPositionalOption(const std::string& name, int i){
	_posOptionDesc.add(name.c_str(), i);
}

int ProgramOptions::checkVariablesMap(po::variables_map &vm){
    if (vm.count("help")) {
        std::cout << "Usage:\n\n"
                  << "\t" << _appName <<" [options] <project-file> \n"
                  << "\t" << _appName <<" [options] <workcell-file> \n"
                  << "\t" << _appName <<" [options] <device-file> \n"
                  << "\n";
        rw::common::Log::infoLog() << _optionDesc << "\n";
        return -1;
    }

    if (vm.count("version") ){
        Log::infoLog() << "\n\t" << _appName <<" version " << _version << std::endl;
        return -1;
    }

    if( vm.count("property") ){
        StringOptionList vals = vm["property"].as< StringOptionList >();
        BOOST_FOREACH(StringOption& prop, vals){
            _pmap.add(prop.name,"",prop.value);
        }
    }
    if( vm.count("intproperty") ){
        IntOptionList vals = vm["intproperty"].as< IntOptionList >();
        BOOST_FOREACH(IntOption& prop, vals){
            _pmap.add(prop.name,"",prop.value);
        }
    }
    if( vm.count("doubleproperty") ){
        DoubleOptionList vals = vm["doubleproperty"].as< DoubleOptionList >();
        BOOST_FOREACH(DoubleOption& prop, vals){
            _pmap.add(prop.name,"",prop.value);
        }
    }
    if( vm.count("qproperty") ){
        QOptionList vals = vm["qproperty"].as< QOptionList >();
        BOOST_FOREACH(QOption& prop, vals){
            _pmap.add(prop.name,"",prop.value);
        }
    }

    BOOST_FOREACH(std::string strOption, _additionalStringOptions){
        //std::cout << strOption <<" sfdkjskf "<< std::endl;
        if( vm.count(strOption.c_str()) ){
            std::string val = vm[strOption.c_str()].as< std::string >();
            _pmap.add(strOption,"",val);
        }
    }
    //if( vm.count("input-file") ){
        //std::cout << "input-file: " << vm["input-file"].as<std::string>() << std::endl;
    //    inputfile = vm["input-file"].as<std::string>();
    //}

/*
    if( vm.count("ini-file") ){
        inifile = vm["ini-file"].as<std::string>();
    }
    */
    return 0;
}


int ProgramOptions::parse(const std::string& str){
    try {
        po::variables_map vm;

#if(BOOST_VERSION<104100)
        using namespace boost::algorithm;
        //std::vector<std::string> args = po::split_unix(str);
        std::vector<std::string> args; // #2: Search for tokens
        split( args, str, is_any_of(" \t") );
#else
        std::vector<std::string> args = po::split_unix(str);
#endif
        po::store(po::command_line_parser(args).allow_unregistered().
                  options(_optionDesc).positional(_posOptionDesc).run(), vm);
        po::notify(vm);
        return checkVariablesMap(vm);
    } catch (std::exception &e){
        rw::common::Log::infoLog() << "Command line input error:\n\t " << e.what() << "\n";
        rw::common::Log::infoLog() << "Specify --help for usage. \n";

    }
    return -1;
}

int ProgramOptions::parse(int argc, char** argv){
	try {
        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).allow_unregistered().
                  options(_optionDesc).positional(_posOptionDesc).run(), vm);
		po::notify(vm);
		return checkVariablesMap(vm);
    } catch (std::exception &e){
    	rw::common::Log::infoLog() << "Command line input error:\n\t " << e.what() << "\n";
    	rw::common::Log::infoLog() << "Specify --help for usage. \n";
    }
    return -1;
}


