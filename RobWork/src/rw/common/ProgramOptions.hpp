#ifndef RW_COMMON_PROGRAMOPTIONS_HPP_
#define RW_COMMON_PROGRAMOPTIONS_HPP_

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>

#include <rw/math/Q.hpp>
#include <boost/foreach.hpp>

#include "PropertyMap.hpp"

namespace rw {
namespace common {

	/**
	 * @brief a class for parsing program comand line into a PropertyMap
	 */
	class ProgramOptions {
	public:
		ProgramOptions(const std::string& applicationName, const std::string& version):
			_appName(applicationName),
			_version(version),
			_optionDesc("Options"){}

		void initOptions();

		void addStringOption(const std::string& name, const std::string& defval, const std::string& desc);
		void setPositionalOption(const std::string& name, int i);

		void parse(int argc, char** argv);

		boost::program_options::options_description& getOptionDescription(){ return _optionDesc; }
		boost::program_options::positional_options_description& getPosOptionDescription(){ return _posOptionDesc; }

	private:
		std::string _appName;
		rw::common::PropertyMap _pmap;
		std::string _inputFile,_version;
		std::vector<std::string> _additionalStringOptions;
		boost::program_options::options_description _optionDesc;
		boost::program_options::positional_options_description _posOptionDesc;

	};
}
}

#endif
