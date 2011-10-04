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

		/**
		 * @brief this initialize default options that can add simple properties to the propertymap.
		 */
		void initOptions();

		/**
		 * @brief add a string option that is only allowed to occur once on the command line
		 * @param name [in] name of option
		 * @param defval [in] the default string value if any
		 * @param desc [in] description of commandline option
		 */
		void addStringOption(const std::string& name, const std::string& defval, const std::string& desc);
		void setPositionalOption(const std::string& name, int i);

		void parse(int argc, char** argv);
		void parse(const std::string& string);

		boost::program_options::options_description& getOptionDescription(){ return _optionDesc; }
		boost::program_options::positional_options_description& getPosOptionDescription(){ return _posOptionDesc; }

		rw::common::PropertyMap getPropertyMap(){ return _pmap;};
	private:
		void checkVariablesMap(boost::program_options::variables_map &vm);
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
