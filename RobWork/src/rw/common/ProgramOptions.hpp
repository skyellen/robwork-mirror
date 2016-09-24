#ifndef RW_COMMON_PROGRAMOPTIONS_HPP_
#define RW_COMMON_PROGRAMOPTIONS_HPP_

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/positional_options.hpp>

#include "PropertyMap.hpp"

namespace rw {
namespace common {

	/**
	 * @brief a class for parsing program command line into a PropertyMap
	 */
	class ProgramOptions {
	public:
		/**
		 * @brief Construct new set of program options.
		 * @param applicationName [in] the name of the application.
		 * @param version [in] the version of the application.
		 */
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

		/**
		 * @brief Set \b name of option number \b i.
		 * @param name [in] the name.
		 * @param i [in] index of the option.
		 */
		void setPositionalOption(const std::string& name, int i);

		/**
		 * @brief parses input, if
		 * @param argc
		 * @param argv
		 * @return if 0 is returned then help or an error
		 */
		int parse(int argc, char** argv);

		/**
		 * @brief Parses input from a string.
		 * @param string [in] input line.
		 * @return 0 if success.
		 */
		int parse(const std::string& string);

		/**
		 * @brief Get the underlying program options description from boost.
		 * @return reference to options_description.
		 */
		boost::program_options::options_description& getOptionDescription(){ return _optionDesc; }

		/**
		 * @brief Get the underlying positional program options description from boost.
		 * @return reference to positional_options_description.
		 */
		boost::program_options::positional_options_description& getPosOptionDescription(){ return _posOptionDesc; }

		/**
		 * @brief Get parsed properties in RobWork format in the form of a PropertyMap.
		 * @return the property map with parsed options.
		 */
		rw::common::PropertyMap getPropertyMap(){ return _pmap;}

	private:
		int checkVariablesMap(boost::program_options::variables_map &vm);
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
