/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RW_LOADERS_XMLPROXIMITYSETUPLOADER_HPP
#define RW_LOADERS_XMLPROXIMITYSETUPLOADER_HPP


#include <rw/proximity/ProximitySetup.hpp>

#include <xercesc/util/XercesDefs.hpp>

#include <string>

XERCES_CPP_NAMESPACE_BEGIN
class DOMElement;
XERCES_CPP_NAMESPACE_END

namespace rw {
namespace loaders {
/** @addtogroup loaders */
/*@{*/

/**
 * @brief Format specification for the XML PropertySetup format 
 */
class XMLProximitySetupFormat {
public:
	/**
	 * @brief Identifier for rw::proximity::ProximitySetup in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idProximitySetup();

	/**
	 * @brief Identifier for exclude rule in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idExcludeRule();

	/**
	 * @brief Identifier for include rule in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idIncludeRule();

	/**
	 * @brief Identifier for the attribute specifying whether to include all frame pairs.
	 * @return the identifier.
	 */
	static const XMLCh* idIncludeAllAttribute();

	/**
	 * @brief Identifier for the attribute specifying whether to exclude frame pairs with static transformation.
	 * @return the identifier.
	 */
	static const XMLCh* idExcludeStaticAttributePairs();

	/**
	 * @brief Identifier for the attributed specifying the first pattern in rule.
	 * @return the identifier.
	 */
	static const XMLCh* idPatternAAttribute();

	/**
	 * @brief Identifier for the attributed specifying the second pattern in rule.
	 * @return the identifier.
	 */
	static const XMLCh* idPatternBAttribute();

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLProximitySetupFormat is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLProximitySetupFormat is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;

    XMLProximitySetupFormat() {};
};


/**
 * @brief Loader for the XML PropertySetup format
 * 
 * The loader is based on Xerces
 */
class XMLProximitySetupLoader
{
public:
	/**
	 * @brief Loads ProximitySetup from \b filename
	 *
	 * Throws rw::common::Exception on errors
	 *
	 * @param filename [in] Name of input file
	 * @param schemaFileName [in] Optional name of schema file to be used for verification
	 * @return The ProximitySetup
	 */
	static rw::proximity::ProximitySetup load(const std::string& filename, const std::string& schemaFileName = "");

	/**
	 * @brief Loads ProximitySetup from \b instream
	 *
	 * Throws rw::common::Exception on errors
	 *
	 * @param instream [in] Stream containing XML ProximitySetup
	 * @param schemaFileName [in] Optional name of schema file to be used for verification
	 * @return The ProximitySetup
	 */
	static rw::proximity::ProximitySetup load(std::istream& instream, const std::string& schemaFileName = "");

	/**
	 * @brief Reads ProximitySetup from \b element
	 *
	 * Throws rw::common::Exception on errors
	 *
	 * @param element [in] Element containing ProximitySetup
	 * @return The ProximitySetup
	 */
	static rw::proximity::ProximitySetup readProximitySetup(xercesc::DOMElement* element);

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLProximitySetupLoader is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLProximitySetupLoader is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;

	static std::pair<std::string, std::string> readFramePatternAttributes(xercesc::DOMElement* element);
};

/** @} */


} //end namespace loaders
} //end namespace rw

#endif //enc include guard
