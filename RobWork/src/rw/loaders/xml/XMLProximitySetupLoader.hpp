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
class XercesInitializer;

/** @addtogroup loaders */
/*@{*/

/**
 * @brief Format specification for the XML PropertySetup format 
 */
class XMLProximitySetupFormat {
	private:
		//Needed to make sure Xerces is initialized
		static const XercesInitializer initializer;
	public:

    /** @brief Identifier for rw::proximity::ProximitySetup in the XML format  */
    static const XMLCh* ProximitySetupId;

    /** @brief Identifier for exclude rule in the XML format  */
    static const XMLCh* ExcludeRuleId;

    /** @brief Identifier for include rule in the XML format  */
    static const XMLCh* IncludeRuleId;

	/** @brief Identifier for the attribute specifying whether to include all frame pairs */
	static const XMLCh* IncludeAllAttributeId;

	/** @brief Identifier for the attribute specifying whether to exclude frame pairs with static transformation*/
	static const XMLCh* ExcludeStaticAttributePairsId;

	/** @brief Identifier for the attributed specifying the first pattern in rule */
	static const XMLCh* PatternAAttributeId;

	/** @brief Identifier for the attributed specifying the second pattern in rule */
	static const XMLCh* PatternBAttributeId;

private:
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


private:
	static std::pair<std::string, std::string> readFramePatternAttributes(xercesc::DOMElement* element);

};

/** @} */


} //end namespace loaders
} //end namespace rw

#endif //enc include guard
