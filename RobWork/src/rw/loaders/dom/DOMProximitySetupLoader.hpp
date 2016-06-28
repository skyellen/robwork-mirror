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

#ifndef RW_LOADERS_DOMPROXIMITYSETUPLOADER_HPP
#define RW_LOADERS_DOMPROXIMITYSETUPLOADER_HPP


#include <rw/proximity/ProximitySetup.hpp>
#include <string>

namespace rw { namespace common { class DOMElem; } }

namespace rw {
namespace loaders {

/** @addtogroup loaders */
/*@{*/

/**
 * @brief Loader for the XML PropertySetup format
 * 
 * The loader is based on Xerces
 */
class DOMProximitySetupLoader
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
	static rw::proximity::ProximitySetup readProximitySetup(rw::common::Ptr<rw::common::DOMElem> element);


private:
	static std::pair<std::string, std::string> readFramePatternAttributes(rw::common::Ptr<rw::common::DOMElem> element);

};

/** @} */


} //end namespace loaders
} //end namespace rw

#endif //enc include guard
