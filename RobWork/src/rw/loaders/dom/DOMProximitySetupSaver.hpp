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

#ifndef RW_LOADERS_DOMPROXIMITYSETUPSAVER_HPP
#define RW_LOADERS_DOMPROXIMITYSETUPSAVER_HPP

#include <rw/common/DOMElem.hpp>

namespace rw { namespace common { class DOMParser; } }
namespace rw { namespace proximity { class ProximitySetup; } }

namespace rw {
namespace loaders {

/** @addtogroup loaders */
/*@{*/

/**
 * @brief Class for saving rw::proximity::ProximitySetup to XML
 *
 * Implemented using RobWork DOM parser abstraction.
 */
class DOMProximitySetupSaver
{
public:

    /**
     * @brief Saves proximity setup patterns of a ProximitySetup as childs to \b element.
     *
     * Constructs element representing the include or exclude patterns defined in the proximity setup.
     *
     * Throws rw::common::Exception if the type of a ProximitySetupRule is not supported.
     *
     * @param prox [in] ProximitySetup to save.
     * @param parent [in] DOMDocument which should contain the ProximitySetup representation
     */
    static void save(const rw::proximity::ProximitySetup& prox, rw::common::DOMElem::Ptr parent);

    /**
     * @brief Saves the proximity setup \b prox to a file named \b filename
     *
     * @throws rw::common::Exception if the type of a ProximitySetupRule is not supported.
     *
     * @param prox [in] ProximitySetup to save.
     * @param filename [in] Filename
     */
    static void save(const rw::proximity::ProximitySetup& prox, const std::string& filename);

    /**
     * @brief Writes the properties of \b map to \b outstream
     *
     * @throws rw::common::Exception if the type of a property is not supported.
     *
     * @param map [in] Map of properties to save
     * @param outstream [in] Output stream
     */
    static void write(const rw::proximity::ProximitySetup& prox, std::ostream& outstream);

    /**
     * @brief Creates DOMDocument for \b map
     *
     * @throws rw::common::Exception if the type of a property is not supported.
     *
     * @param map [in] Map of properties
     * @param parser [in] DOMParser to use
     * @return DOMDocument containing properties.
     */
    static rw::common::DOMElem::Ptr createDOMDocument(const rw::proximity::ProximitySetup& prox, rw::common::Ptr<rw::common::DOMParser> parser);

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the DOMPropertyMapSaver is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if DOMPropertyMapSaver is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;
    DOMProximitySetupSaver() {};
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif // end include guard
