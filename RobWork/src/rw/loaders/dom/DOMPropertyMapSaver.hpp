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

#ifndef RW_LOADERS_DOMPROPERTYMAPSAVER_HPP
#define RW_LOADERS_DOMPROPERTYMAPSAVER_HPP

#include <rw/common/DOMElem.hpp>

namespace rw { namespace common { class DOMParser; } }
namespace rw { namespace common { class PropertyBase; } }
namespace rw { namespace common { class PropertyMap; } }

namespace rw {
namespace loaders {

/** @addtogroup loaders */
/*@{*/

/**
 * @brief Class for saving rw::common::PropertyMap to XML
 *
 * The saver is capable of saving all types defined in rw::common::PropertyType.
 *
 * Implemented using RobWork DOM parser abstraction.
 */
class DOMPropertyMapSaver
{
public:
    /**
     * @brief Writes a single property to a DOMElement
     *
     * Constructs a new DOMElement for the document \b parent and writes the property to it.
     *
     * @throws rw::common::Exception if the type of the property is not supported.
     *
     * @param property [in] Property to save
     * @param parent [in] DOMDocument which should contain the property representation
     */
    static void save(rw::common::Ptr<rw::common::PropertyBase> property, rw::common::DOMElem::Ptr parent);

    /**
     * @brief Saves properties of a PropertyMap as childs to \b element.
     *
     * Constructs element representing the properties in \b map and adds these as childs to \b element.
     *
     * Throws rw::common::Expcetion if the type of a property is not supported.
     *
     * @param map [in] Map of properties to save.
     * @param parent [in] DOMDocument which should contain the PropertyMap representation
     */
    static void save(const rw::common::PropertyMap& map, rw::common::DOMElem::Ptr parent);

    /**
     * @brief Saves the properties of \b map to file named \b filename
     *
     * @throws rw::common::Exception if the type of a property is not supported.
     *
     * @param map [in] Map of properties to save
     * @param filename [in] Filename
     */
    static void save(const rw::common::PropertyMap& map, const std::string& filename);

    /**
     * @brief Writes the properties of \b map to \b outstream
     *
     * @throws rw::common::Exception if the type of a property is not supported.
     *
     * @param map [in] Map of properties to save
     * @param outstream [in] Output stream
     */
    static void write(const rw::common::PropertyMap& map, std::ostream& outstream);

    /**
     * @brief Creates DOMDocument for \b map
     *
     * @throws rw::common::Exception if the type of a property is not supported.
     *
     * @param map [in] Map of properties
     * @param parser [in] DOMParser to use
     * @return DOMDocument containing properties.
     */
    static rw::common::DOMElem::Ptr createDOMDocument(const rw::common::PropertyMap& map, rw::common::Ptr<rw::common::DOMParser> parser);

private:
    DOMPropertyMapSaver() {};
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif // end include guard
