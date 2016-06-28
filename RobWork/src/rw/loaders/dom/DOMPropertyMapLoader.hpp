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

#ifndef RW_LOADERS_DOMPROPERTYMAPLOADER_HPP
#define RW_LOADERS_DOMPROPERTYMAPLOADER_HPP


#include <rw/common/PropertyMap.hpp>
#include <rw/common/PropertyBase.hpp>

namespace rw { namespace common { class DOMElem; } }

namespace rw {
namespace loaders {

/** @addtogroup loaders */
/*@{*/

/**
 * @brief Class for loading rw::common::PropertyMap from XML
 *
 * The loader is capable of loading all type defined in rw::common::PropertyType.
 *
 * Implemented using RobWork DOM parser abstraction.
 */
class DOMPropertyMapLoader
{
public:

    /**
     * @brief Reads in a Property from DOMElement.
     *
     * May throw rw::common::Exception
     *
     * @param element [in] DOMElement describing Property
     * @param checkHeader [in] True to check that the header of \b element matches XMLPropertyFormat::PropertyId
     * @return Pointer to the property
     */
	static rw::common::PropertyBase::Ptr readProperty(rw::common::Ptr<rw::common::DOMElem> element, bool checkHeader = true);


    /**
     * @brief Reads in a PropertyMap from DOMElement
     *
     * May throw rw::common::Exception
     *
     * @param element [in] DOMElement describing PropertyMap
     * @param checkHeader [in] True to check that the header of \b element matches XMLPropertyFormat::PropertyMapId
     * @return Loaded PropertyMap
     */
    static rw::common::PropertyMap readProperties(rw::common::Ptr<rw::common::DOMElem> element, bool checkHeader = true);
    static bool hasProperties(rw::common::Ptr<rw::common::DOMElem> element);

    /**
     * @brief Read in rw::common::PropertyMap from file
     *
     * Throws rw::common::Exception if an error occurs
     *
     * @param filename [in] File to load
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     * @return Loaded PropertyMap
     */
    static rw::common::PropertyMap load(const std::string& filename, const std::string& schemaFileName = "");

    /**
     * @brief Read in rw::common::PropertyMap from istream
     *
     * Throws rw::common::Exception if an error occurs
     *
     * @param instream [in] Input stream to read from
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     * @return Loaded PropertyMap
     */
    static rw::common::PropertyMap load(std::istream& instream, const std::string& schemaFileName = "");

private:
    DOMPropertyMapLoader();
    virtual ~DOMPropertyMapLoader();

};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //end include guard
