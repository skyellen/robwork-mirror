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

#ifndef RW_LOADERS_XMLPROPERTYFORMAT_HPP
#define RW_LOADERS_XMLPROPERTYFORMAT_HPP

#include <xercesc/util/XercesDefs.hpp>

namespace rw {
namespace loaders {
class XercesInitializer;

/** @addtogroup loaders */
/*@{*/

/**
 * @brief Class storing the identifiers used for properties in the XML Path Format
 */
class XMLPropertyFormat
{
private:
    static const XercesInitializer initializer;
public:
    /** @brief Identifier for rw::common::PropertyMap in the XML format  */
    static const XMLCh* PropertyMapId;

    /** @brief Identifier for rw::common::Property in the XML format  */
    static const XMLCh* PropertyId;

    /** @brief Identifier for the name of a rw::common::Property */
    static const XMLCh* PropertyNameId;

    /** @brief Identifier for the description of a rw::common::Property */
    static const XMLCh* PropertyDescriptionId;

    /** @brief Identifier for the type of a rw::common::Property */
    static const XMLCh* PropertyTypeId;

    /** @brief Identifier for the value of a rw::common::Property */
    static const XMLCh* PropertyValueId;


};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //end include guard
