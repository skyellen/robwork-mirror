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

#ifndef RW_LOADERS_DOMPROPERTYMAPFORMAT_HPP
#define RW_LOADERS_DOMPROPERTYMAPFORMAT_HPP

#include <string>

namespace rw {
namespace loaders {

/** @addtogroup loaders */
/*@{*/

/**
 * @brief Class storing the identifiers used for properties
 */
class DOMPropertyMapFormat
{
public:
    /** @brief Identifier for rw::common::PropertyMap */
    static const std::string PropertyMapId;
    /** @brief Identifier for rw::common::Property */
    static const std::string PropertyId;
    /** @brief Identifier for the name of a rw::common::Property */
    static const std::string PropertyNameId;
    /** @brief Identifier for the description of a rw::common::Property */
    static const std::string PropertyDescriptionId;
    /** @brief Identifier for the value of a rw::common::Property */
    static const std::string PropertyValueId;
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif // include guard
