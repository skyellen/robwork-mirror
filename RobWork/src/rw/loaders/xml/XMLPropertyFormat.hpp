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
/** @addtogroup loaders */
/*@{*/

/**
 * @brief Class storing the identifiers used for properties in the XML Path Format
 */
class XMLPropertyFormat
{
public:
    /**
     * @brief Identifier for rw::common::PropertyMap in the XML format.
     * @return the identifier.
     */
    static const XMLCh* idPropertyMap();

    /**
     * @brief Identifier for rw::common::Property in the XML format.
     * @return the identifier.
     */
    static const XMLCh* idProperty();

    /**
     * @brief Identifier for the name of a rw::common::Property.
     * @return the identifier.
     */
    static const XMLCh* idPropertyName();

    /**
     * @brief Identifier for the description of a rw::common::Property.
     * @return the identifier.
     */
    static const XMLCh* idPropertyDescription();

    /**
     * @brief Identifier for the type of a rw::common::Property.
     * @return the identifier.
     */
    static const XMLCh* idPropertyType();

    /**
     * @brief Identifier for the value of a rw::common::Property.
     * @return the identifier.
     */
    static const XMLCh* idPropertyValue();

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLPropertyFormat is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLPropertyFormat is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //end include guard
