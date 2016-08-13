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
    /**
     * @brief Get identifier for rw::common::PropertyMap.
     * @return the identifier.
     */
    static const std::string& idPropertyMap();

    /**
     * @brief Get identifier for rw::common::Property.
     * @return the identifier.
     */
    static const std::string& idProperty();

    /**
     * @brief Get identifier for the name of a rw::common::Property.
     * @return the identifier.
     */
    static const std::string& idPropertyName();

    /**
     * @brief Get identifier for the description of a rw::common::Property.
     * @return the identifier.
     */
    static const std::string& idPropertyDescription();

    /**
     * @brief Get identifier for the value of a rw::common::Property.
     * @return the identifier.
     */
    static const std::string& idPropertyValue();

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the DOMPropertyMapFormat is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if DOMPropertyMapFormat is to be used in local static
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

#endif // include guard
