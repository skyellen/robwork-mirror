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

#ifndef RW_LOADERS_XMLPROPERTYSAVER_HPP
#define RW_LOADERS_XMLPROPERTYSAVER_HPP

#include <rw/common/PropertyMap.hpp>
#include <rw/common/PropertyBase.hpp>

#include <xercesc/util/XercesDefs.hpp>

XERCES_CPP_NAMESPACE_BEGIN
class DOMDocument;
class DOMElement;
XERCES_CPP_NAMESPACE_END

namespace rw {
namespace loaders {


/** @addtogroup loaders */
/*@{*/


/**
 * @brief Class for saving rw::common::PropertyMap to XML
 *
 * The class support saving all in rw::common::PropertyType defined types.
 */
class XMLPropertySaver
{
public:
    /**
     * @brief Writes a single property to a DOMElement
     *
     * Constructs a new DOMElement for the document \b doc and writes the property to it.
     *
     * Throws rw::common::Exception if the type of the property is not supported.
     *
     * @param property [in] Property to save
     * @param doc [in] DOMDocument which should contain the property representation
     * @return DOMElement representing \b property and belonging to \b doc.
     */
	static xercesc::DOMElement* save(rw::common::PropertyBase::Ptr property, xercesc::DOMDocument* doc);

    /**
     * @brief Writes PropertyMap to a DOMElement
     *
     * Constructs a new DOMElement for the document \b doc and adds the properties in \b map to it.
     *
     * Throws rw::common::Expcetion if the type of a property is not supported.
     *
     * @param map [in] Map of properties to save
     * @param doc [in] DOMDocument which should contain the properties
     * @return DOMElement representing \b map and belonging to \b doc
     */
    static xercesc::DOMElement* save(const rw::common::PropertyMap& map, xercesc::DOMDocument* doc);

    /**
     * @brief Saves properties of a PropertyMap as childs to \b element.
     *
     * Constructs element representing the properties in \b map and adds these as childs to \b element.
     *
     * Throws rw::common::Expcetion if the type of a property is not supported.
     *
     * @param map [in] Map of properties to save.
     * @param element [in] Element to which properties should be stored as children.
     * @param doc [in] DOMDocument containing element and which should contain the individual properties
     */
    static void save(const rw::common::PropertyMap& map, xercesc::DOMElement* element, xercesc::DOMDocument* doc);

    /**
     * @brief Saves the properties of \b map to file named \b filename
     *
     * Throws rw::common::Exception if the type of a property is not supported.
     *
     * @param map [in] Map of properties to save
     * @param filename [in] Filename
     */
    static void save(const rw::common::PropertyMap& map, const std::string& filename);

    /**
     * @brief Writes the properties of \b map to \b outstream
     *
     * Throws rw::common::Exception if the type of a property is not supported.
     *
     * @param map [in] Map of properties to save
     * @param outstream [in] Output stream
     */
    static void write(const rw::common::PropertyMap& map, std::ostream& outstream);

    /**
     * @brief Creates DOMDocument for \b map
     *
     * IMPORTANT: Remember to call release on the returned DOMDocument
     *
     * @param map [in] Map of properties
     * @return DOMDocument containing properties.
     */
    static xercesc::DOMDocument* createDOMDocument(const rw::common::PropertyMap& map);

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLPropertySaver is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLPropertySaver is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;

    XMLPropertySaver() {};
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif // end include guard
