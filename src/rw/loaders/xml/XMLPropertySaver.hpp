#ifndef RW_LOADERS_XMLPROPERTYSAVER_HPP
#define RW_LOADERS_XMLPROPERTYSAVER_HPP

#include <rw/common/PropertyMap.hpp>
#include <rw/common/PropertyBase.hpp>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>

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
    static xercesc::DOMElement* save(rw::common::PropertyBasePtr property, xercesc::DOMDocument* doc);

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
     * Throws rw::common::Expcetion if the type of a property is not supported.
     *
     * @param map [in] Map of properties to save
     * @param filename [in] Filename
     */
    static void save(const rw::common::PropertyMap& map, const std::string& filename);

private:
    XMLPropertySaver() {};
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif // end include guard
