#ifndef RW_LOADERS_XMLPROPERTYLOADER_HPP
#define RW_LOADERS_XMLPROPERTYLOADER_HPP


#include <rw/common/PropertyMap.hpp>
#include <rw/common/PropertyBase.hpp>

#include <xercesc/dom/DOMElement.hpp>

namespace rw {
namespace loaders {

/** @addtogroup loaders */
/*@{*/

/**
 * @brief Class for loading rw::common::PropertyMap from XML
 *
 * The loader is capable of loading all type defined in rw::common::PropertyType.
 *
 * Implemented using Xerces.
 */
class XMLPropertyLoader
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
	static rw::common::PropertyBase::Ptr readProperty(xercesc::DOMElement* element, bool checkHeader = true);

    /**
     * @brief Reads in a PropertyMap from DOMElement
     *
     * May throw rw::common::Exception
     *
     * @param element [in] DOMElement describing PropertyMap
     * @param checkHeader [in] True to check that the header of \b element matches XMLPropertyFormat::PropertyMapId
     * @return Loaded PropertyMap
     */
    static rw::common::PropertyMap readProperties(xercesc::DOMElement* element, bool checkHeader = true);

    /**
     * @brief Read in rw::common::PropertyMap from file
     *
     * May throw rw::common::Exception
     *
     * @param filename [in] File to load
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     * @param Loaded PropertyMap
     */
    static rw::common::PropertyMap load(const std::string& filename, const std::string& schemaFileName = "");

private:
    XMLPropertyLoader();
    virtual ~XMLPropertyLoader();

};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //end include guard
