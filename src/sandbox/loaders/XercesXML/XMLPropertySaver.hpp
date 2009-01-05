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


class XMLPropertySaver
{
public:

    static xercesc::DOMElement* save(const rw::common::PropertyBase* property, xercesc::DOMDocument* doc);

    static xercesc::DOMElement* save(const rw::common::PropertyMap& map, xercesc::DOMDocument* doc);

    static void save(const rw::common::PropertyMap& map, xercesc::DOMElement* element, xercesc::DOMDocument* doc);

    static void save(const rw::common::PropertyMap& map, const std::string& filename);

private:
    XMLPropertySaver() {};
};

} //end namespace loaders
} //end namespace rw

#endif // end include guard
