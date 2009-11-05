#ifndef RW_LOADERS_XMLPROPERTYFORMAT_HPP
#define RW_LOADERS_XMLPROPERTYFORMAT_HPP


#include "XercesUtils.hpp"

namespace rw {
namespace loaders {

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
