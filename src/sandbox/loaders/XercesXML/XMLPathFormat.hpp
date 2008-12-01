/*
 * XMLPathFormat.hpp
 *
 *  Created on: Nov 27, 2008
 *      Author: lpe
 */

#ifndef RW_LOADERS_XMLPATHFORMAT_HPP
#define RW_LOADERS_XMLPATHFORMAT_HPP

#include "XercesUtils.hpp"

namespace rw {

namespace loaders {


class XMLPathFormat
{
private:
    static const XercesInitializer initializer;
public:

    /** @brief Identifier for rw::trajectory::QPath in the XML format  */
    static const XMLCh* QPathId;

    /** @brief Identifier for rw::trajectory::Vector3DPath in the XML format  */
    static const XMLCh* V3DPathId;

    /** @brief Identifier for rw::trajectory::Rotation3DPath in the XML format  */
    static const XMLCh* R3DPathId;

    /** @brief Identifier for rw::trajectory::Transform3DPath in the XML format  */
    static const XMLCh* T3DPathId;

private:
    XMLPathFormat() {};
};

} //end namespace loaders
} //end namespace rw

#endif //End include guard
