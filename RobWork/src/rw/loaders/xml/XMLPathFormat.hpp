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

#ifndef RW_LOADERS_XMLPATHFORMAT_HPP
#define RW_LOADERS_XMLPATHFORMAT_HPP

#include <xercesc/util/XercesDefs.hpp>

namespace rw {
namespace loaders {
class XercesInitializer;

/** @addtogroup loaders */
/*@{*/


/**
 * @brief Class storing the identifiers used for paths in the XML Path Format
 */
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

    /** @brief Identifier for rw::trajectory::StatePath in the XML format  */
    static const XMLCh* StatePathId;

    /** @brief Identifier for rw::trajectory::TimedQPath in the XML format  */
    static const XMLCh* TimedQPathId;

    /** @brief Identifier for rw::trajectory::TimedState in the XML format  */
    static const XMLCh* TimedStateId;

    /** @brief Identifier for rw::trajectory::TimedQ in the XML format  */
    static const XMLCh* TimedQId;

    /** @brief Identifier for rw::trajectory::TimedStatePath in the XML format  */
    static const XMLCh* TimedStatePathId;

    /** @brief Identifier for time attribute used for rw::trajectory::TimedQPath and rw::trajectory::TimedStatePath in the XML format  */
    static const XMLCh* TimeId;

	static const int* myTestVar;

private:
    XMLPathFormat() {};
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //End include guard
