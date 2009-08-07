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


#ifndef RWLIBS_TASK_XMLTASKFORMAT_HPP
#define RWLIBS_TASK_XMLTASKFORMAT_HPP

#include <rw/loaders/xml/XercesUtils.hpp>

namespace rwlibs {

namespace task {


/** @addtogroup task */
/*@{*/


/**
 * @brief Class storing the identifiers used for paths in the XML Task Format
 */
class XMLTaskFormat
{
private:
    static const rw::loaders::XercesInitializer initializer;
public:

    /** @brief Identifier for rw::task::Task with rw::math::Q as content in the XML format  */
    static const XMLCh* QTaskId;

    static const XMLCh* CartesianTaskId;

    static const XMLCh* TargetsId;

    static const XMLCh* EntitiesId;

    static const XMLCh* AugmentationsId;


    static const XMLCh* QTargetId;

    static const XMLCh* CartesianTargetId;


    static const XMLCh* MotionId;
    static const XMLCh* ActionId;




    static const XMLCh* EntityIndexId;
    static const XMLCh* EntityIdId;

    static const XMLCh* TargetIdAttrId;

    static const XMLCh* MotionTypeAttrId;



    static const XMLCh* MotionStartId;
    static const XMLCh* MotionMidId;
    static const XMLCh* MotionEndId;

    static const XMLCh* LinearMotionId;
    static const XMLCh* P2PMotionId;
    static const XMLCh* CircularMotionId;


    static const XMLCh* ActionTypeAttrId;

private:
	XMLTaskFormat() {};
};

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //End include guard
