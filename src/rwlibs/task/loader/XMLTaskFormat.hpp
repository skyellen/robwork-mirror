/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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
