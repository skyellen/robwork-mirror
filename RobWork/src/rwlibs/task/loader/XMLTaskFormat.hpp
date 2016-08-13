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

#include <xercesc/util/XercesDefs.hpp>

namespace rwlibs {

namespace task {


/** @addtogroup task */
/*@{*/


/**
 * @brief Class storing the identifiers used for paths in the XML Task Format
 */
class XMLTaskFormat
{
public:
    /**
     * @brief Identifier for rw::task::Task with rw::math::Q as content in the XML format.
     * @return the identifier.
     */
    static const XMLCh* idQTask();

    /**
     * @brief Identifier for CartesianTask.
     * @return the identifier.
     */
    static const XMLCh* idCartesianTask();

    /**
     * @brief Identifier for Targets.
     * @return the identifier.
     */
    static const XMLCh* idTargets();

    /**
     * @brief Identifier for Entities.
     * @return the identifier.
     */
    static const XMLCh* idEntities();

    /**
     * @brief Identifier for Augmentations.
     * @return the identifier.
     */
    static const XMLCh* idAugmentations();

    /**
     * @brief Identifier for QTarget.
     * @return the identifier.
     */
    static const XMLCh* idQTarget();

    /**
     * @brief Identifier for CartesianTarget.
     * @return the identifier.
     */
    static const XMLCh* idCartesianTarget();

    /**
     * @brief Identifier for Motion.
     * @return the identifier.
     */
    static const XMLCh* idMotion();

    /**
     * @brief Identifier for Action.
     * @return the identifier.
     */
    static const XMLCh* idAction();

    /**
     * @brief Identifier for EntityIndex.
     * @return the identifier.
     */
    static const XMLCh* idEntityIndex();

    /**
     * @brief Identifier for EntityId.
     * @return the identifier.
     */
    static const XMLCh* idEntityId();

    /**
     * @brief Identifier for TargetIdAttr.
     * @return the identifier.
     */
    static const XMLCh* idTargetIdAttr();

    /**
     * @brief Identifier for MotionTypeAttr.
     * @return the identifier.
     */
    static const XMLCh* idMotionTypeAttr();

    /**
     * @brief Identifier for MotionStart.
     * @return the identifier.
     */
    static const XMLCh* idMotionStart();

    /**
     * @brief Identifier for MotionMid.
     * @return the identifier.
     */
    static const XMLCh* idMotionMid();

    /**
     * @brief Identifier for MotionEnd.
     * @return the identifier.
     */
    static const XMLCh* idMotionEnd();

    /**
     * @brief Identifier for LinearMotion.
     * @return the identifier.
     */
    static const XMLCh* idLinearMotion();

    /**
     * @brief Identifier for P2PMotion.
     * @return the identifier.
     */
    static const XMLCh* idP2PMotion();

    /**
     * @brief Identifier for CircularMotion.
     * @return the identifier.
     */
    static const XMLCh* idCircularMotion();

    /**
     * @brief Identifier for ActionTypeAttr.
     * @return the identifier.
     */
    static const XMLCh* idActionTypeAttr();

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLTaskFormat is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLTaskFormat is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;

	XMLTaskFormat() {};
};

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //End include guard
