/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_TASK_DOMTASKFORMAT_HPP_
#define RWLIBS_TASK_DOMTASKFORMAT_HPP_

/**
 * @file DOMTaskFormat.hpp
 *
 * \copydoc rwlibs::task::DOMTaskFormat
 */

#include <string>

namespace rwlibs {
namespace task {
//! @addtogroup task

//! @{
/**
 * @brief Definition of the identifier used in the task format.
 */
class DOMTaskFormat {
public:
    /**
     * @brief Identifier for rw::task::Task with rw::math::Q as content in the XML format.
     * @return the identifier.
     */
    static const std::string& idQTask();

    /**
     * @brief Identifier for CartesianTask.
     * @return the identifier.
     */
    static const std::string& idCartesianTask();

    /**
     * @brief Identifier for Targets.
     * @return the identifier.
     */
    static const std::string& idTargets();

    /**
     * @brief Identifier for Entities.
     * @return the identifier.
     */
    static const std::string& idEntities();

    /**
     * @brief Identifier for Augmentations.
     * @return the identifier.
     */
    static const std::string& idAugmentations();

    /**
     * @brief Identifier for QTarget.
     * @return the identifier.
     */
    static const std::string& idQTarget();

    /**
     * @brief Identifier for CartesianTarget.
     * @return the identifier.
     */
    static const std::string& idCartesianTarget();

    /**
     * @brief Identifier for Motion.
     * @return the identifier.
     */
    static const std::string& idMotion();

    /**
     * @brief Identifier for Action.
     * @return the identifier.
     */
    static const std::string& idAction();

    /**
     * @brief Identifier for EntityIndex.
     * @return the identifier.
     */
    static const std::string& idEntityIndex();

    /**
     * @brief Identifier for EntityId.
     * @return the identifier.
     */
    static const std::string& idEntityId();

    /**
     * @brief Identifier for TargetIdAttr.
     * @return the identifier.
     */
    static const std::string& idTargetIdAttr();

    /**
     * @brief Identifier for MotionTypeAttr.
     * @return the identifier.
     */
    static const std::string& idMotionTypeAttr();

    /**
     * @brief Identifier for MotionStart.
     * @return the identifier.
     */
    static const std::string& idMotionStart();

    /**
     * @brief Identifier for MotionMid.
     * @return the identifier.
     */
    static const std::string& idMotionMid();

    /**
     * @brief Identifier for MotionEnd.
     * @return the identifier.
     */
    static const std::string& idMotionEnd();

    /**
     * @brief Identifier for LinearMotion.
     * @return the identifier.
     */
    static const std::string& idLinearMotion();

    /**
     * @brief Identifier for P2PMotion.
     * @return the identifier.
     */
    static const std::string& idP2PMotion();

    /**
     * @brief Identifier for CircularMotion.
     * @return the identifier.
     */
    static const std::string& idCircularMotion();

    /**
     * @brief Identifier for ActionTypeAttr.
     * @return the identifier.
     */
    static const std::string& idActionTypeAttr();

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the DOMTaskLoader is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if DOMTaskLoader is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;

	DOMTaskFormat() {}
	virtual ~DOMTaskFormat() {}
};
//! @}
} /* namespace task */
} /* namespace rwlibs */

#endif /* RWLIBS_TASK_DOMTASKFORMAT_HPP_ */
