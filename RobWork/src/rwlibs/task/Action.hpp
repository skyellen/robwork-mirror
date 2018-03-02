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


#ifndef RWLIBS_TASK_ACTION_HPP
#define RWLIBS_TASK_ACTION_HPP

#include "Entity.hpp"

namespace rwlibs {
namespace task {

/** @addtogroup task */
/*@{*/


/**
 * @brief Specification of Action Type
 */
class ActionType {
public:
    /** Enumeration for different action types */
    enum {  Undefined = -1 /** Undefined action */
            ,AttachFrame = 0 /** Attach frame action */
            ,On = 1 /** Turn on */
            ,Off= 2 /** Turn off */
            ,User = 1024 /**User defined action types starts with this index */
            };

    /**
     * @brief Creates an ActionType
     *
     * Creates an ActionType object with a given type. If no type is specified it is defined as Undefined.
     *
     * @param type [in] The type id
     */
    ActionType(int type = Undefined):
        _type(type)
    {
    }




    /**
     * @brief Cast operator enable implicit conversion to int
     *
     * This operator enables using ActionType in a switch statement.
     */
    operator int() {
        return _type;
    }



private:
    int _type;
};


/**
 * @brief Specification of an action in a task
 *
 * An Action in a task can be used to specify e.g. when to turn a tool on/off.
 * The default action only contains a simple type and a rw::common::PropertyMap,
 * which can be used to store value associated with the event. It is the responsibility
 * of the user to interpret an Action and do what is necessary.
 */
class Action: public Entity
{
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<Action> Ptr;

    /**
     * @brief Construct an Action with a given type
     *
     * @param actionType [in] Type of the action
     */
    Action(ActionType actionType):
        Entity(EntityType::Action),
        _actionType(actionType)
    {
    }

    /**
     * @brief Destructor
     */
    virtual ~Action() {}



    /**
     * @brief Returns the type of the action.
     */
    ActionType actionType() const {
        return _actionType;
    }

    /**
     * @brief Make a copy of the action.
     * @return new identical action.
     */
    virtual rw::common::Ptr<Action> clone() {
    	return rw::common::ownedPtr(new Action(_actionType));
    }

private:
    int _actionType;
};

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
