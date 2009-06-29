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


private:
    int _actionType;
};

/**
 * @brief Definition of a rw::common::Ptr to an Action.
 */
typedef rw::common::Ptr<Action> ActionPtr;

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
