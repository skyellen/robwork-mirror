/*
 * Action.hpp
 *
 *  Created on: Jan 29, 2009
 *      Author: lpe
 */

#ifndef RW_TASK_ACTION_HPP
#define RW_TASK_ACTION_HPP

#include "Entity.hpp"

namespace rw {
namespace task4 {

class ActionType {
public:
    enum { Undefined = -1, AttachFrame = 0, On = 1, Off= 2, User = 1024};

    ActionType(int type = Undefined):
        _type(type)
    {
    }

    operator int() {
        return _type;
    }

private:
    int _type;
};

class Action: public Entity
{
public:
    Action(int actionType):
        Entity(EntityType::Action),
        _actionType(actionType)
    {
    }

    virtual ~Action() {}

    ActionType getActionType() const {
        return _actionType;
    }

    /*int type() const {
        return ActionId;
    }*/

private:
    int _actionType;
};

typedef rw::common::Ptr<Action> ActionPtr;

} //end namespace task
} //end namespace rw

#endif //end include guard
