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
namespace task2 {

class Action: public Entity
{
public:
    Action(int actionType) {
        _actionType = actionType;
    }

    virtual ~Action() {}

    enum ActionType { USER = 0, ATTACH_FRAME, ON, OFF};

    int getActionType() const {
        return _actionType;
    }

    int type() const {
        return ACTION;
    }

private:
    int _actionType;
};

typedef rw::common::Ptr<Action> ActionPtr;

} //end namespace task
} //end namespace rw

#endif //end include guard
