
#ifndef CONDITION_HPP_
#define CONDITION_HPP_

#include <rw/common/Ptr.hpp>

class BrainState;

class Condition {
public:
    typedef rw::common::Ptr<Condition> Ptr;

    virtual bool isConditionMet(const BrainState& state) = 0;
};


#endif /* LUAEDITORWINDOW_HPP_ */
