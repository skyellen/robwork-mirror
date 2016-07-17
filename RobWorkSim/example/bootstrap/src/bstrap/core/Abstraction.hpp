

#ifndef ABSTRACTION_HPP_
#define ABSTRACTION_HPP_

#include <rw/common/Ptr.hpp>

class BrainState;
class Memory;

/**
 * @brief something that computes abstract knowledge and puts this into the state
 */
class Abstraction {
public:
    typedef rw::common::Ptr<Abstraction> Ptr;

    /**
     * @brief updates currentstate based on mem and whatever abstract knowledge this
     * class may derive from currentState and/or mem
     * @param currentState [in/out]
     * @param mem [in]
     */
    virtual void update(BrainState& currentState, Memory& mem) = 0;

};

#endif
