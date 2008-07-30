#ifndef TIMEDSTATEPATH_HPP_
#define TIMEDSTATEPATH_HPP_

#include "Timed.hpp"
#include <vector>
#include <rw/kinematics/State.hpp>

namespace rw { namespace interpolator {
    typedef std::vector<Timed<rw::kinematics::State> > TimedStatePath;
}} // end namespaces

#endif /*TIMEDSTATEPATH_HPP_*/
