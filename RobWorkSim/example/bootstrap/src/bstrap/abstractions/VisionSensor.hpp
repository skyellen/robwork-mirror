
#ifndef VISIONSENSOR_HPP_
#define VISIONSENSOR_HPP_

#include <bstrap/core/Abstraction.hpp>
#include <rw/common/Log.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>

/**
 * @brief a vision sensor that extracts abstract knowledge in the form of
 * visible objects.
 */
class VisionSensor: public Abstraction {
public:
    VisionSensor(rw::kinematics::Frame *visionFrame,
                 rwsim::simulator::ThreadSimulator::Ptr sim,
                 rwsim::simulator::DynamicSimulator::Ptr dsim,
                 rwsim::dynamics::DynamicWorkCell::Ptr dwc);

    void update(BrainState& currentState, Memory& mem);

    rwsim::simulator::DynamicSimulator::Ptr _dsim;
    rwsim::simulator::ThreadSimulator::Ptr _sim;
    rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
    rw::kinematics::Frame *_visionFrame;
};

#endif /* VISIONSENSOR_HPP_ */
