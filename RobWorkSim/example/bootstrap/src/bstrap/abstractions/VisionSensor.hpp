
#ifndef VISIONSENSOR_HPP_
#define VISIONSENSOR_HPP_

#include <bstrap/core/Abstraction.hpp>

namespace rw { namespace kinematics { class Frame; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }

/**
 * @brief a vision sensor that extracts abstract knowledge in the form of
 * visible objects.
 */
class VisionSensor: public Abstraction {
public:
	VisionSensor(rw::kinematics::Frame *visionFrame,
			//rwsim::simulator::ThreadSimulator::Ptr sim,
			//rwsim::simulator::DynamicSimulator::Ptr dsim,
			rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc
	);

	void update(BrainState& currentState, Memory& mem);

	//rwsim::simulator::DynamicSimulator::Ptr _dsim;
	//rwsim::simulator::ThreadSimulator::Ptr _sim;
	rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
    rw::kinematics::Frame *_visionFrame;
};

#endif /* VISIONSENSOR_HPP_ */
