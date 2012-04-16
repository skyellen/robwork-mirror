#ifndef RWSIM_CONTROL_VELRAMPCONTROLLER_HPP_
#define RWSIM_CONTROL_VELRAMPCONTROLLER_HPP_

//! @file VelRampController.hpp

#include <rwlibs/control/JointController.hpp>
#include <rwlibs/control/SyncVelocityRamp.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

#include <rwsim/dynamics/KinematicDevice.hpp>

namespace rwsim {
namespace control {
	//! @addtogroup control
	//! @{

	/**
	 * @brief a JointController that use a velocityramp profile of a
	 * device to set acceleration, velocity and position of a dynamic device.
	 *
	 */
	class VelRampController: public rwlibs::control::JointController, public rwlibs::simulation::SimulatedController {

	public:

		VelRampController(const std::string& name, dynamics::KinematicDevice* kdev, const rw::kinematics::State& state):
			JointController(name, &kdev->getModel()),
			_ddev(kdev),
			_time(0.0),
			_velramp(&(kdev->getModel())),
			_target(kdev->getModel().getQ(state)),
			_currentQ(_target)
		{
			_velramp.setTarget(_target,_target);

		}

		virtual ~VelRampController(){};

		unsigned int getControlModes(){
			return POSITION;
		}

		void setControlMode(ControlMode mode){
			if(mode!=POSITION)
				RW_THROW("Unsupported control mode!");
		}

		void setTargetPos(const rw::math::Q& target);

		void setTargetVel(const rw::math::Q& vals){};
		void setTargetAcc(const rw::math::Q& vals){};

		//! @copydoc rwlibs::simulation::SimulatedController::update
		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

		//! @copydoc rwlibs::simulation::SimulatedController::reset
		void reset(const rw::kinematics::State& state);

		rw::math::Q getQ();

		rw::math::Q getQd(){ return _target;}

		std::string getControllerName(){ return getName(); };

		Controller* getController(){ return this;};

        void setEnabled(bool enabled){ _enabled = enabled; };

        bool isEnabled(){ return _enabled; } ;
	private:
		dynamics::KinematicDevice *_ddev;
		double _time;
		rw::control::SyncVelocityRamp _velramp;
		rw::math::Q _target;
		rw::math::Q _currentQ;
		bool _enabled;

	};

	typedef rw::common::Ptr<VelRampController> VelRampControllerPtr;

	//! @}
}
}

#endif /*VELRAMPCONTROLLER_HPP_*/
