#ifndef RWSIM_CONTROL_VELRAMPCONTROLLER_HPP_
#define RWSIM_CONTROL_VELRAMPCONTROLLER_HPP_

//! @file VelRampController.hpp

#include <rwlibs/control/JointController.hpp>
#include <rwlibs/control/SyncVelocityRamp.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

#include <rwsim/dynamics/KinematicDevice.hpp>

namespace rwsim {
namespace control {
	//! @addtogroup control @{

	/**
	 * @brief a JointController that use a velocityramp profile of a
	 * device to set acceleration, velocity and position of a dynamic device.
	 *
	 */
	class VelRampController: public JointController, public rwlibs::simulation::SimulatedController {

	public:

		VelRampController(dynamics::KinematicDevice* kdev, const rw::kinematics::State& state):
			JointController(&kdev->getModel()),
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

		void setTargetPos(const rw::math::Q& target){
			rw::math::Q q = _velramp.x(_time);
			_velramp.setTarget(q,target);
			_time = 0;
			_target = target;
		}

		void setTargetVel(const rw::math::Q& vals){};
		void setTargetAcc(const rw::math::Q& vals){};

		/**
		 * @brief updates the state of the dynamicdevice
		 */
		void update(double dt, rw::kinematics::State& state) {
			_time += dt;
			rw::math::Q qvel = _velramp.dx(_time);

			std::cout << "Setting Qvelocity: "<< qvel << std::endl;
			_ddev->setVelocity(qvel, state);
			_currentQ = qvel;
		}

		void reset(const rw::kinematics::State& state){
			_currentQ = _ddev->getModel().getQ(state);
			_target = _currentQ;
			_velramp.setTarget(_currentQ, _target);
			_time = 0;
		}

		rw::math::Q getQ(){
			return _target;
		}

		rw::math::Q getQd(){ return _target;}

		Controller* getController(){ return this;};

	private:
		dynamics::KinematicDevice *_ddev;
		double _time;
		rw::control::SyncVelocityRamp _velramp;
		rw::math::Q _target;
		rw::math::Q _currentQ;

	};

	typedef rw::common::Ptr<VelRampController> VelRampControllerPtr;

	//! @}
}
}

#endif /*VELRAMPCONTROLLER_HPP_*/
