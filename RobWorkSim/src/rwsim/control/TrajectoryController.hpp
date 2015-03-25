#ifndef RWSIM_CONTROL_TRAJECTORYCONTROLLER_HPP
#define RWSIM_CONTROL_TRAJECTORYCONTROLLER_HPP

//! @file TrajectoryController.hpp

#include <rwlibs/control/JointController.hpp>
#include <rwlibs/control/SyncVelocityRamp.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

#include <rwsim/dynamics/RigidDevice.hpp>

namespace rwsim {
namespace control {
	//! @addtogroup rwsim_control
	//! @{

	/**
	 * @brief a JointController that use a PD loop on each joint
	 * to control the velocity such that the position target is
	 * reached at the same time. The PD controls the joint position and
	 * velocity from a generated synchronous ramp profile.
	 */
	class TrajectoryController: public rwlibs::control::JointController, public rwlibs::simulation::SimulatedController{

	public:
		/**
		 * @brief constructor
		 * @param rdev
		 * @param state
		 */
		TrajectoryController(RigidDevice* rdev, const rw::kinematics::State& state):
			JointController(&rdev->getModel()),
			_ddev(rdev),
			_time(0.0),
			_target(rdev->getModel().getQ(state)),
			_lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
			_velramp(&(rdev->getModel())),
			_currentQ(_target)
		{
			_velramp.setTarget(_target,_target);
		}

		//! @brief destructor
		virtual ~TrajectoryController(){};

		/**
		 * @brief the time between samples
		 * @return the sample time in seconds
		 */
		double getSampleTime();

		/**
		 * @brief set the time between samples in seconds
		 * @param stime [in] sample time
		 */
		void setSampleTime(double stime);

		//! @copydoc SimulatedController::update
		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

		//! @copydoc SimulatedController::reset
		void reset(const rw::kinematics::State& state);

		//! @copydoc SimulatedController::getController
		Controller* getController(){ return this; };

		////// inherited from JointController

		/**
		 * @copydoc JointController::getControlModes
		 *
		 * This controller supports both position and velocity control.
		 */
		unsigned int getControlModes(){return _mode;}

		//! @copydoc JointController::setControlModes
		void setControlMode(ControlMode mode);

		//! @copydoc JointController::setTargetPos
		void setTargetPos(const rw::math::Q& target){_target = target; }

		//! @copydoc JointController::setTargetVel
		void setTargetVel(const rw::math::Q& vals){_targetVel = vals; }

		//! @copydoc JointController::setTargetAcc
		void setTargetAcc(const rw::math::Q& vals){};

		//! @copydoc JointController::getQ
		rw::math::Q getQ(){ return _currentQ;}

		//! @copydoc JointController::getQd
		rw::math::Q getQd(){ return _currentVel;}

        void setEnabled(bool enabled){ _enabled = enabled; };

        bool isEnabled() const { return _enabled; } ;


        rwlibs::control::Controller::Ptr getControllerHandle(rwlibs::simulation::Simulator::Ptr sim){ return this;}

	private:
		RigidDevice *_ddev;
		double _time;
		rw::math::Q _target;
		rw::math::Q _lastError;
		rw::sandbox::SyncVelocityRamp _velramp;
		rw::math::Q _currentQ;
		rw::math::Q _maxVel;
		rw::math::Q _x;
		int _mode;
		bool _enabled;

	};
	//! @}
}
}

#endif /*TrajectoryController_HPP_*/
