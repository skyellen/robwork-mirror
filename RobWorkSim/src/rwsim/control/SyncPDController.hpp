#ifndef RWSIM_CONTROL_SYNCPDCONTROLLER_HPP_
#define RWSIM_CONTROL_SYNCPDCONTROLLER_HPP_

//! @file SyncPDController.hpp

#include <rwlibs/control/JointController.hpp>
#include <rwlibs/control/SyncVelocityRamp.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

#include <rwsim/dynamics/RigidDevice.hpp>

#include "PDController.hpp"

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
	class SyncPDController: public rwlibs::control::JointController, public rwlibs::simulation::SimulatedController {

	public:

	    typedef rw::common::Ptr<SyncPDController> Ptr;

		/**
		 * @brief constrictor
		 * @param rdev
		 * @param state
		 */
		SyncPDController(const std::string& name, dynamics::RigidDevice* rdev, const rw::kinematics::State& state);

		//! @brief destructor
		virtual ~SyncPDController(){};

		/**
		 * @brief the PD parameters
		 * @return list of PD parameters
		 */
		std::vector<PDParam> getParameters();

		/**
		 * @brief set the PD parameters
		 * @param params [in] list of parameters. must be same length as DOF
		 * of controlling device
		 */
		void setParameters(const std::vector<PDParam>& params);

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

		std::string getControllerName(){ return getName(); };

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
		void setTargetPos(const rw::math::Q& target);

		//! @copydoc JointController::setTargetVel
		void setTargetVel(const rw::math::Q& vals);

		//! @copydoc JointController::setTargetAcc
		void setTargetAcc(const rw::math::Q& vals);

		//! @copydoc JointController::getQ
		rw::math::Q getQ(){ return _currentQ;}

		//! @copydoc JointController::getQd
		rw::math::Q getQd(){ return _currentVel;}

        void setEnabled(bool enabled){ _enabled = enabled; };

        bool isEnabled() const { return _enabled; } ;

        rwlibs::control::Controller::Ptr getControllerHandle(rwlibs::simulation::Simulator::Ptr sim){ return this;}

	private:
		dynamics::RigidDevice *_ddev;
		double _time;
		rw::math::Q _target;
		rw::math::Q _lastError;
		rw::control::SyncVelocityRamp _velramp;
		rw::math::Q _currentQ, _currentVel, _targetVel;
		rw::math::Q _maxVel;
		rw::math::Q _x;
		ControlMode _mode;
		bool _enabled;

	};

	//! @}
}
}

#endif /*SyncPDController_HPP_*/
