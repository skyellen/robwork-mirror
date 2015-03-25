#ifndef RWSIM_CONTROL_BeamJointController_HPP_
#define RWSIM_CONTROL_BeamJointController_HPP_

//! @file BeamJointController.hpp

#include <rwlibs/control/JointController.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

#include <rwsim/dynamics/RigidDevice.hpp>

namespace rwsim {
namespace control {
	//! @addtogroup rwsim_control
	//! @{

	/**
	 * @brief The beamjoint controller controls a joint device composed of coupled beam joints.
	 * The coupling of beamjoints is somewhat special since its not an configuration space
	 * coupling (position) but rather a coupling of the forces acting on the joints.
	 *
	 * This controller continuesly updates the torques acting on the joints by considering the
	 * current deflection of the joint.
	 *
	 */
	class BeamJointController: public rwlibs::control::JointController, public rwlibs::simulation::SimulatedController {
	public:


		/**
		 * @brief constructor
		 * @param rdev [in] device that is to be controlled
		 * @param state [in] target state
		 * @param cmode [in] the control mode used
		 * @param pdparams [in] list of pd parameters. must be same length as number of joints.
		 * @param dt [in] the sampletime (time between samples in seconds) used in the control
		 * loop, this should be larger than the expected update sample time.
		 */
		BeamJointController(
		        const std::string& name,
		        dynamics::RigidDevice* rdev,
				ControlMode cmode,
				double dt
				);


		/**
		 * @brief destructor
		 */
		virtual ~BeamJointController(){};


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

        rwlibs::control::Controller::Ptr getControllerHandle(rwlibs::simulation::Simulator::Ptr sim){ return NULL;}

	private:
		BeamJointController();

	private:
		//std::vector<rw::models::BeamJoint*> _beamJoints;

		dynamics::RigidDevice *_ddev;
		rw::math::Q _maxVel;
		rw::math::Q _lastError, _target, _currentQ, _currentVel;
		rw::math::Q _targetVel;
		ControlMode _mode;
		double _stime, _accTime; // sample time
		rw::math::Q  _P, _D;
		bool _enabled;
	};

	typedef rw::common::Ptr<BeamJointController> BeamJointControllerPtr;
	//! @}
}
}

#endif /*BeamJointController_HPP_*/
