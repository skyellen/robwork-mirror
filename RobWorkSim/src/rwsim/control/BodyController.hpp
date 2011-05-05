#ifndef RWSIM_CONTROL_BODYCONTROLLER_HPP_
#define RWSIM_CONTROL_BODYCONTROLLER_HPP_

//! @file BodyController.hpp

#include <rwlibs/control/Controller.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

#include <rwsim/dynamics/RigidDevice.hpp>

#include <list>

namespace rwsim {
namespace control {
	//! @addtogroup control @{

	/**
	 * @brief a JointController that use a PD loop on each joint
	 * to control the velocity such that the position target is
	 * reached.
	 */
	class BodyController: public rwlibs::control::Controller, public rwlibs::simulation::SimulatedController {
	public:

	    typedef rw::common::Ptr<BodyController> Ptr;
		/**
		 * @brief constructor
		 * @param rdev [in] device that is to be controlled
		 * @param state [in] target state
		 * @param cmode [in] the control mode used
		 * @param pdparams [in] list of pd parameters. must be same length as number of joints.
		 * @param dt [in] the sampletime (time between samples in seconds) used in the control
		 * loop, this should be larger than the expected update sample time.
		 */
		BodyController(const std::string& name);

		/**
		 * @brief destructor
		 */
		virtual ~BodyController(){};

		void setTarget(rwsim::dynamics::Body *body, const rw::math::Transform3D<>& target, rw::kinematics::State& state);

		rw::math::Transform3D<> getTarget(rwsim::dynamics::Body *body);

		void disableBodyControl(rwsim::dynamics::Body *body);

		void disableBodyControl();

		//! @copydoc SimulatedController::update
		void update(double dt, rw::kinematics::State& state);

		//! @copydoc SimulatedController::reset
		void reset(const rw::kinematics::State& state);

		//! @copydoc SimulatedController::getController
		Controller* getController(){ return this; };

		std::string getControllerName(){ return getName(); };


	private:
		BodyController();

	private:
		/*struct ControlData {
		    rw::control::SyncVelocityRamp _ramp;
		    rw::math::Transform3D<> _target;
		    double _time;
		};*/
		std::map<rwsim::dynamics::Body*, rw::math::Transform3D<> > _bodyMap;
		std::list<rwsim::dynamics::Body*> _bodies;
	};

	//! @}
}
}

#endif /*PDController_HPP_*/
