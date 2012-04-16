#ifndef RWSIM_CONTROL_BODYCONTROLLER_HPP_
#define RWSIM_CONTROL_BODYCONTROLLER_HPP_

//! @file BodyController.hpp

#include <rwlibs/control/Controller.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/control/SyncVelocityRamp.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <list>

namespace rwsim {
namespace control {
	//! @addtogroup control
	//! @{

	/**
	 * @brief a Controller that use a PD loop to follow a trajectory
	 * generated from different target types. If the body is a Kinematic body then
	 * the velocities of the body is directly controlled. else wrenches are used to
	 * control the body.
	 */
	class BodyController: public rwlibs::control::Controller, public rwlibs::simulation::SimulatedController {
	public:

	    typedef rw::common::Ptr<BodyController> Ptr;

		BodyController(const std::string& name);

		/**
		 * @brief destructor
		 */
		virtual ~BodyController(){};

		/**
		 * @brief sets the target transform of a body. The target is defined in world frame.
		 * @param body
		 * @param target
		 * @param state
		 */
		void setTarget(rwsim::dynamics::Body *body, const rw::math::Transform3D<>& target, rw::kinematics::State& state);

		/**
		 * @brief set a target trajectory of a body. The initial configuration of the trajectory must match
		 * the current configuration of the body.
		 *
		 * A Kinematic body will follow the exact path of the trajectory whereas a RigidBody
		 * will use a PD controller to follow the trajectory
		 *
		 * @param body [in] the body that should be moved
		 * @param traj
		 * @param state
		 */
		void setTarget(rwsim::dynamics::Body* body, rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr traj, rw::kinematics::State& state);

		rw::math::Transform3D<> getTarget(rwsim::dynamics::Body *body);

		rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr getTargetTrajectory(rwsim::dynamics::Body *body);

		void disableBodyControl(rwsim::dynamics::Body *body);

		void disableBodyControl();

		//! @copydoc SimulatedController::update
		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

		//! @copydoc SimulatedController::reset
		void reset(const rw::kinematics::State& state);

		//! @copydoc SimulatedController::getController
		Controller* getController(){ return this; };

		std::string getControllerName(){ return getName(); };

        void setEnabled(bool enabled){ _enabled = enabled; };

        bool isEnabled(){ return _enabled; } ;

        typedef enum {Pose6DController, TrajectoryController} ControlType;
        struct TargetData {
            TargetData():_type(Pose6DController),_enabled(false){ reset();}
            TargetData(ControlType type):_type(type),_enabled(true){ reset(); }
            void reset(){
                _time = 0;
                _lastTime = 0;
                _lastDt = 0;
            }
            ControlType _type;
            rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr _traj;
            rw::math::Transform3D<> _target;
            double _time, // current time on the trajectory
                   _lastTime, // the time on the trajectory taken at the last non-rollback step
                   _lastDt; // the simulated starting time
            bool _enabled;
        };

	private:
		BodyController();

	private:
		std::map<rwsim::dynamics::Body*, TargetData> _bodyMap;
		//std::map<rwsim::dynamics::Body*, rw::math::Transform3D<> > _bodyMap;
		std::list<rwsim::dynamics::Body*> _bodies;
		bool _enabled;
	};

	//! @}
}
}

#endif /*PDController_HPP_*/
