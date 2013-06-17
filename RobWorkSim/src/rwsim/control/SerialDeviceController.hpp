#ifndef RWSIM_CONTROL_SERIALDEVICECONTROLLER_HPP_
#define RWSIM_CONTROL_SERIALDEVICECONTROLLER_HPP_

//! @file SerialDeviceController.hpp

#include <rwlibs/control/JointController.hpp>
#include <rwlibs/control/SyncVelocityRamp.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>

#include <rw/trajectory/Trajectory.hpp>

#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Wrench6D.hpp>

namespace rwsim {
namespace control {
	//! @addtogroup rwsim_control
	//! @{

	/**
	 * @brief a SerialDeviceController that enables control of the robot in both
	 * position, velocity and force modes. Also both joint and cartesean control
	 * is available.
	 *
	 */
	class SerialDeviceController: public rwlibs::simulation::SimulatedController {

	public:
		typedef rw::common::Ptr<SerialDeviceController> Ptr;
		/**
		 *
		 * @param name [in] controller name
		 * @param ddev [in]
		 */
		SerialDeviceController(const std::string& name, dynamics::DynamicDevice::Ptr ddev);

		SerialDeviceController(const std::string& name, dynamics::RigidDevice::Ptr ddev);


		//! destructor
		virtual ~SerialDeviceController();

		//! the max linear/translational velocity in m/s
		void setMaxLinearVelocity( double maxVel );

		//! the max angular velocity in rad/s
		void setMaxAngularVelocity( double maxVel );

		//! @brief move robot in a linear Cartesian path
		bool moveLin(const rw::math::Transform3D<>& target, float speed=100, float blend=0);

		//! @brief move robot from point to point
		bool movePTP(const rw::math::Q& target, float speed=100, float blend=0);

		//! @brief move robot from point to point but using a pose as target (require invkin)
		virtual bool movePTP_T(const rw::math::Transform3D<>& target, float speed=100, float blend=0);

		//! @brief move robot in a servoing fasion
		virtual bool moveVelQ(const rw::math::Q& target_joint_velocity);

		virtual bool moveVelT(const rw::math::VelocityScrew6D<>& vel);

		//! move robot with a hybrid position/force control
		virtual bool moveLinFC(const rw::math::Transform3D<>& target,
								  const rw::math::Wrench6D<>& wtarget,
								  float selection[6],
								  std::string refframe,
								  rw::math::Transform3D<> offset,
								  float speed = 100,
								  float blend = 0);

		//! hard stop the robot,
		bool stop();

		//! pause the robot, should be able to continue trajectory
		bool pause();

		//! enable safe mode, so that robot stops when collisions are detected
		bool setSafeModeEnabled(bool enable);

		rw::math::Q getQ();
		rw::math::Q getQd();

		bool isMoving();

		// simulated controller stuff

		void update(const rwlibs::simulation::Simulator::UpdateInfo& info,
					 rw::kinematics::State& state);

		void updateFTcontrolWrist(const rwlibs::simulation::Simulator::UpdateInfo& info,
					 rw::kinematics::State& state);


        std::string getControllerName(){ return _name; };

        void reset(const rw::kinematics::State& state){ }

        rwlibs::control::Controller* getController(){ return NULL; }

        void setEnabled(bool enabled){ _enabled=enabled; }

        bool isEnabled(){ return _enabled; };

	protected:
        /**
         * A compiled target may be a series of targets such as PTP that has been compiled into
         * one target.
         */
		struct CompiledTarget {
			CompiledTarget():ftcontrol(false),fttime(0.01){}
			rw::trajectory::QTrajectory::Ptr qtraj;
			rw::trajectory::Transform3DTrajectory::Ptr t3dtraj;
			rw::math::Wrench6D<> _wrenchTarget;
			bool ftcontrol;
			double fttime;
			// the id of the last target defining this CompiledTarget
			int toId;

			bool isFinished(double time){
				if( (qtraj!=NULL) && time<qtraj->endTime())
					return false;
				if( (t3dtraj!=NULL) && time<t3dtraj->endTime())
					return false;
				if( (ftcontrol) && time<fttime )
					return false;
				return true;
			}
		};

		typedef enum{PTP, PTP_T, Lin, VelQ, VelT, LinFC} TargetType;
		struct Target {
			TargetType type;
			rw::math::Q q_target;
			rw::math::Q q_vel_target;
			rw::math::Transform3D<> lin_target;
			rw::math::VelocityScrew6D<> lin_vel_target;

			float speed, blend;

			// this is only for internal use
			rw::math::Q q_start;
			rw::math::Transform3D<> t_start;
			int id;
		};

        //! create trajectory from current joint position and joint velocity
        CompiledTarget makeTrajectory(const std::vector<Target>& targets, rw::kinematics::State& state);

	private:
		dynamics::DynamicDevice::Ptr _ddev;
		dynamics::RigidDevice::Ptr _rdev;
		double _time;
		rw::math::Q _target;
		rw::math::Q _currentQ, _currentQd;
		bool _enabled, _stop, _pause;
		rw::invkin::JacobianIKSolver::Ptr _solver;

		bool _targetAdded;




		void addTarget(const Target& target);
		std::vector<Target> _targetQueue;
		int _currentTargetIdx;
		boost::mutex _targetMutex;
		rw::trajectory::QTrajectory::Ptr _currentTraj;
		double _currentTrajTime;
		std::string _name;


		double _linVelMax, _angVelMax;

		int _currentCompiledTarget;
		std::deque<CompiledTarget> _compiledTargets;
		CompiledTarget _executingTarget;
		unsigned int _idCnt;

		bool _isRunning;


		// Hybrid/Force torque control
		Eigen::VectorXd _q_error, _q_error_last;
		Eigen::Matrix<double,6,1> _S;
		rw::kinematics::Frame *_taskFrame;
		rw::math::Transform3D<> _bXd;
		rw::math::Wrench6D<> _bFd;

		rw::math::Transform3D<> _bXe_last;
		rw::math::VelocityScrew6D<> _bXde_last;
		std::ofstream _out;

	};

	//! @}
}
}

#endif /*VELRAMPCONTROLLER_HPP_*/
