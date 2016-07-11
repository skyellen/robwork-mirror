#ifndef RWSIM_CONTROL_SERIALDEVICECONTROLLER_HPP_
#define RWSIM_CONTROL_SERIALDEVICECONTROLLER_HPP_

//! @file SerialDeviceController.hpp

#include <rwlibs/simulation/SimulatedController.hpp>

#include <rw/trajectory/Trajectory.hpp>

#include <rw/math/Q.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Wrench6D.hpp>

#include <rwsim/dynamics/DynamicDevice.hpp>

#include <boost/thread/mutex.hpp>

#include <deque>
#include <fstream>

// Forward declarations
namespace rw { namespace invkin { class JacobianIKSolver; }}
namespace rw { namespace sensor { class FTSensor; }}
namespace rwsim { namespace dynamics { class RigidDevice; }}

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

		/**
		 * @brief construct using RigidDevice.
		 * @param name [in] name of controller
		 * @param ddev [in] the RigidDevice that should be controlled
		 */
		SerialDeviceController(const std::string& name, rw::common::Ptr<rwsim::dynamics::RigidDevice> ddev);

		//! destructor
		virtual ~SerialDeviceController();


		/////////////////// Move commandoes. The move commandoes are queued

		//! @brief move robot in a linear Cartesian path
		bool moveLin(const rw::math::Transform3D<>& target, float speed=100, float blend=0);

		//! @brief move robot from point to point
		bool movePTP(const rw::math::Q& target, float speed=100, float blend=0);

		//! @brief move robot from point to point but using a pose as target (require invkin)
		bool movePTP_T(const rw::math::Transform3D<>& target, float speed=100, float blend=0);

		//! @brief
		bool moveTraj(const rw::trajectory::QTrajectory::Ptr traj, float speed=100);

		/**
		 *  @brief flushes the move command queue. This is usefull for braking the current trajectory without
		 *  stopping the robot. However, if no commands are placed on the queue after calling flush then the
		 *  robot will stop.
		 */
		void flushQueue();


		//////////////////// Servo commandoes. The servo commandoes does not get queued

		//! @brief move robot in a servoing fasion
		virtual bool moveVelQ(const rw::math::Q& target_joint_velocity);

		virtual bool moveVelT(const rw::math::VelocityScrew6D<>& vel);

		//! move robot with a hybrid position/force control
		virtual bool moveLinFC(const rw::math::Transform3D<>& target,
								  const rw::math::Wrench6D<>& wtarget,
								  float selection[6],
								  std::string refframe,
								  rw::math::Rotation3D<> offset,
								  float speed = 100,
								  float blend = 0);

		/////////////////// Other control commands that effects the robot movements.
		//! hard stop the robot,
		bool stop();

		/**
		 * @brief stops the robot but without flushing the command queue. Please notice that
		 * the trajectories will
		 */
		bool pause();

		bool isStopped(){ return _stop; } ;

		/**
		 * @brief reenable control after calling stop
		 * @return
		 */
		bool start();

		//! enable safe mode, so that robot stops when collisions are detected
		bool setSafeModeEnabled(bool enable);

		//! the max linear/translational velocity in m/s
		void setMaxLinearVelocity( double maxVel );

		//! the max angular velocity in rad/s
		void setMaxAngularVelocity( double maxVel );


		//////////////////   get state information

		rw::math::Q getQ();

		rw::math::Q getQd();

		bool isMoving();


        dynamics::DynamicDevice::Ptr getDynamicDevice(){return _ddev;};


		//////////////////////////   simulated controller stuff

		void update(const rwlibs::simulation::Simulator::UpdateInfo& info,
					 rw::kinematics::State& state);

		void updateFTcontrolWrist(const rwlibs::simulation::Simulator::UpdateInfo& info,
					 rw::kinematics::State& state);


        std::string getControllerName(){ return _name; };

        void reset(const rw::kinematics::State& state){ }

        rwlibs::control::Controller* getController(){ return NULL; }

        void setEnabled(bool enabled){ _enabled=enabled; }

        bool isEnabled() const { return _enabled; };

        /**
         * @brief Set a FTSensor.
         *
         * When using hybrid position and force/torque control it is important to set the sensor used, such
         * that the measured force/torque can be taken into account when calculating the required motor forces.
         *
         * @param sensor [in] a pointer to the FTSensor (must have its frame at one of the joints of the robot).
         */
        void setFTSensor(rw::sensor::FTSensor* sensor);

        rwlibs::control::Controller::Ptr getControllerHandle(rwlibs::simulation::Simulator::Ptr sim){ return NULL; }


	protected:
        /**
         * A compiled target may be a series of targets such as PTP that has been compiled into
         * one target.
         */
		struct CompiledTarget {
			CompiledTarget():ftcontrol(false),velcontrol(false),fttime(0.01){}
			rw::trajectory::QTrajectory::Ptr qtraj;
			rw::trajectory::Transform3DTrajectory::Ptr t3dtraj;
			rw::math::Wrench6D<> _wrenchTarget;
			rw::math::VelocityScrew6D<> _screw;
			bool ftcontrol,velcontrol;
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

		typedef enum{PTP, PTP_T, Lin, VelQ, VelT, LinFC, TrajQ} TargetType;
		struct Target {
			TargetType type;
			rw::math::Q q_target;
			rw::math::Q q_vel_target;
			rw::math::Transform3D<> lin_target;
			rw::math::VelocityScrew6D<> lin_vel_target;
			rw::trajectory::QTrajectory::Ptr qtraj;

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
		rw::common::Ptr<rwsim::dynamics::RigidDevice> _rdev;
		rw::math::Q _target;
		rw::math::Q _currentQ, _currentQd;
		bool _enabled, _stop, _pause;
		rw::common::Ptr<rw::invkin::JacobianIKSolver> _solver;

		bool _targetAdded;




		void addTarget(const Target& target);
		std::vector<Target> _targetQueue;
		boost::mutex _targetMutex;
		rw::trajectory::QTrajectory::Ptr _currentTraj;
		double _currentTrajTime;
		std::string _name;


		double _linVelMax, _angVelMax;

		std::deque<CompiledTarget> _compiledTargets;
		CompiledTarget _executingTarget;
		unsigned int _idCnt;

		// Hybrid/Force torque control
		Eigen::VectorXd _q_error, _q_error_last;
		Eigen::Matrix<double,6,1> _S;
		rw::kinematics::Frame *_taskFrame;
		rw::math::Rotation3D<> _eRoffset;
		rw::math::Transform3D<> _bXd;
		rw::math::Wrench6D<> _bFd;

		rw::math::Transform3D<> _bXe_last;
		rw::math::VelocityScrew6D<> _bXde_last;
		std::ofstream _out;

		rw::sensor::FTSensor* _ftSensor;

	};

	//! @}
}
}

#endif /*VELRAMPCONTROLLER_HPP_*/
