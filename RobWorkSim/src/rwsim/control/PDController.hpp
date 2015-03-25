#ifndef RWSIM_CONTROL_PDCONTROLLER_HPP_
#define RWSIM_CONTROL_PDCONTROLLER_HPP_

//! @file PDController.hpp

#include <rwlibs/control/JointController.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

#include <rwsim/dynamics/RigidDevice.hpp>

namespace rwsim {
namespace control {
	//! @addtogroup rwsim_control
	//! @{

	//! @brief struct for holding PD parameters
	struct PDParam {
		PDParam():P(10),D(0.003){}
		PDParam(double p, double d):P(p),D(d){};
		double P; //! the proportional parameter
		double D; //! the derivative parameter
	};

	/**
	 * @brief a JointController that use a PD loop on each joint
	 * to control the velocity such that the position target is
	 * reached.
	 */
	class PDController: public rwlibs::control::JointController, public rwlibs::simulation::SimulatedController {
	public:

	    typedef rw::common::Ptr<PDController> Ptr;
		/**
		 * @brief constructor
		 * @param rdev [in] device that is to be controlled
		 * @param state [in] target state
		 * @param cmode [in] the control mode used
		 * @param pdparams [in] list of pd parameters. must be same length as number of joints.
		 * @param dt [in] the sampletime (time between samples in seconds) used in the control
		 * loop, this should be larger than the expected update sample time.
		 */
		PDController(
		        const std::string& name,
		        dynamics::DynamicDevice::Ptr rdev,
				ControlMode cmode,
				const std::vector<PDParam>& pdparams,
				double dt
				);

		/**
		 * @brief constructor
		 * @param rdev [in] device that is to be controlled
		 * @param cmode [in] the control mode used
		 * @param pdparam [in] pd parameter - used for all joints
		 * @param dt [in] the sampletime (time between samples in seconds) used in the control
		 * loop, this should be larger than the expected update sample time.
		 */
		PDController(
		        const std::string& name,
		        dynamics::DynamicDevice::Ptr rdev,
				ControlMode cmode,
				const PDParam& pdparam,
				double dt
				);

		/**
		 * @brief destructor
		 */
		virtual ~PDController(){};

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
		PDController();

	private:
		dynamics::DynamicDevice::Ptr _ddev;
		rw::math::Q _maxVel;
		rw::math::Q _lastError, _currentError, _target, _currentQ, _currentVel;
		rw::math::Q _targetVel;
		std::vector<PDParam> _pdparams;
		ControlMode _mode;
		double _stime, _accTime; // sample time
		rw::math::Q  _P, _D;
		bool _enabled;
	};

	typedef rw::common::Ptr<PDController> PDControllerPtr;
	//! @}
}
}

#endif /*PDController_HPP_*/
