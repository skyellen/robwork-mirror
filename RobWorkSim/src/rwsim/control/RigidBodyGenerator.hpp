#ifndef RWSIM_CONTROL_RIGIDBODYGENERATOR_HPP_
#define RWSIM_CONTROL_RIGIDBODYGENERATOR_HPP_



#include <rwlibs/simulation/SimulatedController.hpp>


namespace rwsim {
namespace control {
	//! @addtogroup rwsim_control
	//! @{

	/**
	 * @brief a generator of rigid bodies.
	 */
	class RigidBodyGenerator: public rwlibs::simulation::SimulatedController {
	public:

	    typedef rw::common::Ptr<RigidBodyGenerator> Ptr;


	    /**
		 * @brief constructor
		 * @param name [in] name of controller
		 * @param pos [in] position from which bodies are generated
		 * @param geoms
		 */
	    RigidBodyGenerator(
		        const std::string& name,
		        const rw::math::Vector3D<>& pos,
		        std::vector<rw::geometry::Geometry::Ptr> geoms,
		        double delay,
		        double lifetime = -1,
		        bool repeat = false
				);


		/**
		 * @brief destructor
		 */
		virtual ~RigidBodyGenerator(){};

		//! @copydoc SimulatedController::update
		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

		//! @copydoc SimulatedController::reset
		void reset(const rw::kinematics::State& state);

		//! @copydoc SimulatedController::getController
		Controller* getController(){ return NULL; };

        void setEnabled(bool enabled){ _enabled = enabled; };

        bool isEnabled(){ return _enabled; } ;

	private:
		bool _enabled;
	};
	//! @}
}
}

#endif /*PDController_HPP_*/
