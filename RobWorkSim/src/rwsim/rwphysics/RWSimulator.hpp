#ifndef RWSIMULATOR_HPP_
#define RWSIMULATOR_HPP_

#include "ContactGraph.hpp"

#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rw/kinematics/FrameMap.hpp>

#include <rw/kinematics/State.hpp>
#include "ConstraintSolver.hpp"
#include "RWDebugRender.hpp"
#include "BodyIntegrator.hpp"
#include "BodyController.hpp"
#include "ConstantForceManipulator.hpp"
#include "RWBodyPool.hpp"

namespace rwsim {
namespace dynamics{
	class RigidBody;
	class KinematicBody;
}

namespace simulator {

    class CNodePool;
    class ContactModelFactory;


	class RWSimulator: public PhysicsEngine
	{
	public:
		/**
		 * @brief constructor
		 */
		RWSimulator(dynamics::DynamicWorkCell::Ptr dwc);

		/**
		 * @brief default destructor
		 */
		virtual ~RWSimulator(){
			exitPhysics();
		}

		/**
		 * @copydoc Simulator::initPhysics
		 */
		void initPhysics(rw::kinematics::State& state);

		/**
		 * @copydoc Simulator::step
		 */
		void step(double dt, rw::kinematics::State& state);

		/**
		 * @copydoc Simulator::resetScene
		 */
		void resetScene(rw::kinematics::State& state);

		/**
		 * @copydoc Simulator::exitPhysics
		 */
		void exitPhysics();

		/**
		 * @copydoc Simulator::getTime
		 */
		double getTime(){
			return _time;
		}

		/**
		 * @copydoc Simulator::createDebugRender
		 */
		drawable::SimulatorDebugRender* createDebugRender(){
			return new RWDebugRender(*_dwc);
		}

		virtual void setEnabled(dynamics::Body* body, bool enabled){}

		rw::common::PropertyMap& getPropertyMap()
		{ return _propertyMap; };

		void emitPropertyChanged(){}

		void addController(rwlibs::simulation::SimulatedController::Ptr controller){
			_controllers.push_back(controller);
		}

		void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor){
			_sensors.push_back(sensor);
		}

		void removeController(rwlibs::simulation::SimulatedController::Ptr controller){}

		void removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor){};

		std::vector<rwlibs::simulation::SimulatedSensor::Ptr> getSensors(){ return _sensors;};

	private:
		rw::common::PropertyMap _propertyMap;

		/**
		 * @brief a step of dt if no penetrating collision occour, else a
		 * step until the collision occour.
		 * @param dt [in] the step to take.
		 * @param state [in] current state
		 * @return the actual time step dt that was taken
		 */
		double internalStep(double dt, rw::kinematics::State& state);

		void rollBack(rw::kinematics::State &state);

	private:
		rwsim::dynamics::DynamicWorkCell::Ptr _dwc;

		CNodePool *_pool;
		ContactModelFactory *_factory;
		ContactGraph *_cgraph;
		ConstraintSolver *_solver;

		RWBodyPool _bodyPool;
		std::vector<BodyController*> _manipulators;

		std::vector<RWBody*> _bodies;

		std::vector<rwsim::dynamics::RigidBody*> _rbodies;
		std::vector<rwsim::dynamics::KinematicBody*> _kbodies;
		std::vector<BodyIntegrator*> _integrators;

		double _time;

		rw::kinematics::FrameMap<RWBody*> _frameToBody;

		ConstantForceManipulator *_gravityManipulator;

		std::vector<rwlibs::simulation::SimulatedController::Ptr> _controllers;
		std::vector<rwlibs::simulation::SimulatedSensor::Ptr> _sensors;
	};

}
}

#endif /*RWSIMULATOR_HPP_*/
