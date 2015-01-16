#ifndef BT_SIMULATOR_HPP_
#define BT_SIMULATOR_HPP_

#include <rw/common/Cache.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>

// Bullet includes
#include <bullet/LinearMath/btAlignedObjectArray.h>
#include <bullet/LinearMath/btQuickprof.h>

// Forward Declarations
class btDynamicsWorld;
class btDiscreteDynamicsWorld;
class btRigidBody;
class btTypedConstraint;
class btBroadphaseInterface;
class btCollisionShape;
class btCollisionDispatcher;
class btConstraintSolver;
class btDefaultCollisionConfiguration;

namespace rwsim { namespace dynamics { class KinematicBody; } }

namespace rwsim {
namespace simulator {
	class BtDebugRender;

    class BtSimulator: public PhysicsEngine
    {
    public:
        typedef rw::common::Cache<std::string, btCollisionShape> ColCache;

        //! empty constructor
        BtSimulator();

		//! @copydoc PhysicsEngine::load
		void load(rwsim::dynamics::DynamicWorkCell::Ptr dwc);


        class btDevice
        {
        public:

            virtual ~btDevice()
            {
            }
            ;

            virtual void update(double dt, rw::kinematics::State& state) = 0;

            virtual void postUpdate(rw::kinematics::State& state) = 0;
        protected:
            btDevice()
            {
            }
            ;
        };

    private:
        ColCache _colCache;
        ///this is the most important class
        btDiscreteDynamicsWorld* m_dynamicsWorld;

        //keep the collision shapes, for deletion/cleanup
        btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

        btBroadphaseInterface* m_overlappingPairCache;

        btCollisionDispatcher* m_dispatcher;

        btConstraintSolver* m_solver;

        btDefaultCollisionConfiguration* m_collisionConfiguration;

        dynamics::DynamicWorkCell::Ptr _dwc;
        dynamics::MaterialDataMap _materialMap;
		dynamics::ContactDataMap _contactMap;

        std::vector<btRigidBody*> _btBodies;
        std::vector<dynamics::RigidBody::Ptr> _rwBodies;

        std::vector<btRigidBody*> _btLinks;
        std::vector<rw::common::Ptr<dynamics::KinematicBody> > _rwLinks;

        std::vector<dynamics::DynamicDevice::Ptr> _devices;

        std::vector<btDevice*> _btDevices;

        // map from rw frame to its bullet rigidbody equivalent
        std::map<rw::kinematics::Frame*, btRigidBody*> _rwFrameToBtBody;
        std::map<btRigidBody*, rw::kinematics::Frame*> _rwBtBodyToFrame;

        std::map<rw::models::Joint*, btTypedConstraint*> _jointToConstraintMap;

        rw::common::Ptr<BtDebugRender> _render;

        std::vector<rwlibs::simulation::SimulatedController::Ptr> _controllers;
        btClock m_clock;

        double _time, _dt;
        bool _initPhysicsHasBeenRun;

    public:
        BtSimulator(dynamics::DynamicWorkCell *dwc);

        virtual ~BtSimulator()
        {
            exitPhysics();
        }

        std::vector<btRigidBody*> getBodies(){
        	return _btBodies;
        }

        btAlignedObjectArray<btCollisionShape*> getCollisionShapes(){
        	return m_collisionShapes;
        }

    	//! @copydoc PhysicsEngine::setContactDetector
    	bool setContactDetector(rw::common::Ptr<rwsim::contacts::ContactDetector> detector);

        void initPhysics(rw::kinematics::State& state);

        void step(double dt, rw::kinematics::State& state);

        /**
         * @brief reset velocity and acceleration of all bodies to 0. And sets the position of all bodies
         * to that described in state
         */
        void resetScene(rw::kinematics::State& state);

        /**
         * @brief cleans up the allocated storage for bullet physics
         */
        void exitPhysics();

    	//! @copydoc Simulator::createDebugRender
		drawable::SimulatorDebugRender::Ptr createDebugRender();

        double getTime()
        {
            return _time;
        }

        btDynamicsWorld* getBtWorld() const;

        btRigidBody* createRigidBody(rw::geometry::Geometry::Ptr geometry, double mass, const rw::kinematics::State& state,
                                     double margin);
        void setEnabled(dynamics::RigidBody* body, bool enabled)
        {
        }
        ;

        rw::common::PropertyMap& getPropertyMap()
        {
            return _propertyMap;
        }
        ;

        void emitPropertyChanged();

		void addController(rwlibs::simulation::SimulatedController::Ptr controller){
			_controllers.push_back(controller);
		}
		void removeController(rwlibs::simulation::SimulatedController::Ptr controller){}

        void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);
        void removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);

		dynamics::DynamicWorkCell::Ptr getDynamicWorkCell(){ return _dwc;};

		//! get current gravity
		rw::math::Vector3D<> getGravity(){ return _dwc->getGravity(); }

        // -------------- under here is so far just to compile
		void DWCChangedListener(dynamics::DynamicWorkCell::DWCEventType type, boost::any data);

		//! @copydoc Simulator::setEnabled
		void setEnabled(dynamics::Body::Ptr body, bool enabled);

		void setDynamicsEnabled(dynamics::Body::Ptr body, bool enabled);

		void addBody(rwsim::dynamics::Body::Ptr body, rw::kinematics::State& state);
		/**
		 * @brief Add a Constraint between two bodies.
		 * @param constraint [in] a pointer to the RobWork constraint.
		 */
		void addConstraint(rwsim::dynamics::Constraint::Ptr constraint);
		void addDevice(rwsim::dynamics::DynamicDevice::Ptr device, rw::kinematics::State& state);
		void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State& state);


		void attach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);
		void detach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

		void disableCollision(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

		void enableCollision(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

		std::vector<rwlibs::simulation::SimulatedSensor::Ptr> getSensors(){
			return _sensors;
		}

    private:
        rw::common::PropertyMap _propertyMap;
		std::vector<rwlibs::simulation::SimulatedSensor::Ptr> _sensors;


    };

}
}
#endif /*BtSimulator_HPP_*/
