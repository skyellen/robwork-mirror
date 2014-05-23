#ifndef BT_SIMULATOR_HPP_
#define BT_SIMULATOR_HPP_

// RW includes
#include <rw/common/Cache.hpp>
#include <rw/math/Math.hpp>

// dynamic includes
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/DynamicDevice.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>

#include "BtDebugRender.hpp"

// Bullet includes
#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btQuickprof.h>
#include <LinearMath/btAlignedObjectArray.h>

//class	btCollisionShape;
class btDynamicsWorld;
class btRigidBody;
class btTypedConstraint;
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

namespace rwsim {
namespace simulator {

    class BtSimulator: public PhysicsEngine
    {
    public:
        typedef rw::common::Cache<std::string, btCollisionShape> ColCache;

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
        btDynamicsWorld* m_dynamicsWorld;

        //keep the collision shapes, for deletion/cleanup
        btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

        btBroadphaseInterface* m_overlappingPairCache;

        btCollisionDispatcher* m_dispatcher;

        btConstraintSolver* m_solver;

        btDefaultCollisionConfiguration* m_collisionConfiguration;

        dynamics::DynamicWorkCell *_dwc;

        std::vector<btRigidBody*> _btBodies;
        std::vector<dynamics::RigidBody::Ptr> _rwBodies;

        std::vector<btRigidBody*> _btLinks;
        std::vector<dynamics::KinematicBody::Ptr> _rwLinks;

        std::vector<dynamics::DynamicDevice::Ptr> _devices;

        std::vector<btDevice*> _btDevices;

        // map from rw frame to its bullet rigidbody equivalent
        std::map<rw::kinematics::Frame*, btRigidBody*> _rwFrameToBtBody;
        std::map<btRigidBody*, rw::kinematics::Frame*> _rwBtBodyToFrame;

        std::map<rw::models::Joint*, btTypedConstraint*> _jointToConstraintMap;

        btClock m_clock;

        double _time, _dt;

    public:
        BtSimulator(dynamics::DynamicWorkCell *dwc);

        virtual ~BtSimulator()
        {
            exitPhysics();
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
         * @brief cleans up the allocated storage fo bullet physics
         */
        void exitPhysics();

        double getTime()
        {
            return _time;
        }

        btDynamicsWorld* getBtWorld()
        {
            return m_dynamicsWorld;
        }

        btRigidBody* createRigidBody(rw::kinematics::Frame* bframe, double mass, const rw::kinematics::State& state,
                                     double margin);

        drawable::SimulatorDebugRender::Ptr createDebugRender()
        {
            return NULL;
        }

        void setEnabled(dynamics::RigidBody* body, bool enabled)
        {
        }
        ;

        rw::common::PropertyMap& getPropertyMap()
        {
            return _propertyMap;
        }
        ;

        void emitPropertyChanged()
        {
        }

        void addController(rwlibs::simulation::SimulatedController::Ptr controller){};
        void removeController(rwlibs::simulation::SimulatedController::Ptr controller){};
        void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor){};
        void removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor){};
        std::vector<rwlibs::simulation::SimulatedSensor::Ptr> getSensors(){};

    private:
        rw::common::PropertyMap _propertyMap;
    };

}
}
#endif /*BtSimulator_HPP_*/
