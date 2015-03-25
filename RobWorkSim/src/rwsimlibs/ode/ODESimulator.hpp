/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWSIM_SIMULATOR_ODESIMULATOR_HPP_
#define RWSIM_SIMULATOR_ODESIMULATOR_HPP_

#define dDOUBLE 1

#include <ode/ode.h>

#include <rw/sensor/Sensor.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/kinematics/FramePairMap.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <rwsim/sensor/SimulatedTactileSensor.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/MaterialDataMap.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactManifold.hpp>

#include <rwsim/simulator/PhysicsEngineFactory.hpp>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/unordered_map.hpp>

#include "ODEUtil.hpp"
#include "ODEMaterialMap.hpp"
#include "ODEJoint.hpp"
#include "ODEConstraint.hpp"
#include "ODEDevice.hpp"
#include "ODEBody.hpp"
#include "ODETactileSensor.hpp"

namespace rwsim { namespace contacts { class ContactDetector; }}

namespace rwsim {
namespace simulator {

	class ODEDebugRender;

	/**
	 * @brief an implementation that use the physics engine ODE to implement
	 * the \b Simulator interface of RWSim. All information into the simulator
	 * is derived through RWSim classes.
	 *
	 * Supported objects types include Rigid, Kinematic and fixed
	 * Supported device types include Rigid and Kinematic
	 *
	 * The simulation step looks like this
	 *
	 * 1 updateControllers()
	 * 2 updateDevices() // sets the target velocity of devices
	 * 3 performContactDetection()
	 * 4 performConstraintSolving()
	 * 5 updateSensorStates()
	 * 6 updateDeviceStates()
	 * 7 updateBodyStates()
	 *
	 *
	 * The ODE physics engine support several engine specific properties. These can be tweaked to
	 * obtain higher performance or robustness.
	 * ODE specific options:
	 *
	 *
	 *
	 * Only used when RobWork contact generation is used (its the default).
	 * \b MaxSepDistance - float - All triangles within a distance \b MaxSepDistance is used
	 * in the contact generation.
	 *
	 *
	 * object specific:
	 *
	 * SoftLayer - float -
	 *
	 * example:
	 * \verbatim
	 * <Property name="StepMethod">WorldStep</Property>
     * <Property name="WorldCFM" type="float">0.000001</Property>
     * <Property name="WorldERP" type="float">0.2</Property>
     * <Property name="MaxIterations" type="int">100</Property>
     * <Property name="ContactSurfaceLayer" type="float">0.001</Property>
     *
     * <Property name="MaxSepDistance" type="float">0.01</Property>
     * <Property name="MaxCorrectingVelocity" type="float">0.1</Property>
	 * \endverbatim
	 *
	 */
	class ODESimulator : public PhysicsEngine
	{
	public:
		/**
		 * type of step method, worldStep is direct solver, worldquickstep is faster iterative solver.
		 * When dynamically simulating robot arms use worldstep, else simulate chains as kinematic bodies.
		 */
		typedef enum{WorldStep, WorldQuickStep, WorldFast1} StepMethod;
		//! type of broad phase collision detection to use
		typedef enum{Simple, HashTable, QuadTree} SpaceType;
		//! smart pointer type
		typedef rw::common::Ptr<ODESimulator> Ptr;

		//! empty constructor
        ODESimulator();

		/**
		 * @brief constructor
		 * @param dwc [in] the dynamic workcell for which the simulator should work
		 * @param detector [in] the contact detector to use
		 */
		ODESimulator(dynamics::DynamicWorkCell::Ptr dwc, rw::common::Ptr<rwsim::contacts::ContactDetector> detector = NULL);

		/**
		 * @brief destructor
		 */
		virtual ~ODESimulator(){
			delete _narrowStrategy;
		}

		//! @copydoc PhysicsEngine::load
		void load(rwsim::dynamics::DynamicWorkCell::Ptr dwc);

		//! @copydoc PhysicsEngine::setContactDetector
		bool setContactDetector(rw::common::Ptr<rwsim::contacts::ContactDetector> detector);

		/**
		 * @brief sets the ODE step method that should be used for stepping
		 */
		void setStepMethod(StepMethod method){ _stepMethod = method; };


		// inherited functions
		/**
		 * @copydoc step
		 */
		void step(double dt, rw::kinematics::State& state);

		/**
		 * @copydoc resetScene
		 */
		void resetScene(rw::kinematics::State& state);

		/**
		 * @copydoc initPhysics
		 */
		void initPhysics(rw::kinematics::State& state);

		/**
		 * @copydoc exitPhysics
		 */
		void exitPhysics();

		/**
		 * @copydoc getTime
		 */
		double getTime(){
			return _time;
		}

		void DWCChangedListener(dynamics::DynamicWorkCell::DWCEventType type, boost::any data);

		//! @copydoc Simulator::setEnabled
		void setEnabled(dynamics::Body::Ptr body, bool enabled);

		//! @copydoc Simulator::setEnabled
		void setDynamicsEnabled(dynamics::Body::Ptr body, bool enabled);

		//! @copydoc Simulator::createDebugRender
		drawable::SimulatorDebugRender::Ptr createDebugRender();

		//! @copydoc Simulator::getPropertyMap
		virtual rw::common::PropertyMap& getPropertyMap(){ return _propertyMap;};

		//! @copydoc Simulator::emitPropertyChanged
		void emitPropertyChanged();

		void addController(rwlibs::simulation::SimulatedController::Ptr controller);

		bool isInitialized(){ return _isSimulatorInitialized;};

		void addBody(rwsim::dynamics::Body::Ptr body, rw::kinematics::State& state);

		/**
		 * @brief Add a Constraint between two bodies.
		 * @param constraint [in] a pointer to the RobWork constraint.
		 */
		void addConstraint(rwsim::dynamics::Constraint::Ptr constraint);
		void addDevice(rwsim::dynamics::DynamicDevice::Ptr device, rw::kinematics::State& state);
		void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State& state);

		void removeController(rwlibs::simulation::SimulatedController::Ptr controller);
		void removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);

		const rw::kinematics::FramePairMap<std::vector<dynamics::ContactManifold> >&
		getContactManifoldMap(){
			return _manifolds;
		}

		std::vector<ODEBody*>& getODEBodies(){ return _odeBodies;}
		//std::vector<ODESensor>& getODEBodies(){ return _odeBodies;}

		dynamics::DynamicWorkCell::Ptr getDynamicWorkCell(){ return _dwc;};

		std::vector<rwlibs::simulation::SimulatedSensor::Ptr> getSensors(){
			return _sensors;
		}

		void attach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);
		void detach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

		void disableCollision(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

		void enableCollision(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

		//! get current gravity
		rw::math::Vector3D<> getGravity(){ return _dwc->getGravity(); }


        //std::map<rw::kinematics::Frame*, dBodyID> _rwFrameToODEBody;
        //std::map< dBodyID, rw::kinematics::Frame*> _rwODEBodyToFrame;
        //std::map<rw::kinematics::Frame*, ODEJoint*> _jointToODEJoint;
        //std::map<rw::kinematics::Frame*, dGeomID> _frameToOdeGeoms;

		dWorldID getODEWorldId() const { return _worldId; }

		void addODEJoint(ODEJoint* odejoint){
		    _jointToODEJoint[odejoint->getJoint()] = odejoint;
		}

		ODEJoint* getODEJoint(rw::models::Joint* joint){
            if( _jointToODEJoint.find(joint)== _jointToODEJoint.end()){
                return NULL;
            }
            return _jointToODEJoint[joint];
		}

        void addODEBody(ODEBody* odebody);
        void addODEBody(dBodyID body);
        void addODEJoint(dJointID joint);


		ODEBody* getODEBody(rw::kinematics::Frame* frame){
            if( _rwFrameToODEBody.find(frame)== _rwFrameToODEBody.end()){
                return NULL;
            }
            return _rwFrameToODEBody[frame];
		}

		dBodyID getODEBodyId(rw::kinematics::Frame* frame){
		    if( _rwFrameToODEBody.find(frame)== _rwFrameToODEBody.end()){
		        return NULL;
		    }
		    return _rwFrameToODEBody[frame]->getBodyID();
		}

		dBodyID getODEBodyId(rwsim::dynamics::Body* body){ return getODEBodyId(body->getBodyFrame()); }


        std::vector<ODEDevice*> getODEDevices() { return _odeDevices;}

        void addEmulatedContact(const rw::math::Vector3D<>& pos, const rw::math::Vector3D<>& force, const rw::math::Vector3D<>& normal, dynamics::Body* b);

	protected:
		//ODEBody* createKinematicBody(KinematicBody* kbody, rw::kinematics::State &state, dSpaceID spaceid);
        void detectCollisionsContactDetector(const rw::kinematics::State& state);
        bool detectCollisionsRW(rw::kinematics::State& state, bool onlyTestPenetration=false);

	public:
		/// Enables logging of the contact points
        void setContactLoggingEnabled(bool enable) { _logContactingBodies = enable; }
        
        /// Returns a map of contacting body frame names and their contact points
        boost::unordered_map<std::pair<std::string,std::string>, bool > getContactingBodies() {
			return _contactingBodies;
		}


		/**
		 * @brief Returns a vector of all contact points
		 * 
		 * @return vector of tuples (name1, name2, contact point)
		 */ /*
        std::vector<boost::tuple<std::string, std::string, dynamics::ContactPoint> > getContactPoints() const {
			return _contactPoints;
		} */

		void handleCollisionBetween(dGeomID o0, dGeomID o1);

		const std::vector<ODEUtil::TriGeomData*>& getTriMeshs(){
			return _triGeomDatas;
		}

		std::vector<dynamics::ContactPoint> getContacts(){
			std::vector<dynamics::ContactPoint> contacts;
		    {
		    	boost::mutex::scoped_lock lock(_contactMutex);
		    	contacts = _allcontactsTmp;
		    }

			return contacts;
		}

		int getContactCnt() {
			return _nrOfCon;
		}

		struct ODEStateStuff{
			ODEStateStuff():body(NULL){}
			dBodyID body;
			dReal pos[4];
			dReal rot[4];
			dReal lvel[4];
			dReal avel[4];
			dReal force[4];
			dReal torque[4];

			ODEJoint *joint;
			dReal desvel; //desired vel
			dReal fmax;
		};

	public:
		/*
		ODEBody* createBody(dynamics::Body* bframe, const rw::kinematics::State& state, dSpaceID spaceid);


		// create bodies that match rw bodies
		ODEBody* createRigidBody(dynamics::Body* bframe,
                                const rw::kinematics::State& state,
                                dSpaceID spaceid);

        ODEBody* createKinematicBody(
                                dynamics::Body* kbody,
                                const rw::kinematics::State& state,
                                dSpaceID spaceid);

        ODEBody* createFixedBody(dynamics::Body* bframe,
                                const rw::kinematics::State& state,
                                dSpaceID spaceid);

		*/

        //


        double getMaxSeperatingDistance();


        dSpaceID getODESpace(){ return _spaceId; };

        void addContacts(std::vector<dContact>& contacts, size_t nr_con, ODEBody* dataB1, ODEBody* dataB2);

        std::vector<ODETactileSensor*> getODESensors(dBodyID odebody){ return _odeBodyToSensor[odebody]; }

        dynamics::MaterialDataMap& getMaterialMap(){ return _materialMap; }

        dynamics::ContactDataMap& getContactMap(){ return _contactMap; }

	private:
		void saveODEState();
		void restoreODEState();
		void readProperties();
		// return the contact normal
		//rw::math::Vector3D<> addContacts(int numc, dBodyID b1, dBodyID b2, dGeomID o1, dGeomID o2, rw::kinematics::Frame *f1, rw::kinematics::Frame *f2);
        rw::math::Vector3D<> addContacts(int numc, ODEBody* b1, ODEBody* b2, rw::kinematics::Frame *f1, rw::kinematics::Frame *f2);

	private:

		dynamics::DynamicWorkCell::Ptr _dwc;
		double _time;
        rw::common::Ptr<ODEDebugRender> _render;
        std::vector<dContact> _contacts;
        std::vector<dContact> _filteredContacts;
        std::vector<dynamics::ContactPoint> _rwcontacts;
        std::vector<dynamics::ContactPoint> _rwClusteredContacts;
        std::vector<int> _srcIdx, _dstIdx;
        int _nrOfCon;
        rw::kinematics::FrameMap<int> _enabledMap;
        dynamics::MaterialDataMap _materialMap;
        dynamics::ContactDataMap _contactMap;
        rwlibs::proximitystrategies::ProximityStrategyPQP *_narrowStrategy;
        std::vector<dJointFeedback> _sensorFeedbacks;
        std::vector<dJointFeedback> _sensorFeedbacksGlobal;

        int _nextFeedbackIdx, _nextFeedbackGlobalIdx;
        rw::kinematics::FramePairMap<int> _excludeMap;

        // ODE specific variables
        dWorldID _worldId;
        dSpaceID _spaceId;
        dJointGroupID _contactGroupId;
        //std::vector<dBodyID> _bodies;
        std::vector<dBodyID> _allbodies;
        std::vector<dJointID> _alljoints;

        // RWODE bodies
        std::vector<ODEBody*> _odeBodies;

        // RW rigid bodies
        std::vector<dynamics::RigidBody*> _rwBodies;
        //FrameMap<dBodyID> _rwFrameToODEBody;

        std::map<const rw::kinematics::Frame*, ODEBody*> _rwFrameToODEBody;
        std::map< ODEBody*, rw::kinematics::Frame*> _rwODEBodyToFrame;
        std::map<rw::kinematics::Frame*, ODEJoint*> _jointToODEJoint;
        std::map<rw::kinematics::Frame*, dGeomID> _frameToOdeGeoms;

        std::vector<ODEJoint*> _allODEJoints;
        double _maxPenetration;

        std::map<rwsim::dynamics::Constraint::Ptr, ODEConstraint*> _constraintToODEConstraint;
        std::vector<ODEConstraint*> _odeConstraints;

        std::map< dBodyID, std::vector<ODETactileSensor*> > _odeBodyToSensor;
        std::map<rwlibs::simulation::SimulatedSensor::Ptr, ODETactileSensor*> _rwsensorToOdesensor;
        std::vector<ODETactileSensor*> _odeSensors;

        std::vector<ODEDevice*> _odeDevices;

        StepMethod _stepMethod;
        rw::kinematics::State *_stepState;

		std::vector<dynamics::ContactPoint> _allcontacts,_allcontactsTmp;
		//std::vector<std::vector<ContactPoint> > _rwClusteredContacts;

		rw::kinematics::FramePairMap<std::vector<dynamics::ContactManifold> > _manifolds;

		std::vector<ODEStateStuff> _odeStateStuff;

		std::vector<ODEUtil::TriGeomData*> _triGeomDatas;

		rw::common::PropertyMap _propertyMap;

		// ENGINE specific properties
		int _maxIter;
		ODEMaterialMap *_odeMaterialMap;
		SpaceType _spaceType;

		double _worldCFM, _worldERP, _oldTime;
		std::string _clusteringAlgStr;

		std::string _collidingObjectsMsg;

		double _contactSurfaceLayer;
		double _maxSepDistance;
		double _maxAllowedPenetration;
		double _contactMaxCorrectingVel;

		bool _useRobWorkContactGeneration;
		bool _prevStepEndedInCollision;
		std::vector<rwlibs::simulation::SimulatedController::Ptr> _controllers;
		std::vector<rwlibs::simulation::SimulatedSensor::Ptr> _sensors;

		bool _isSimulatorInitialized;

		rw::proximity::BasicFilterStrategy::Ptr _bpstrategy;
		rw::kinematics::FrameMap<rw::proximity::ProximityModel::Ptr> _frameToModels;
		boost::mutex _contactMutex;

		rw::common::Ptr<rwsim::contacts::ContactDetector> _detector;

        bool _logContactingBodies;
        std::vector<boost::tuple<std::string, std::string, dynamics::ContactPoint> > _contactPoints, _contactPointsTmp;
        boost::unordered_map<std::pair<std::string,std::string>, bool> _contactingBodies, _contactingBodiesTmp;

		struct CutState {


		};


		struct BodyBodyContact {
		    BodyBodyContact():
		        firstContact(true)
		    {}
		    BodyBodyContact(rw::math::Transform3D<> tta, rw::math::Transform3D<> ttb):
		        aT(tta),bT(ttb),firstContact(true)
		    {}
            rw::math::Transform3D<> aT, bT;
            bool firstContact;
            // defined in a
            rw::math::Vector3D<> cnormal;
		};

		std::map< std::pair<rw::kinematics::Frame*, rw::kinematics::Frame*>, BodyBodyContact > _lastNonCollidingTransform;

		std::map< std::pair<rw::kinematics::Frame*, rw::kinematics::Frame*>, dJointID > _attachConstraints;


	};

}
}

#endif /*ODESIMULATOR_HPP_*/
