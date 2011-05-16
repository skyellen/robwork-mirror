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

#include <ode/ode.h>

#include <rw/sensor/Sensor.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/kinematics/FramePairMap.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <rwsim/sensor/SimulatedTactileSensor.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/MaterialDataMap.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactManifold.hpp>

#include <rwsim/simulator/PhysicsEngineFactory.hpp>


#include "ODEUtil.hpp"
#include "ODEMaterialMap.hpp"
#include "ODEJoint.hpp"
#include "ODEDevice.hpp"
#include "ODEBody.hpp"
#include "ODETactileSensor.hpp"


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
	 */
	class ODESimulator : public PhysicsEngine
	{
	public:
		typedef enum{WorldStep, WorldQuickStep, WorldFast1} StepMethod;
		typedef enum{Simple, HashTable, QuadTree} SpaceType;
		/**
		 * @brief constructor
		 * @param dwc [in] the dynamic workcell for which the simulator should work
		 */
		ODESimulator(dynamics::DynamicWorkCell::Ptr dwc);

		/**
		 * @brief destructor
		 */
		virtual ~ODESimulator(){}

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

		/**
		 * @copydoc Simulator::setEnabled
		 */
		void setEnabled(dynamics::Body* body, bool enabled);

		/**
		 * @copydoc Simulator::createDebugRender
		 */
		drawable::SimulatorDebugRender* createDebugRender();

		/**
		 * @copydoc Simulator::getPropertyMap
		 */
		virtual rw::common::PropertyMap& getPropertyMap(){ return _propertyMap;};

		/**
		 * @copydoc Simulator::emitPropertyChanged
		 */
		void emitPropertyChanged();

		void addController(rwlibs::simulation::SimulatedController::Ptr controller){
			_controllers.push_back(controller);
		}

		bool isInitialized(){ return _isSimulatorInitialized;};

		void addBody(rwsim::dynamics::Body::Ptr body, rw::kinematics::State& state);
		void addDevice(rwsim::dynamics::DynamicDevice::Ptr device, rw::kinematics::State& state);
		void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State& state);

		void removeController(rwlibs::simulation::SimulatedController::Ptr controller){}

		void removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor){};

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

		bool detectCollisionsRW(rw::kinematics::State& state, bool onlyTestPenetration=false);

		rw::math::Vector3D<> getGravity(){ return _dwc->getGravity(); }


        //std::map<rw::kinematics::Frame*, dBodyID> _rwFrameToODEBody;
        //std::map< dBodyID, rw::kinematics::Frame*> _rwODEBodyToFrame;
        //std::map<rw::kinematics::Frame*, ODEJoint*> _jointToODEJoint;
        //std::map<rw::kinematics::Frame*, dGeomID> _frameToOdeGeoms;

		dWorldID getODEWorldId(){ return _worldId; }

		ODEBody* getODEBody(rw::kinematics::Frame* frame){
            if( _rwFrameToODEBody.find(frame)== _rwFrameToODEBody.end()){
                return 0;
            }
            return _rwFrameToODEBody[frame];
		}

		dBodyID getODEBodyId(rw::kinematics::Frame* frame){
		    if( _rwFrameToODEBody.find(frame)== _rwFrameToODEBody.end()){
		        return 0;
		    }
		    return _rwFrameToODEBody[frame]->getBodyID();
		}

		dBodyID getODEBodyId(rwsim::dynamics::Body* body){ return getODEBodyId(body->getBodyFrame()); }

	public:
		//ODEBody* createKinematicBody(KinematicBody* kbody, rw::kinematics::State &state, dSpaceID spaceid);

	public:

		struct TriMeshData {
		public:
	        typedef rw::common::Ptr<TriMeshData> Ptr;

		    TriMeshData(int sizeI,int sizeV):
				indices(sizeI*2,0),
				vertices(sizeV*2,0)
			{
				indices.resize(sizeI);
				vertices.resize(sizeV);
			}

			std::vector<dTriIndex> indices;
			std::vector<float> vertices;
			dTriMeshDataID triMeshID;
		};

		struct TriGeomData {
		public:
			TriGeomData(TriMeshData::Ptr triData):
				tridata(triData), mBuffIdx(0)
			{
				for (int j=0; j<16; j++){
					mBuff[0][j] = 0.0f;
					mBuff[1][j] = 0.0f;
				}
			}

			TriMeshData::Ptr tridata;
			dMatrix4 mBuff[2];
			dQuaternion rot;
			rw::math::Transform3D<> t3d;
			dVector3 p;
			dGeomID geomId;
			int mBuffIdx;
			bool isGeomTriMesh;
		};

		void handleCollisionBetween(dGeomID o0, dGeomID o1);

		const std::vector<TriGeomData*>& getTriMeshs(){
			return _triGeomDatas;
		}

		std::vector<dynamics::ContactPoint> getContacts(){
			return _allcontactsTmp;
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
		ODEBody* createBody(dynamics::Body* bframe, const rw::kinematics::State& state, dSpaceID spaceid);

		// create bodies that match rw bodies
		ODEBody* createRigidBody(dynamics::Body* bframe,
                                const rw::kinematics::State& state,
                                dSpaceID spaceid);

        ODEBody* createKinematicBody(
                                dynamics::KinematicBody* kbody,
                                const rw::kinematics::State& state,
                                dSpaceID spaceid);

        ODEBody* createFixedBody(dynamics::Body* bframe,
                                const rw::kinematics::State& state,
                                dSpaceID spaceid);

        //
        void addODEBody(dBodyID body){_allbodies.push_back(body);};
        void addODEJoint(dJointID joint){_alljoints.push_back(joint);};


        double getMaxSeperatingDistance();


        dSpaceID getODESpace(){ return _spaceId; };

        void addContacts(std::vector<dContact>& contacts, size_t nr_con, ODEBody* dataB1, ODEBody* dataB2);
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
        ODEDebugRender *_render;
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
        int _nextFeedbackIdx;
        rw::kinematics::FramePairMap<int> _excludeMap;

        // ODE specific variables
        dWorldID _worldId;
        dSpaceID _spaceId;
        dJointGroupID _contactGroupId;
        std::vector<dBodyID> _bodies;
        std::vector<dBodyID> _allbodies;
        std::vector<dJointID> _alljoints;

        // RWODE bodies
        std::vector<ODEBody*> _odeBodies;

        // RW rigid bodies
        std::vector<dynamics::RigidBody*> _rwBodies;
        //FrameMap<dBodyID> _rwFrameToODEBody;

        std::map<rw::kinematics::Frame*, ODEBody*> _rwFrameToODEBody;
        std::map< ODEBody*, rw::kinematics::Frame*> _rwODEBodyToFrame;
        std::map<rw::kinematics::Frame*, ODEJoint*> _jointToODEJoint;
        std::map<rw::kinematics::Frame*, dGeomID> _frameToOdeGeoms;

        std::vector<ODEJoint*> _allODEJoints;
        double _maxPenetration;

        std::map< dBodyID, ODETactileSensor*> _odeBodyToSensor;
        std::vector<ODETactileSensor*> _odeSensors;

        std::vector<ODEDevice*> _odeDevices;

        StepMethod _stepMethod;
        rw::kinematics::State *_stepState;

		std::vector<dynamics::ContactPoint> _allcontacts,_allcontactsTmp;
		//std::vector<std::vector<ContactPoint> > _rwClusteredContacts;

		rw::kinematics::FramePairMap<std::vector<dynamics::ContactManifold> > _manifolds;

		std::vector<ODEStateStuff> _odeStateStuff;

		std::vector<TriGeomData*> _triGeomDatas;

		rw::common::PropertyMap _propertyMap;


		int _maxIter;

		ODEMaterialMap *_odeMaterialMap;

		SpaceType _spaceType;

		double _worldCFM, _worldERP, _oldTime;
		std::string _clusteringAlgStr;

		std::vector<rwlibs::simulation::SimulatedController::Ptr> _controllers;
		std::vector<rwlibs::simulation::SimulatedSensor::Ptr> _sensors;

		bool _isSimulatorInitialized;

		rw::proximity::BasicFilterStrategy::Ptr _bpstrategy;
		rw::kinematics::FrameMap<rw::proximity::ProximityModel::Ptr> _frameToModels;


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



	};

	static const bool ODERegistrered = rwsim::simulator::PhysicsEngineFactory::Register<ODESimulator>::_Register("ODE");

}
}

#endif /*ODESIMULATOR_HPP_*/
