#ifndef ODESIMULATOR_HPP_
#define ODESIMULATOR_HPP_

#include <ode/ode.h>

#include <rw/sensor/Sensor.hpp>

#include <sensors/SimulatedTactileSensor.hpp>
#include <rw/math/MetricUtil.hpp>

#include <simulator/Simulator.hpp>
#include <dynamics/RigidBody.hpp>
#include <dynamics/DynamicWorkcell.hpp>
#include <rw/kinematics/FrameMap.hpp>

#include <dynamics/MaterialDataMap.hpp>
#include <dynamics/ContactPoint.hpp>
#include <dynamics/ContactManifold.hpp>
#include <sandbox/kinematics/FramePairMap.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include "ODEUtil.hpp"
#include "ODEMaterialMap.hpp"
#include "ODEJoint.hpp"
#include "ODEDevice.hpp"
#include "ODEBody.hpp"
#include "ODETactileSensor.hpp"


namespace drawable {
    class ODEDebugRender;
}

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
class ODESimulator : public Simulator
{
private:
    // ODE specific variables
	dWorldID _worldId;
	dSpaceID _spaceId;
	dJointGroupID _contactGroupId;
	std::vector<dBodyID> _bodies;
	std::vector<dBodyID> _allbodies;

	// RWODE bodies
    std::vector<ODEBody*> _odeBodies;

    // RW rigid bodies
	std::vector<dynamics::RigidBody*> _rwBodies;

	dynamics::DynamicWorkcell *_dwc;

	//FrameMap<dBodyID> _rwFrameToODEBody;

    std::map<rw::kinematics::Frame*, dBodyID> _rwFrameToODEBody;
    std::map< dBodyID, rw::kinematics::Frame*> _rwODEBodyToFrame;
    std::map<rw::kinematics::Frame*, ODEJoint*> _jointToODEJoint;
    std::vector<ODEJoint*> _allODEJoints;
    double _maxPenetration;

    std::map< dBodyID, ODETactileSensor*> _odeBodyToSensor;
    std::vector<ODETactileSensor*> _odeSensors;
	double _time;
public:

private:
    std::vector<ODEDevice*> _odeDevices;
public:
	typedef enum{WorldStep, WorldQuickStep, WorldFast1} StepMethod;
	typedef enum{Simple, HashTable, QuadTree} SpaceType;
	/**
	 * @brief constructor
	 * @param dwc [in] the dynamic workcell for which the simulator should work
	 */
	ODESimulator(dynamics::DynamicWorkcell *dwc);

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
	void setEnabled(dynamics::RigidBody* body, bool enabled);

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

	void addController(rwlibs::simulation::SimulatedControllerPtr controller){
		_controllers.push_back(controller);
	}

	bool isInitialized(){ return _isSimulatorInitialized;};

	void addSensor(rwlibs::simulation::SimulatedSensorPtr sensor);

    void removeController(rwlibs::simulation::SimulatedControllerPtr controller){}

    void removeSensor(rwlibs::simulation::SimulatedSensorPtr sensor){};

	const rw::kinematics::FramePairMap<std::vector<ContactManifold> >&
    getContactManifoldMap(){
	    return _manifolds;
	}

	std::vector<ODEBody*>& getODEBodies(){ return _odeBodies;}
	//std::vector<ODESensor>& getODEBodies(){ return _odeBodies;}

	dynamics::DynamicWorkcell* getDynamicWorkcell(){ return _dwc;};

    std::vector<rwlibs::simulation::SimulatedSensorPtr> getSensors(){
        return _sensors;
    }


public:

    struct TriMeshData {
    public:
        TriMeshData(int sizeI,int sizeV):
            indices(sizeI*2,0),
            vertices(sizeV*2,0)
        {
            indices.resize(sizeI);
            vertices.resize(sizeV);
        }

        std::vector<dTriIndex> indices;
        std::vector<dReal> vertices;
        dTriMeshDataID triMeshID;
    };
    typedef rw::common::Ptr<TriMeshData> TriMeshDataPtr;

    struct TriGeomData {
    public:
        TriGeomData(TriMeshDataPtr triData):
            tridata(triData), mBuffIdx(0)
        {
            for (int j=0; j<16; j++){
                mBuff[0][j] = 0.0f;
                mBuff[1][j] = 0.0f;
            }
        }

        TriMeshDataPtr tridata;
        dMatrix4 mBuff[2];
        dQuaternion rot;
        rw::math::Transform3D<> t3d;
        dVector3 p;
        dGeomID geomId;
        int mBuffIdx;
    };

    void handleCollisionBetween(dGeomID o0, dGeomID o1);

    const std::vector<TriGeomData*>& getTriMeshs(){
        return _triGeomDatas;
    }

    std::vector<ContactPoint> getContacts(){
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

private:
	StepMethod _stepMethod;

	void saveODEState();

	void restoreODEState();

    void readProperties();

    drawable::ODEDebugRender *_render;
	std::vector<dContact> _contacts;
    std::vector<dContact> _filteredContacts;

    std::vector<ContactPoint> _rwcontacts;
    std::vector<ContactPoint> _rwClusteredContacts;

    std::vector<ContactPoint> _allcontacts,_allcontactsTmp;
    //std::vector<std::vector<ContactPoint> > _rwClusteredContacts;
    std::vector<int> _srcIdx, _dstIdx;

    rw::kinematics::FramePairMap<std::vector<ContactManifold> > _manifolds;

    int _nrOfCon;

    rw::kinematics::FrameMap<int> _enabledMap;

    std::vector<ODEStateStuff> _odeStateStuff;

    std::vector<TriGeomData*> _triGeomDatas;

    rw::common::PropertyMap _propertyMap;

    dBodyID createRigidBody(dynamics::Body* bframe,
							const dynamics::BodyInfo& info,
                            const rw::kinematics::State& state,
                            dSpaceID spaceid);

    dBodyID createKinematicBody(dynamics::KinematicBody* kbody,
                            const dynamics::BodyInfo& info,
                            const rw::kinematics::State& state,
                            dSpaceID spaceid);

    dBodyID createFixedBody(dynamics::Body* bframe,
							const dynamics::BodyInfo& info,
                            const rw::kinematics::State& state,
                            dSpaceID spaceid);

    int _maxIter;

    dynamics::MaterialDataMap _materialMap;
    dynamics::ContactDataMap _contactMap;
    ODEMaterialMap *_odeMaterialMap;

    SpaceType _spaceType;

    rw::kinematics::FramePairMap<int> _excludeMap;

	double _worldCFM, _worldERP;
	std::string _clusteringAlgStr;

	std::vector<rwlibs::simulation::SimulatedControllerPtr> _controllers;
	std::vector<rwlibs::simulation::SimulatedSensorPtr> _sensors;



	std::vector<dJointFeedback> _sensorFeedbacks;
	int _nextFeedbackIdx;


	rwlibs::proximitystrategies::ProximityStrategyPQP *_narrowStrategy;

	bool _isSimulatorInitialized;
};

#endif /*ODESIMULATOR_HPP_*/
