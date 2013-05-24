%module rwsim

%{
#include <rwlibs/swig/ScriptTypes.hpp>
//#include <rwslibs/swig/ScriptTypes.hpp>
#include <rwsimlibs/swig/ScriptTypes.hpp>
#if defined (SWIGLUA)
    #include <rwsimlibs/swig/Lua.hpp>
#endif

#include <rw/common/Ptr.hpp>
using namespace rwlibs::swig;
using namespace rwsim::swig;

%}

%include <std_string.i>
%include <std_vector.i>
%include <shared_ptr.i>

%import <rwlibs/swig/rw.i>

%include <stl.i>

%template (BodyPtr) rw::common::Ptr<Body>;
%template (BodyPtrVector) std::vector<rw::common::Ptr<Body> >;
%template (DynamicDevicePtr) rw::common::Ptr<DynamicDevice>;
%template (DynamicDevicePtrVector) std::vector<rw::common::Ptr<DynamicDevice> >;
%template (RigidDevicePtr) rw::common::Ptr<RigidDevice>;
%template (RigidDevicePtrVector) std::vector<rw::common::Ptr<RigidDevice> >;


%template (RigidBodyPtr) rw::common::Ptr<RigidBody>;
%template (RigidBodyPtrVector) std::vector<rw::common::Ptr<RigidBody> >;
%template (KinematicBodyPtr) rw::common::Ptr<KinematicBody>;
%template (KinematicBodyPtrVector) std::vector<rw::common::Ptr<KinematicBody> >;
%template (FixedBodyPtr) rw::common::Ptr<FixedBody>;
%template (FixedBodyPtrVector) std::vector<rw::common::Ptr<FixedBody> >;

%template (ThreadSimulatorPtr) rw::common::Ptr<ThreadSimulator>;
%template (DynamicSimulatorPtr) rw::common::Ptr<DynamicSimulator>;

%template (SimulatedFTSensorPtr) rw::common::Ptr<SimulatedFTSensor>;
%template (SerialDeviceControllerPtr) rw::common::Ptr<SerialDeviceController>;


DynamicWorkCell* getDynamicWorkCell();
void setDynamicWorkCell(DynamicWorkCell* dwc);

rw::common::Ptr<ThreadSimulator> getSimulatorInstance(const std::string& id);
void addSimulatorInstance(rw::common::Ptr<ThreadSimulator> sim, const std::string& id);
rw::common::Ptr<ThreadSimulator> getSimulatorInstance(const std::string& id);
rw::common::Ptr<ThreadSimulator> getSimulatorInstance();
void removeSimulatorInstance(const std::string& id);
std::vector<std::string> getSimulatorInstances();


struct BodyInfo {
public:
    BodyInfo();

    std::string material;
    std::string objectType;
    double mass;
    Vector3D masscenter;
    InertiaMatrix inertia;
    std::string integratorType;
    //std::vector<Frame*> frames;

    void print() const;
    void print(std::ostream& ostr) const;
};


class Body
{
public:
    //typedef rw::common::Ptr<Body> Ptr;
    Frame* getBodyFrame() const;

    //const std::vector<rw::geometry::Geometry::Ptr>& getGeometry();
    const std::vector<Frame*>& getFrames();

    const BodyInfo& getInfo() const;
    BodyInfo& getInfo();

    const std::string& getName() const;
    const std::string& getMaterialID() const;
    const InertiaMatrix& getInertia() const;

    //typedef enum{MassChangedEvent} BodyEventType;

    //typedef boost::function<void(BodyEventType)> BodyChangedListener;
    //typedef rw::common::Event<BodyChangedListener, BodyEventType> BodyChangedEvent;
    //BodyChangedEvent& changedEvent();
    //BodyChangedEvent _bodyChangedEvent;

    void setMass(double m);
    void setMass(double m, const InertiaMatrix& inertia);
    void setMass(double m, const InertiaMatrix& inertia, const Vector3D& com);

    //! interface functions
    virtual Vector3D getPointVelW(const Vector3D& p, const State& state) const = 0;
    virtual void reset(State &state) = 0;
    virtual double calcEnergy(const State& state) = 0;
    virtual void setForce(const Vector3D& f, State& state) = 0;
    virtual Vector3D getForce(const State& state) const = 0;
    virtual void addForce(const Vector3D& force, State& state) = 0;
    virtual void setTorque(const Vector3D& t, State& state) = 0;
    virtual void addTorque(const Vector3D& t, State& state) = 0;
    virtual Vector3D getTorque(const State& state) const = 0;

    virtual Frame* getParentFrame(const State& state) const;
    virtual void setForceW(const Vector3D& f, State& state);
    virtual Vector3D getForceW(const State& state) const;
    virtual void addForceW(const Vector3D& force, State& state);
    void addForceToPos(const Vector3D& force,
                               const Vector3D& pos,
                               State& state);
    virtual void addForceWToPosW(const Vector3D& force,
                                 const Vector3D& pos,
                                 State& state);
    virtual void setTorqueW(const Vector3D& t, State& state);
    virtual void addTorqueW(const Vector3D& t, State& state);
    virtual Vector3D getTorqueW(State& state);
    virtual Transform3D getTransformW(const State& state);

    Transform3D pTbf(const State& state);

    Transform3D pTcom(const State& state);

    Transform3D wTbf(const State& state);
    // world
    Transform3D wTcom(const State& state);

    %extend {

        Transform3D place(rw::common::Ptr<CollisionDetector> coldect, const State& state, const Vector3D& dir){
            return rwsim::dynamics::BodyUtil::placeBody($self, coldect, state, dir);
        }

        Transform3D place(rw::common::Ptr<CollisionDetector> coldect, const State& state){
            return rwsim::dynamics::BodyUtil::placeBody($self, coldect, state, -Vector3D::z());
        }
        
    };
};


class RigidBody : public Body
{
public:
    RigidBody(
        const BodyInfo& info,
        MovableFrame* frame,
        rw::common::Ptr<Geometry> geom
        );

    RigidBody(
        const BodyInfo& info,
        MovableFrame* frame,
        const std::vector<rw::common::Ptr<Geometry> >& geoms
        );

    //InertiaMatrix getEffectiveMassW(const Vector3D& wPc);
    Frame* getParent(State& state) const;
    Transform3D getPTBody(const State& state) const;
    void setPTBody(const Transform3D& pTb, State& state);
    Transform3D getWTBody(const State& state) const;

    Transform3D getWTParent(const State& state) const;
    Vector3D getLinVel(const State& state) const;

    /**
     * @brief return the linear velocity described in world frame
     */
    Vector3D getLinVelW(const State& state) const;
    void setLinVel(const Vector3D &lvel, State& state);
    void setLinVelW(const Vector3D &lvel, State& state);
    Vector3D getAngVel(const State& state) const ;
    Vector3D getAngVelW(State& state);
    void setAngVel(const Vector3D &avel, State& state);
    void setAngVelW(const Vector3D &avel, State& state);
    Vector3D getPointVel(const Vector3D& p, const State& state);
    double getMass() const;
    const InertiaMatrix& getBodyInertia() const;
    const InertiaMatrix& getBodyInertiaInv() const;
    InertiaMatrix calcInertiaTensorInv(const State& state) const;
    //InertiaMatrix calcInertiaTensorInvW(const State& state) const;
    InertiaMatrix calcInertiaTensor(const State& state) const;
    MovableFrame* getMovableFrame();
    InertiaMatrix calcEffectiveMass(const Vector3D& wPc, const State& state) const;
    InertiaMatrix calcEffectiveMassW(const Vector3D& wPc, const State& state) const;
    //InertiaMatrix calcEffectiveInertia(const State& state) const;
    //InertiaMatrix calcEffectiveInertiaInv(const State& state) const;
};

%nodefaultctor PoseController;
class PoseController //: public SimulatedController
{
public:

    double getSampleTime();

    void setSampleTime(double stime);

    //void update(const rwlibs::simulation::Simulator::UpdateInfo& info, State& state);

    //void reset(const State& state);

    //Controller* getController(){ return this; };

    std::string getControllerName();

    //rw::common::Ptr<rw::models::Device> getControlledDevice(){ return _device; }

    void setEnabled(bool enabled);

    bool isEnabled();

    void setTarget(const Transform3D& target);

    void setTarget(const Transform3D& target, const VelocityScrew6D& vals);

};

%nodefaultctor SerialDeviceController;
class SerialDeviceController //: public rwlibs::simulation::SimulatedController 
{
public:

	/**
	 *
	 * @param name [in] controller name
	 * @param ddev [in]
	 */
	// SerialDeviceController(const std::string& name, dynamics::DynamicDevice::Ptr ddev);

	//! destructor
	virtual ~SerialDeviceController();

	//! @brief move robot in a linear Cartesian path
	bool moveLin(const Transform3D& target, float speed=100, float blend=0);

	//! @brief move robot from point to point
	bool movePTP(const Q& target, float speed=100, float blend=0);

	//! @brief move robot from point to point but using a pose as target (require invkin)
	virtual bool movePTP_T(const Transform3D& target, float speed=100, float blend=0);

	//! @brief move robot in a servoing fasion
	virtual bool moveVelQ(const Q& target_joint_velocity);

	virtual bool moveVelT(const VelocityScrew6D& vel);

	//! move robot with a hybrid position/force control

    %extend {

    	bool moveLinFC(const Transform3D& target,
    							  const Wrench6D& wtarget,
    							  Q selection,
    							  std::string refframe,
    							  Transform3D offset,
    							  double speed=100,
    							  double blend=0)
    	{
    		float arr[6];
    		for(int i=0;i<6;i++)
    			arr[i] = selection[i];
    		return $self->SerialDeviceController::moveLinFC(target,wtarget,arr,refframe,offset,speed,blend);
    	}

        
    };
	
	//! hard stop the robot,
	bool stop();

	//! pause the robot, should be able to continue trajectory
	bool pause();

	//! enable safe mode, so that robot stops when collisions are detected
	bool setSafeModeEnabled(bool enable);

	Q getQ();
	Q getQd();

	bool isMoving();

	// simulated controller stuff

	//void update(const rwlibs::simulation::Simulator::UpdateInfo& info, State& state);


    std::string getControllerName();

    void reset(const rw::kinematics::State& state);

    //rwlibs::control::Controller* getController();

    void setEnabled(bool enabled);

    bool isEnabled();

};




%nodefaultctor DynamicDevice;
class DynamicDevice {

public:
    virtual Q getQ(const State& state);

    virtual void setQ(const Q &q, State& state);

    rw::common::Ptr<Device> getKinematicModel();
    rw::common::Ptr<Body> getBase();

    virtual Q getJointVelocities(const State& state);
    virtual void setJointVelocities(const Q &vel, State& state);

    //deprecated
    virtual Q getVelocity(const State& state);
    virtual void setVelocity(const Q& vel, State& state);

    virtual std::vector<rw::common::Ptr<Body> > getLinks();

};


%nodefaultctor RigidDevice;
class RigidDevice : public DynamicDevice {
    public:
        void setMotorForceLimits(const Q& force);

        Q getMotorForceLimits();

        Q getJointVelocities(const State& state);
        double getJointVelocity(int i, const State& state);

        void setJointVelocities(const Q& q, State& state);
        void setJointVelocity(double vel, int i, State& state);

        typedef enum{Force, Velocity} MotorControlMode;

        //std::vector<MotorControlMode> getMotorModes(const State& state);
        MotorControlMode getMotorMode(int i, const State& state);

        Q getMotorTargets(const State& state);
        double getMotorTarget(int i, const State& state);

        void setMotorTargets(const Q& q, State& state);
        void setMotorForceTargets(const Q& force, State& state);
        void setMotorVelocityTargets(const Q& vel, State& state);

        void setMotorTarget(double q, int i, State& state);
        void setMotorForceTarget(double force, int i, State& state);
        void setMotorVelocityTarget(double vel, int i, State& state);

        rw::common::Ptr<JointDevice> getJointDevice();
        std::vector<rw::common::Ptr<Body> > getLinks();

        //virtual void registerStateData(StateStructure::Ptr statestructure);

    public: ///// DEPRECATED FUNCTIONS
        //Q getForceLimit() { return getMotorForceLimits(); }
        // void setVelocity(Q& vel, State& state){ setJointVelocities(vel, state);}
    };

%nodefaultctor SuctionCup;
class SuctionCup : public DynamicDevice {
public:

    rw::common::Ptr<Body> getBaseBody();

    rw::common::Ptr<Body> getEndBody();

    //void addToWorkCell(rwsim::dynamics::DynamicWorkCell::Ptr dwc);

    double getRadius();

    double getHeight();

    Q getSpringParamsOpen();

    Q getSpringParamsClosed();

    Q getJointVelocities(const State& state);

    void setJointVelocities(const Q &vel, State& state);

    void addForceTorque(const Q &forceTorque, State& state);

    Transform3D getOffset();

    std::vector<rw::common::Ptr<Body> > getLinks();

    bool isClosed(const State& state);
    void setClosed(bool closed, State& state);

    rw::common::Ptr<Body> getContactBody(const State& state);
    void setContactBody(rw::common::Ptr<Body> b, State& state);

    double getPressure(const State& state);
    void setPressure(double pressure, State& state);

};

class DynamicWorkCell
{
public:
    //const BodyList& getBodies();

    /**
     * @brief Constructor
     */
    
    DynamicWorkCell(rw::common::Ptr<WorkCell> workcell,
                    const std::vector<rw::common::Ptr<Body> >& bodies,
                    const std::vector<rw::common::Ptr<Body> >& allbodies,
                    const std::vector<rw::common::Ptr<DynamicDevice> >& devices,
                    const std::vector<rw::common::Ptr<SimulatedController> >& controllers);
	
    rw::common::Ptr<Body> findBody(const std::string& name) const;

    //template<class T> T* findBody(const std::string& name) const;
    //template<class T> std::vector<T*> findBodies() const;

    //const DeviceList& getDynamicDevices(){ return _devices; };
    //DynamicDevice* findDevice(const std::string& name);

    //const ControllerList& getControllers();
    //const SensorList& getSensors();
    //void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);
    //const std::vector<Constraint>& getConstraints();
    void addController(rw::common::Ptr<SimulatedController> manipulator);
    rw::common::Ptr<SimulatedController> findController(const std::string& name);
    rw::common::Ptr<DynamicDevice> findDevice(const std::string& name) const;
    rw::common::Ptr<SimulatedSensor> findSensor(const std::string& name);

    //ContactDataMap& getContactData();
    //MaterialDataMap& getMaterialData();

    void addBody(rw::common::Ptr<Body> body);
    rw::common::Ptr<Body> getBody(Frame *f);

    rw::common::Ptr<WorkCell> getWorkcell();

    double getCollisionMargin();
    void setCollisionMargin(double margin);

    //WorkCellDimension getWorldDimension();

    bool inDevice(rw::common::Ptr<Body> body);
    void setGravity(const Vector3D& grav);
    const Vector3D& getGravity();
    PropertyMap& getEngineSettings();


    /*typedef enum{GravityChangedEvent,
                ConstraintAddedEvent,
                BodyAddedEvent,
                DeviceAddedEvent,
                ControllerAddedEvent,
                SensorAddedEvent
    } DWCEventType;
    */

    //typedef boost::function<void(DWCEventType, boost::any)> DWCChangedListener;
    //typedef rw::common::Event<DWCChangedListener, DWCEventType, boost::any> DWCChangedEvent;
    //DWCChangedEvent& changedEvent() { return _changedEvent; }

    %extend {
        rw::common::Ptr<RigidBody> findRigidBody(const std::string& name)
        { return $self->DynamicWorkCell::findBody<RigidBody>(name); }
        rw::common::Ptr<KinematicBody> findKinematicBody(const std::string& name)
        { return $self->DynamicWorkCell::findBody<KinematicBody>(name); }
        rw::common::Ptr<FixedBody> findFixedBody(const std::string& name)
        { return $self->DynamicWorkCell::findBody<FixedBody>(name); }

        rw::common::Ptr<RigidDevice> findRigidDevice(const std::string& name)
        { return $self->DynamicWorkCell::findDevice<RigidDevice>(name); }
        rw::common::Ptr<SuctionCup> findSuctionCup(const std::string& name)
        { return $self->DynamicWorkCell::findDevice<SuctionCup>(name); }

        rw::common::Ptr<SimulatedFTSensor> findFTSensor(const std::string& name)
        { 
        	rw::common::Ptr<SimulatedFTSensor> sensor = $self->DynamicWorkCell::findSensor<SimulatedFTSensor>(name);
        	if(sensor==NULL)
        		RW_THROW("No such sensor!");
        	return sensor; 
        
        }

        rw::common::Ptr<SerialDeviceController> findSerialDeviceController(const std::string& name)
        { return $self->DynamicWorkCell::findController<SerialDeviceController>(name); }
        
        void setGravity(double x, double y, double z){
            $self->DynamicWorkCell::setGravity( rw::math::Vector3D<>(x,y,z) );
        }
        
		rw::common::Ptr<Body> getBody(const std::string& name) const{
			rw::common::Ptr<Body> body = $self->findBody(name);
			if(body==NULL)
				RW_THROW("Could not find body: \"" << name << "\"" );
			return body;
		}


    };

};


///////////////////////////// rwsim::simulator


class DynamicSimulator: public Simulator
{
public:
    DynamicSimulator(rw::common::Ptr<DynamicWorkCell> dworkcell, rw::common::Ptr<PhysicsEngine> pengine);

    DynamicSimulator(rw::common::Ptr<DynamicWorkCell> dworkcell);

    virtual ~DynamicSimulator();

    void exitPhysics();
	double getTime();
	void setEnabled(rw::common::Ptr<Body> body, bool enabled);

	//drawable::SimulatorDebugRender::Ptr createDebugRender();
	PropertyMap& getPropertyMap();
	
	void addController(rw::common::Ptr<SimulatedController> controller);
	void removeController(rw::common::Ptr<SimulatedController> controller);

	void addBody(rw::common::Ptr<Body> body, State &state);
	void addDevice(rw::common::Ptr<DynamicDevice> dev, State &state);
	void addSensor(rw::common::Ptr<SimulatedSensor> sensor, State &state);
	void removeSensor(rw::common::Ptr<SimulatedSensor> sensor);
	std::vector<rw::common::Ptr<SimulatedSensor> > getSensors();

	 // Simulator interface
     void step(double dt, State& state);
     void reset(State& state);
	 void init(State& state);
	 void setEnabled(Frame* f, bool enabled);
	 void setDynamicsEnabled(rw::common::Ptr<Body> body, bool enabled);
	 // interfaces for manipulating/controlling bodies
	 void setTarget(rw::common::Ptr<Body> body, const Transform3D& t3d, State& state);

	 //void setTarget(rw::common::Ptr<Body> body, rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr traj, State& state);

	 void disableBodyControl( rw::common::Ptr<Body> body );
	 void disableBodyControl( );

	 rw::common::Ptr<BodyController> getBodyController();

	 void attach(rw::common::Ptr<Body> b1, rw::common::Ptr<Body> b2);

	 void detach(rw::common::Ptr<Body> b1, rw::common::Ptr<Body> b2);

};



class ThreadSimulator {
	public:
		ThreadSimulator(rw::common::Ptr<DynamicSimulator> simulator, const State &state);
		virtual ~ThreadSimulator();
		//void setPeriodMs(long period);
		void setRealTimeScale(double scale);
		void setTimeStep(double dt);
		void start();
		void stop();
		void postStop();
		void step();
		State getState();
		void setState(const State& state);
		void reset(const State& state);
		bool isRunning();
		double getTime();
		rw::common::Ptr<DynamicSimulator> getSimulator();

		//! The callback type for a hook into the step call
		//typedef boost::function<void(ThreadSimulator* sim, State&)> StepCallback;
		// void setStepCallBack(StepCallback cb);

		bool isInError();
		void setInError(bool inError);
	};



////////////////// SENSORS 

class SimulatedFTSensor //: public SimulatedTactileSensor 
{
public:
    SimulatedFTSensor(const std::string& name,
                      rw::common::Ptr<Body> body,
                      rw::common::Ptr<Body> body1,
                      Frame* frame=NULL);

	virtual ~SimulatedFTSensor();

	void update(const rwlibs::simulation::Simulator::UpdateInfo& info, State& state);
	void reset(const State& state);

	void addForceW(const Vector3D& point,
				   const Vector3D& force,
				   const Vector3D& cnormal,
				   State& state,
				   rw::common::Ptr<Body> body = NULL);

	void addForce(const Vector3D& point,
				  const Vector3D& force,
				  const Vector3D& cnormal,
				  State& state,
				  rw::common::Ptr<Body> body=NULL);

    void addWrenchToCOM(
                  const Vector3D& force,
                  const Vector3D& torque,
                  State& state,
                  rw::common::Ptr<Body> body=NULL);

    void addWrenchWToCOM(
                  const Vector3D& force,
                  const Vector3D& torque,
                  State& state,
                  rw::common::Ptr<Body> body=NULL);

    Transform3D getTransform();

    Vector3D getForce();
    
	Vector3D getTorque();

	double getMaxTorque();

	double getMaxForce();

	Frame* getSensorFrame();

	void acquire();

	//rw::common::Ptr<FTSensor> getSensor();

	rw::common::Ptr<Body> getBody1();
	rw::common::Ptr<Body> getBody2();
};


