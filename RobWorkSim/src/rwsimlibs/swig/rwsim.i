%module rwsim

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rwsimlibs/swig/ScriptTypes.hpp>
#if defined (SWIGLUA)
    #include <rwsimlibs/swig/lua/Lua.hpp>
#endif
#if defined (SWIGJAVA)
	#include <rwsimlibs/swig/java/ThreadSimulatorStepCallbackEnv.hpp>
    typedef rwsimlibs::swig::ThreadSimulatorStepCallbackEnv ThreadSimulatorStepCallbackEnv;
    typedef rwsimlibs::swig::ThreadSimulatorStepCallbackEnv::cThreadSimulatorStepCallback cThreadSimulatorStepCallback;
#endif

#include <rw/common/Ptr.hpp>
using rw::trajectory::Trajectory;
using namespace rwlibs::swig;
using namespace rwsim::swig;

#if defined(SWIGJAVA)

#include <assert.h>
struct callback_data {
  JNIEnv *env;
  jobject obj;
};

void java_ThreadSimulatorStepCallback(ThreadSimulator* sim, State &state, void *ptr) {
  struct callback_data *data = (struct callback_data*) ptr;
  JNIEnv* env = data->env;
  bool fromThread = false;
  
  JavaVM* jvm;
  env->GetJavaVM(&jvm);
  JNIEnv* newEnv;
  int getEnvStat = jvm->GetEnv((void **)&newEnv, JNI_VERSION_1_6);
  if (getEnvStat == JNI_EDETACHED) {
    fromThread = true;
    if (jvm->AttachCurrentThread((void **) &newEnv, NULL) != 0) {
      std::cout << "Failed to attach" << std::endl;
      return;
    } else {
      env = newEnv;
    }
  //} else if (getEnvStat == JNI_OK) {
  } else if (getEnvStat == JNI_EVERSION) {
    std::cout << "GetEnv: version not supported" << std::endl;
    return;
  }
  
  const jclass callbackInterfaceClass = env->FindClass("dk/robwork/ThreadSimulatorStepCallbackHandler");
  assert(callbackInterfaceClass);
  const jmethodID meth = env->GetMethodID(callbackInterfaceClass, "callback", "(Ldk/robwork/ThreadSimulator;Ldk/robwork/State;)V");
  assert(meth);
  
  jclass threadSimClass = env->FindClass("dk/robwork/ThreadSimulator");
  jmethodID threadSimConstructor = env->GetMethodID(threadSimClass, "<init>", "(JZ)V");
  jobject jsim = env->NewObject(threadSimClass, threadSimConstructor, (jlong) sim, (jboolean)false);
  
  jclass stateClass = env->FindClass("dk/robwork/State");
  jmethodID stateConstructor = env->GetMethodID(stateClass, "<init>", "(JZ)V");
  jobject jstate = env->NewObject(stateClass, stateConstructor, (jlong) &state, (jboolean)false);
  
  env->CallVoidMethod(data->obj, meth, jsim, jstate);
  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
  }
  
  if (fromThread) {
    jvm->DetachCurrentThread();
  }
}
#endif
%}

%include <std_string.i>
%include <std_vector.i>
%include <shared_ptr.i>

%import <rwlibs/swig/rw.i>

%pragma(java) jniclassclassmodifiers="class"

%include <stl.i>

/********************************************
 * General utility functions
 ********************************************/

rw::common::Ptr<DynamicWorkCell> getDynamicWorkCell();
void setDynamicWorkCell(rw::common::Ptr<DynamicWorkCell> dwc);

rw::common::Ptr<ThreadSimulator> getSimulatorInstance(const std::string& id);
void addSimulatorInstance(rw::common::Ptr<ThreadSimulator> sim, const std::string& id);
rw::common::Ptr<ThreadSimulator> getSimulatorInstance(const std::string& id);
rw::common::Ptr<ThreadSimulator> getSimulatorInstance();
void removeSimulatorInstance(const std::string& id);
std::vector<std::string> getSimulatorInstances();

/********************************************
 * Boost
 ********************************************/

#if defined(SWIGJAVA)
%typemap(jstype) cThreadSimulatorStepCallback fct "ThreadSimulatorStepCallbackHandler";
%typemap(jtype) cThreadSimulatorStepCallback fct "ThreadSimulatorStepCallbackHandler";
%typemap(jni) cThreadSimulatorStepCallback fct "jobject";
%typemap(javain) cThreadSimulatorStepCallback fct "$javainput";

%typemap(in,numinputs=1) (cThreadSimulatorStepCallback fct, void *userdata) {
  struct callback_data *data = (struct callback_data*) malloc(sizeof *data);
  data->env = jenv;
  data->obj = JCALL1(NewGlobalRef, jenv, $input);
  JCALL1(DeleteLocalRef, jenv, $input);
  $1 = java_ThreadSimulatorStepCallback;
  $2 = data;
}
#endif

%nodefaultctor ThreadSimulatorStepCallback;
class ThreadSimulatorStepCallback {
public:
	// Make pass by value possible in setStepCallback
	ThreadSimulatorStepCallback(const ThreadSimulatorStepCallback &cb);
};

#if defined(SWIGJAVA)
%nodefaultctor ThreadSimulatorStepCallbackEnv;
class ThreadSimulatorStepCallbackEnv/*: public ThreadSimulatorStepCallback*/ {
public:
	ThreadSimulatorStepCallbackEnv(const ThreadSimulatorStepCallbackEnv &cb);
	ThreadSimulatorStepCallbackEnv(cThreadSimulatorStepCallback fct, void *userdata);
	void set(cThreadSimulatorStepCallback fct, void *userdata);
	%rename(dispatch) operator();
	void operator()(ThreadSimulator* sim, State& state);
};
#endif

/********************************************
 * CONTACTS
 ********************************************/

%nodefaultctor ContactDetector;
class ContactDetector {
public:
	/*struct StrategyTableRow {
		std::size_t priority;
		ProximitySetup rules;
		ContactStrategy::Ptr strategy;
		//FrameMap<std::map<std::string, rw::common::Ptr<ContactModel> > > models;
	};*/
	//typedef std::list<StrategyTableRow> StrategyTable;

	ContactDetector(rw::common::Ptr<WorkCell> workcell);
	//ContactDetector(rw::common::Ptr<WorkCell> workcell, rw::common::Ptr<ProximityFilterStrategy> filter);
	virtual ~ContactDetector();
	//void setProximityFilterStrategy(rw::proximity::ProximityFilterStrategy::Ptr filter);
	virtual std::vector<Contact> findContacts(const State& state);
	virtual std::vector<Contact> findContacts(const State& state, ContactDetectorData &data);
	virtual std::vector<Contact> findContacts(const State& state, ContactDetectorData &data, ContactDetectorTracking& tracking);
	virtual std::vector<Contact> updateContacts(const State& state, ContactDetectorData &data, ContactDetectorTracking& tracking);
	//virtual rw::common::Ptr<ProximityFilterStrategy> getProximityFilterStrategy() const;
	virtual double getTimer() const;
	virtual void setTimer(double value = 0);
	//virtual StrategyTable getContactStategies() const;
	//virtual StrategyTable getContactStrategies(const std::string& frameA, rw::common::Ptr<const rw::geometry::Geometry> geometryA, const std::string& frameB, rw::common::Ptr<const rw::geometry::Geometry> geometryB) const;
	virtual void addContactStrategy(rw::common::Ptr<ContactStrategy> strategy, std::size_t priority = 0);
	//virtual void addContactStrategy(ProximitySetupRule rule, rw::common::Ptr<ContactStrategy> strategy, std::size_t priority = 0);
	//virtual void addContactStrategy(ProximitySetup rules, rw::common::Ptr<ContactStrategy> strategy, std::size_t priority = 0);
	//virtual void addContactStrategy(StrategyTableRow &strategy, std::size_t priority = 0);
	virtual void removeContactStrategy(std::size_t priority = 0);
	virtual void clearStrategies();
	//virtual void setContactStrategies(StrategyTable strategies);
	virtual void setDefaultStrategies();
	virtual void setDefaultStrategies(const PropertyMap& map);
	virtual void printStrategyTable() const;
};

%template (ContactDetectorPtr) rw::common::Ptr<ContactDetector>;

class ContactDetectorData {
public:
	ContactDetectorData();
	ContactDetectorData(const ContactDetectorData& data);
	virtual ~ContactDetectorData();
	//ContactDetectorData& operator=(const ContactDetectorData& data);
	void clear();
	//ContactStrategyData& getStrategyData(const ContactModel* modelA, const ContactModel* modelB);
};

%template (ContactDetectorDataPtr) rw::common::Ptr<ContactDetectorData>;

class ContactDetectorTracking {
public:
	ContactDetectorTracking();
	ContactDetectorTracking(const ContactDetectorTracking& data);
	virtual ~ContactDetectorTracking();
	//ContactDetectorTracking& operator=(const ContactDetectorTracking& data);
	void clear();
	void remove(std::size_t index);
	//ContactStrategyTracking::UserData::Ptr getUserData(std::size_t index) const;
	//std::vector<ContactStrategyTracking::UserData::Ptr> getUserData() const;
	//void setUserData(std::size_t index, ContactStrategyTracking::UserData::Ptr data);
	//void setUserData(const std::vector<ContactStrategyTracking::UserData::Ptr> &data);
	std::size_t getSize() const;

	/*struct ContactInfo {
		ContactInfo(): tracking(NULL), id(0), total(0) {}
		std::pair<rw::kinematics::Frame*, rw::kinematics::Frame*> frames;
		std::pair<ContactModel*,ContactModel*> models;
		rw::common::Ptr<ContactStrategy> strategy;
		ContactStrategyTracking* tracking;
		std::size_t id;
		std::size_t total;
	};
	std::vector<ContactInfo>& getInfo();
	const std::vector<ContactInfo>& getInfo() const;*/
	//ContactStrategyTracking& getStrategyTracking(const ContactModel* modelA, const ContactModel* modelB);
};

%template (ContactDetectorTrackingPtr) rw::common::Ptr<ContactDetectorTracking>;

class ContactStrategy //: public rw::proximity::ProximityStrategy
{
public:
	ContactStrategy();
	virtual ~ContactStrategy() {};
	virtual bool match(rw::common::Ptr<GeometryData> geoA, rw::common::Ptr<GeometryData> geoB) = 0;
	/*
	virtual std::vector<Contact> findContacts(rw::common::Ptr<ProximityModel> a,
			const Transform3D& wTa,
			rw::common::Ptr<ProximityModel> b,
			const Transform3D& wTb) = 0;
	virtual std::vector<Contact> findContacts(
			rw::common::Ptr<ProximityModel> a,
			const Transform3D& wTa,
			rw::common::Ptr<ProximityModel> b,
			const Transform3D& wTb,
			ContactStrategyData &data) = 0;
	virtual std::vector<Contact> findContacts(
			rw::common::Ptr<ProximityModel> a,
			const Transform3D& wTa,
			rw::common::Ptr<ProximityModel> b,
			const Transform3D& wTb,
			ContactStrategyData &data,
			ContactStrategyTracking& tracking) = 0;
	virtual std::vector<Contact> updateContacts(
			rw::common::Ptr<ProximityModel> a,
			const Transform3D& wTa,
			rw::common::Ptr<ProximityModel> b,
			const Transform3D& wTb,
			ContactStrategyData& data,
			ContactStrategyTracking& tracking) const = 0;
	*/
	virtual std::string getName() = 0;
	
	// ProximityStrategy interface
	//virtual ProximityModel::Ptr createModel() = 0;
	//virtual void destroyModel(ProximityModel* model) = 0;
	//virtual bool addGeometry(ProximityModel* model, const Geometry& geom) = 0;
	//virtual bool addGeometry(ProximityModel* model, Geometry::Ptr geom, bool forceCopy=false) = 0;
	//virtual bool removeGeometry(ProximityModel* model, const std::string& geomId) = 0;
	//virtual std::vector<std::string> getGeometryIDs(ProximityModel* model) = 0;
	virtual void clear() = 0;
	
	// ContactStrategy:
	virtual PropertyMap& getPropertyMap();
#if !defined(SWIGJAVA)
	virtual const PropertyMap& getPropertyMap() const;
#endif
	virtual void setPropertyMap(const PropertyMap& map);
};

%template (ContactStrategyPtr) rw::common::Ptr<ContactStrategy>;

class Contact {
public:
	Contact();
	virtual ~Contact();
//	ContactModel::Ptr getModelA() const;
//	ContactModel::Ptr getModelB() const;
	const Frame* getFrameA() const;
	const Frame* getFrameB() const;
	Transform3D aTb() const;
	Vector3D getPointA() const;
	Vector3D getPointB() const;
	Vector3D getNormal() const;
	double getDepth() const;
//	void setModelA(ContactModel::Ptr modelA);
//	void setModelB(ContactModel::Ptr modelB);
	void setFrameA(const Frame* frame);
	void setFrameB(const Frame* frame);
	void setTransform(Transform3D aTb);
	void setPointA(Vector3D pointA);
	void setPointB(Vector3D pointB);
	void setPoints(Vector3D pointA, Vector3D pointB);
	void setNormal(Vector3D normal);
	void setDepth();
	void setDepth(double depth);
	//tostring
};

%template (ContactVector) std::vector<Contact>;

/********************************************
 * CONTROL
 ********************************************/

%nodefaultctor SimulatedController;
class SimulatedController
{
};

%template (SimulatedControllerPtr) rw::common::Ptr<SimulatedController>;
%template (SimulatedControllerPtrVector) std::vector<rw::common::Ptr<SimulatedController> >;
 
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

struct PDParam {
	PDParam();
	PDParam(double p, double d);
	double P;
	double D;
};

%template (PDParamVector) std::vector<PDParam>;

%nodefaultctor PDController;
class PDController
{
public:
	PDController(
	        const std::string& name,
	        rw::common::Ptr<DynamicDevice> rdev,
			JointController::ControlMode cmode,
			const std::vector<PDParam>& pdparams,
			double dt
			);

	PDController(
	        const std::string& name,
	        rw::common::Ptr<DynamicDevice> rdev,
			JointController::ControlMode cmode,
			const PDParam& pdparam,
			double dt
			);

	virtual ~PDController();

	std::vector<PDParam> getParameters();
	void setParameters(const std::vector<PDParam>& params);
	double getSampleTime();
	void setSampleTime(double stime);

	// From SimulatedController
	void update(const Simulator::UpdateInfo& info, State& state);
	void reset(const State& state);
	Controller* getController();
	std::string getControllerName();
    void setEnabled(bool enabled);
    bool isEnabled();

	// From JointController
	unsigned int getControlModes();
	void setControlMode(JointController::ControlMode mode);
	void setTargetPos(const Q& target);
	void setTargetVel(const Q& vals);
	void setTargetAcc(const Q& vals);
	Q getQ();
	Q getQd();
};

%template (PDControllerPtr) rw::common::Ptr<PDController>;

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
    							  Rotation3D offset,
    							  float speed=100,
    							  float blend=0)
    	{
    		float arr[6];
    		for(int i=0;i<6;i++)
    			arr[i] = (float)selection[i];
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

    void reset(const State& state);

    //rwlibs::control::Controller* getController();

    void setEnabled(bool enabled);

    bool isEnabled();

};

%template (SerialDeviceControllerPtr) rw::common::Ptr<SerialDeviceController>;

%nodefaultctor BodyController;
class BodyController
{
};

%template (BodyControllerPtr) rw::common::Ptr<BodyController>;

/********************************************
 * DRAWABLE
 ********************************************/

/********************************************
 * DYNAMICS
 ********************************************/

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

#if !defined(SWIGJAVA)
    const BodyInfo& getInfo() const;
#endif
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

%template (BodyPtr) rw::common::Ptr<Body>;
%template (BodyPtrVector) std::vector<rw::common::Ptr<Body> >;

class FixedBody: public Body
{
};

%template (FixedBodyPtr) rw::common::Ptr<FixedBody>;
%template (FixedBodyPtrVector) std::vector<rw::common::Ptr<FixedBody> >;

class KinematicBody: public Body
{
};

%template (KinematicBodyPtr) rw::common::Ptr<KinematicBody>;
%template (KinematicBodyPtrVector) std::vector<rw::common::Ptr<KinematicBody> >;

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

%template (RigidBodyPtr) rw::common::Ptr<RigidBody>;
%template (RigidBodyPtrVector) std::vector<rw::common::Ptr<RigidBody> >;

struct SpringParams {
public:
	SpringParams();
	bool enabled;
	Matrix compliance;
	Matrix damping;
};

%nestedworkaround Constraint::SpringParams;

%nodefaultctor Constraint;
class Constraint {
public:
	typedef enum {
		Fixed,
		Prismatic,
		Revolute,
		Universal,
		Spherical,
		Piston,
		PrismaticRotoid,
		PrismaticUniversal,
		Free
	} ConstraintType;

	Constraint(const std::string& name, const ConstraintType &type, Body* b1, Body* b2);
	virtual ~Constraint();
	ConstraintType getType() const;
	Body* getBody1() const;
	Body* getBody2() const;
	size_t getDOF() const;
	size_t getDOFLinear() const;
	size_t getDOFAngular() const;
	Transform3D getTransform() const;
	void setTransform(const Transform3D &parentTconstraint);
	SpringParams getSpringParams() const;
	void setSpringParams(const SpringParams &params);
	//static bool toConstraintType(const std::string &string, Constraint::ConstraintType &type);
};

%template (ConstraintPtr) rw::common::Ptr<Constraint>;
%template (ConstraintPtrVector) std::vector<rw::common::Ptr<Constraint> >;

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

%template (DynamicDevicePtr) rw::common::Ptr<DynamicDevice>;
%template (DynamicDevicePtrVector) std::vector<rw::common::Ptr<DynamicDevice> >;

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

%template (RigidDevicePtr) rw::common::Ptr<RigidDevice>;
%template (RigidDevicePtrVector) std::vector<rw::common::Ptr<RigidDevice> >;

%nodefaultctor SuctionCup;
class SuctionCup : public DynamicDevice {
public:

    rw::common::Ptr<Body> getBaseBody();

    rw::common::Ptr<Body> getEndBody();

    //void addToWorkCell(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc);

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

%template (SuctionCupPtr) rw::common::Ptr<SuctionCup>;

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
                    const std::vector<rw::common::Ptr<Constraint> >& constraints,
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

    const std::vector<rw::common::Ptr<Body> >& getBodies();
    void addBody(rw::common::Ptr<Body> body);
    rw::common::Ptr<Body> getBody(Frame *f);
    
    void addConstraint(rw::common::Ptr<Constraint> constraint);
    const std::vector<rw::common::Ptr<Constraint> >& getConstraints() const;
	rw::common::Ptr<Constraint> findConstraint(const std::string& name) const;

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
        
        rw::common::Ptr<PDController> findPDController(const std::string& name)
        { return $self->DynamicWorkCell::findController<PDController>(name); }
        
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

%template (DynamicWorkCellPtr) rw::common::Ptr<DynamicWorkCell>;

/********************************************
 * LOADERS
 ********************************************/

class DynamicWorkCellLoader
{
public:
    static rw::common::Ptr<DynamicWorkCell> load(const std::string& filename);
};

/********************************************
 * RWPHYSICS
 ********************************************/

/********************************************
 * SENSOR
 ********************************************/

%nodefaultctor SimulatedSensor;
class SimulatedSensor
{
};

%template (SimulatedSensorPtr) rw::common::Ptr<SimulatedSensor>;
%template (SimulatedSensorPtrVector) std::vector<rw::common::Ptr<SimulatedSensor> >;

class SimulatedFTSensor //: public SimulatedTactileSensor 
{
public:
    SimulatedFTSensor(const std::string& name,
                      rw::common::Ptr<Body> body,
                      rw::common::Ptr<Body> body1,
                      Frame* frame=NULL);

	virtual ~SimulatedFTSensor();

	void update(const Simulator::UpdateInfo& info, State& state);
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

%template (SimulatedFTSensorPtr) rw::common::Ptr<SimulatedFTSensor>;

/********************************************
 * SIMULATOR
 ********************************************/

%nodefaultctor PhysicsEngine;
class PhysicsEngine
{
public:
	virtual ~PhysicsEngine(){};
	virtual void load(rw::common::Ptr<DynamicWorkCell> dwc) = 0;
	virtual bool setContactDetector(rw::common::Ptr<ContactDetector> detector) = 0;
	virtual void step(double dt, State& state) = 0;
	virtual void resetScene(State& state) = 0;
	virtual void initPhysics(State& state) = 0;
	virtual void exitPhysics() = 0;
	virtual double getTime() = 0;
	virtual void setEnabled(rw::common::Ptr<Body> body, bool enabled) = 0;
	virtual void setDynamicsEnabled(rw::common::Ptr<Body> body, bool enabled) = 0;
	//virtual drawable::SimulatorDebugRender::Ptr createDebugRender() = 0;
	virtual PropertyMap& getPropertyMap() = 0;
	virtual void emitPropertyChanged() = 0;
	virtual void addController(rw::common::Ptr<SimulatedController> controller) = 0;
	virtual void removeController(rw::common::Ptr<SimulatedController> controller) = 0;
	virtual void addBody(rw::common::Ptr<Body> body, State &state) = 0;
	virtual void addDevice(rw::common::Ptr<DynamicDevice> dev, State &state) = 0;
	virtual void addSensor(rw::common::Ptr<SimulatedSensor> sensor, State &state) = 0;
	virtual void removeSensor(rw::common::Ptr<SimulatedSensor> sensor) = 0;
	virtual void attach(rw::common::Ptr<Body> b1, rw::common::Ptr<Body> b2) = 0;
	virtual void detach(rw::common::Ptr<Body> b1, rw::common::Ptr<Body> b2) = 0;
	virtual std::vector<rw::common::Ptr<SimulatedSensor> > getSensors() = 0;
};

%template (PhysicsEnginePtr) rw::common::Ptr<PhysicsEngine>;

%nodefaultctor PhysicsEngineFactory;
class PhysicsEngineFactory
{
public:
	static std::vector<std::string> getEngineIDs();
	static bool hasEngineID(const std::string& engineID);
	static rw::common::Ptr<PhysicsEngine> makePhysicsEngine(const std::string& engineID, rw::common::Ptr<DynamicWorkCell> dwc);
    static rw::common::Ptr<PhysicsEngine> makePhysicsEngine(rw::common::Ptr<DynamicWorkCell> dwc);
};

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
	 void setTarget(rw::common::Ptr<Body> body, rw::common::Ptr<Trajectory<Transform3D> > traj, State& state);

	 void disableBodyControl( rw::common::Ptr<Body> body );
	 void disableBodyControl( );

	 rw::common::Ptr<BodyController> getBodyController();

	 void attach(rw::common::Ptr<Body> b1, rw::common::Ptr<Body> b2);

	 void detach(rw::common::Ptr<Body> b1, rw::common::Ptr<Body> b2);

};

%template (DynamicSimulatorPtr) rw::common::Ptr<DynamicSimulator>;

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

#if defined(SWIGJAVA)
%extend {
		void setStepCallBack(ThreadSimulatorStepCallbackEnv cb) {
			(*$self).setStepCallBack((ThreadSimulatorStepCallback)cb);
		}
}
#endif
		void setStepCallBack(ThreadSimulatorStepCallback cb);
		
		bool isInError();
		void setInError(bool inError);
	};

%template (ThreadSimulatorPtr) rw::common::Ptr<ThreadSimulator>;
%template (ThreadSimulatorPtrVector) std::vector<rw::common::Ptr<ThreadSimulator> >;

%nodefaultctor GraspTaskSimulator;
class GraspTaskSimulator
{
public:
	GraspTaskSimulator(rw::common::Ptr<DynamicWorkCell> dwc, int nrThreads=1);
	virtual ~GraspTaskSimulator();
	void load(const std::string& filename);
	void load(rw::common::Ptr<GraspTask> graspTasks);
	rw::common::Ptr<GraspTask> getTasks();
	rw::common::Ptr<GraspTask> getResult();
	size_t getNrTargets();
	rw::common::Ptr<ThreadSimulator> getSimulator();
	std::vector<rw::common::Ptr<ThreadSimulator> > getSimulators();
	void init(rw::common::Ptr<DynamicWorkCell> dwc, const State& initState);
	void startSimulation(const State& initState);
	void pauseSimulation();
	void resumeSimulation();
	bool isRunning();
	bool isFinished();
	int getStat(GraspResult::TestStatus status);
	std::vector<int> getStat();
	std::string getStatDescription();
	int getNrTargetsDone();
	void setAlwaysResting(bool alwaysResting);
	void setStepDelay(int delay);
	void setWallTimeLimit(double limit);
	void setSimTimeLimit(double limit);
};

%template (GraspTaskSimulatorPtr) rw::common::Ptr<GraspTaskSimulator>;

%nodefaultctor AssemblySimulator;
class AssemblySimulator
{
public:
	AssemblySimulator(rw::common::Ptr<DynamicWorkCell> dwc, const std::string &engineID, rw::common::Ptr<ContactDetector> contactDetector = NULL);
	virtual ~AssemblySimulator();
	void start(rw::common::Ptr<ThreadTask> task = NULL);
	void stopFinishCurrent();
	void stopCancelCurrent();
	bool isRunning();
	void setTasks(std::vector<rw::common::Ptr<AssemblyTask> > tasks);
	std::vector<rw::common::Ptr<AssemblyResult> > getResults();
	void setStoreExecutionData(bool enable);
	bool storeExecutionData();
	double getMaxSimTime() const;
	void setMaxSimTime(double maxTime);
};

%template (AssemblySimulatorPtr) rw::common::Ptr<AssemblySimulator>;

/********************************************
 * UTIL
 ********************************************/

/********************************************
 * RWSIMLIBS BULLET
 ********************************************/

/********************************************
 * RWSIMLIBS GUI
 ********************************************/

/********************************************
 * RWSIMLIBS ODE
 ********************************************/

%nodefaultctor ODESimulator;
class ODESimulator: public PhysicsEngine
{
public:
		typedef enum{WorldStep, WorldQuickStep, WorldFast1} StepMethod;
		//typedef enum{Simple, HashTable, QuadTree} SpaceType;
		
		ODESimulator(rw::common::Ptr<DynamicWorkCell> dwc, rw::common::Ptr<ContactDetector> detector = NULL);
		virtual ~ODESimulator();
		
		// PhysicsEngine interface
		void load(rw::common::Ptr<DynamicWorkCell> dwc);
		bool setContactDetector(rw::common::Ptr<ContactDetector> detector);
		void step(double dt, State& state);
		void resetScene(State& state);
		void initPhysics(State& state);
		void exitPhysics();
		double getTime();
		void setEnabled(rw::common::Ptr<Body> body, bool enabled);
		void setDynamicsEnabled(rw::common::Ptr<Body> body, bool enabled);
		//drawable::SimulatorDebugRender::Ptr createDebugRender();
		virtual PropertyMap& getPropertyMap();
		void emitPropertyChanged();
		void addController(rw::common::Ptr<SimulatedController> controller);
		void removeController(rw::common::Ptr<SimulatedController> controller);
		void addBody(rw::common::Ptr<Body> body, State &state);
		void addDevice(rw::common::Ptr<DynamicDevice> dev, State &state);
		void addSensor(rw::common::Ptr<SimulatedSensor> sensor, State &state);
		void removeSensor(rw::common::Ptr<SimulatedSensor> sensor);
		void attach(rw::common::Ptr<Body> b1, rw::common::Ptr<Body> b2);
		void detach(rw::common::Ptr<Body> b1, rw::common::Ptr<Body> b2);
		std::vector<rw::common::Ptr<SimulatedSensor> > getSensors();
		
		// ODESimulator specific
		void setStepMethod(StepMethod method);
		//void DWCChangedListener(dynamics::DynamicWorkCell::DWCEventType type, boost::any data);
		bool isInitialized();
		//const rw::kinematics::FramePairMap<std::vector<dynamics::ContactManifold> >&getContactManifoldMap();
		//std::vector<ODEBody*>& getODEBodies(){ return _odeBodies;}
		rw::common::Ptr<DynamicWorkCell> getDynamicWorkCell();
		void disableCollision(rw::common::Ptr<Body> b1, rw::common::Ptr<Body> b2);
		void enableCollision(rw::common::Ptr<Body> b1, rw::common::Ptr<Body> b2);
		Vector3D getGravity();
		//dWorldID getODEWorldId();
		//void addODEJoint(ODEJoint* odejoint);
		//ODEJoint* getODEJoint(rw::models::Joint* joint);
        //void addODEBody(ODEBody* odebody);
        //void addODEBody(dBodyID body);
        //void addODEJoint(dJointID joint);
		//ODEBody* getODEBody(rw::kinematics::Frame* frame);
		//dBodyID getODEBodyId(rw::kinematics::Frame* frame);
		//dBodyID getODEBodyId(rwsim::dynamics::Body* body);
        //std::vector<ODEDevice*> getODEDevices() { return _odeDevices;}
        void addEmulatedContact(const Vector3D& pos, const Vector3D& force, const Vector3D& normal, Body* b);
        void setContactLoggingEnabled(bool enable);
        //std::map<std::pair<std::string,std::string>,std::vector<dynamics::ContactPoint> > getContactingBodies(){ return _contactingBodiesTmp; }
        //std::map<std::pair<std::string,std::string>,std::vector<dynamics::ContactPoint> > _contactingBodies, _contactingBodiesTmp;
		//void handleCollisionBetween(dGeomID o0, dGeomID o1);
		//const std::vector<ODEUtil::TriGeomData*>& getTriMeshs();
		//std::vector<dynamics::ContactPoint> getContacts();
		int getContactCnt();
		/*struct ODEStateStuff{
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
		};*/

        double getMaxSeperatingDistance();
        //dSpaceID getODESpace(){ return _spaceId; };
        //void addContacts(std::vector<dContact>& contacts, size_t nr_con, ODEBody* dataB1, ODEBody* dataB2);
        //std::vector<ODETactileSensor*> getODESensors(dBodyID odebody){ return _odeBodyToSensor[odebody]; }
        //dynamics::MaterialDataMap& getMaterialMap(){ return _materialMap; }
        //dynamics::ContactDataMap& getContactMap(){ return _contactMap; }
};

%template (ODESimulatorPtr) rw::common::Ptr<ODESimulator>;

/********************************************
 * RWSIMLIBS PLUGINS
 ********************************************/

/********************************************
 * RWSIMLIBS SWIG
 ********************************************/

/********************************************
 * RWSIMLIBS TOOLS
 ********************************************/