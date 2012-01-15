%module rwsim

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rws/swig/ScriptTypes.hpp>

#include <rwsimlibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>
using namespace rwlibs::swig;
using namespace rwsim::swig;
%}

%import <rwlibs/swig/rw.i>


DynamicWorkCell* getDynamicWorkCell();
void setDynamicWorkCell(DynamicWorkCell* dwc);

struct BodyInfo {
public:
    BodyInfo();

    std::string material;
    std::string objectType;
    double mass;
    Vector3D masscenter;
    InertiaMatrix inertia;
    std::string integratorType;
    std::vector<Frame*> frames;

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
    rw::kinematics::Frame* getParent(State& state) const;
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

class DynamicWorkCell
{
public:
    //const BodyList& getBodies();

    /**
     * @brief Constructor
     */
    DynamicWorkCell(rw::common::Ptr<WorkCell> workcell,
                    const std::vector<Body*>& bodies,
                    const std::vector<Body*>& allbodies,
                    const std::vector<DynamicDevice*>& devices,
                    const std::vector<rw::common::Ptr<SimulatedController> >& controllers);

    Body* findBody(const std::string& name) const;

    //template<class T> T* findBody(const std::string& name) const;
    //template<class T> std::vector<T*> findBodies() const;

    //const DeviceList& getDynamicDevices(){ return _devices; };
    //DynamicDevice* findDevice(const std::string& name);

    //const ControllerList& getControllers();
    //const SensorList& getSensors();
    //void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);
    //const std::vector<Constraint>& getConstraints();
    //void addController(rwlibs::simulation::SimulatedController::Ptr manipulator)
    //rwlibs::simulation::SimulatedController::Ptr findController(const std::string& name);

    //ContactDataMap& getContactData();
    //MaterialDataMap& getMaterialData();

    void addBody(Body* body);
    Body* getBody(Frame *f);

    rw::common::Ptr<WorkCell> getWorkcell();

    double getCollisionMargin();
    void setCollisionMargin(double margin);

    //WorkCellDimension getWorldDimension();

    bool inDevice(Body* body);
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
        RigidBody* findRigidBody(const std::string& name)
        { return $self->DynamicWorkCell::findBody<RigidBody>(name); }
        KinematicBody* findKinematicBody(const std::string& name)
        { return $self->DynamicWorkCell::findBody<KinematicBody>(name); }
        FixedBody* findFixedBody(const std::string& name)
        { return $self->DynamicWorkCell::findBody<FixedBody>(name); }

    };

};



