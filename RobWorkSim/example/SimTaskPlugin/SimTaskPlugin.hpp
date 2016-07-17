#ifndef SimTaskPlugin_HPP
#define SimTaskPlugin_HPP

#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

#include <rw/common/Timer.hpp>
#include <rw/sensor/Contact3D.hpp>
#include <rwlibs/task/Task.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include "ui_SimTaskPlugin.h"

#include <QObject>

namespace rw { namespace models { class Device; } }
namespace rw { namespace proximity { class CollisionDetector; } }
namespace rwlibs { namespace control { class JointController; } }
namespace rwsim { namespace sensor { class BodyContactSensor; } }
namespace rwsim { namespace dynamics { class DynamicDevice; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace dynamics { class RigidDevice; } }
namespace rwsim { namespace simulator { class DynamicSimulator; } }
namespace rwsim { namespace simulator { class ThreadSimulator; } }

class PropertyViewEditor;
class QTimer;

/**
 * @brief A plugin that continuesly grasps an object from a target pose whereafter it is
 * lifted to a home pose.
 *
 * The home and target poses are controlled through a task description file. Which is
 * allso used to write back all the results of the simulation.
 *
 * The configuration of the simulation is setup through properties. These can be set from the
 * command prompt, loaded by file, or edited in the gui. These properties include:
 *
 * - Simulator
 * - TimeStepSize
 * - HandOpenConfig
 * - HandCloseConfig
 * - MinRestingTime
 *
 *
 */
class SimTaskPlugin: public rws::RobWorkStudioPlugin, private Ui::SimTaskPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
	Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "GraspTableGeneratorPlugin.json")
#endif
public:
    SimTaskPlugin();
	virtual ~SimTaskPlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();
    /**
     * @brief we listen for events regarding opening and closing of dynamic
     * workcell
     */
    void genericEventListener(const std::string& event);


    void loadTasks(bool automatic);
    void saveTasks(bool automatic);
    void exportMathematica(const std::string& filename);
    void loadConfig(bool automatic);
    void saveConfig();
    void updateConfig();

    void makeSimulator();
    void step(rwsim::simulator::ThreadSimulator* sim, const rw::kinematics::State& state);
    void startSimulation();

    rw::common::PropertyMap& settings();

    //std::vector<rw::sensor::Contact3D> getObjectContacts(const rw::kinematics::State& state);
    struct GraspedObject {
        GraspedObject():object(NULL){}
        rwsim::dynamics::RigidBody::Ptr object;
        std::vector<rw::sensor::Contact3D> contacts;
        std::vector<rwsim::dynamics::Body::Ptr> bodies;
    };

    GraspedObject getObjectContacts(const rw::kinematics::State& state);
    std::vector<rw::sensor::Contact3D> getObjectContacts(const rw::kinematics::State& state,
                                                         rwsim::dynamics::RigidBody::Ptr object,
                                                         rw::common::Ptr<rwsim::sensor::BodyContactSensor> sensor,
                                                         std::vector<rwsim::dynamics::Body::Ptr>& bodies);

    rw::math::Q calcGraspQuality(const rw::kinematics::State& state);


    // for task iteration
    bool hasNextTarget();
    rwlibs::task::CartesianTarget::Ptr getNextTarget();
    void setTask(int);
    rwlibs::task::CartesianTask::Ptr getTask();
    rwlibs::task::CartesianTarget::Ptr getTarget();
private slots:
    void btnPressed();

    void stateChangedListener(const rw::kinematics::State& state);

private:
    typedef enum Status {UnInitialized = 0,
        Success,
        CollisionInitially,
        ObjectMissed,
        ObjectDropped,
        ObjectSlipped,
        TimeOut,
        SimulationFailure,
        InvKinFailure} TestStatus;

    rw::models::WorkCell* _wc;
    rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
    rw::common::Ptr<rwsim::simulator::ThreadSimulator> _tsim;
    rw::common::Ptr<rwsim::simulator::DynamicSimulator> _sim;

    rw::math::Transform3D<> _bTe,
                            _wTe_n,
                            _wTe_home;
                            //_restObjTransform;
    rw::kinematics::State _restObjState, _postLiftObjState, _homeState, _simState;

    //rw::math::Q _startQ;
    rw::common::Ptr<rw::models::Device> _hand;
    rw::common::Ptr<rwsim::dynamics::DynamicDevice> _dhand;
    rw::common::Ptr<rwsim::dynamics::RigidDevice> _rhand;
    rw::kinematics::MovableFrame *_mbase;
    rw::kinematics::Frame *_tcp;
    //rwsim::dynamics::RigidBody *_object;
    std::vector<rwsim::dynamics::RigidBody::Ptr> _objects;
    rw::common::Ptr<rwlibs::control::JointController> _controller;
    //rwsim::sensor::BodyContactSensor::Ptr _bsensor;
    std::vector<rw::common::Ptr<rwsim::sensor::BodyContactSensor> > _bsensors;

    rw::math::Transform3D<> _home, _approach, _approachDef;
    //rw::math::Transform3D<double> _objectBeginLift;

    rwlibs::task::CartesianTask::Ptr _roottask,_currenttask;
    std::vector<rwlibs::task::CartesianTask::Ptr> _taskQueue;
    std::vector<rwlibs::task::CartesianTarget::Ptr> *_targets;
    std::vector<std::pair<rwlibs::task::CartesianTask::Ptr, rwlibs::task::CartesianTarget::Ptr> > _alltargets;
    int _currentTaskIndex;
    int _currentTargetIndex, _nextTargetIndex;
    int _nrOfExperiments, _totalNrOfExperiments;

    int _failed, _success, _slipped, _collision, _timeout, _simfailed,_skipped;

    typedef enum{GRASPING, LIFTING, NEW_GRASP, APPROACH} SimState;
    SimState _currentState;

    double _graspTime,
           _approachedTime; // the simulation time when the approach has finished
    rw::math::Q _closeQ, _openQ;
    rw::math::Q _graspedQ;
    QTimer *_timer;
    rw::common::Timer _wallTimer, _wallTotalTimer;
    double _restingTime, _simTime;
    bool _stopped;
    bool _configured;
    //bool _calcWrenchQuality;
    double _maxObjectGripperDistance;
    rw::common::Ptr<rw::proximity::CollisionDetector> _collisionDetector;
    //rw::common::Timer _wallTimer;
    /**
     *
     *
     *
     * restingtime
     *
     *
     */

    rw::common::PropertyMap _config;
    PropertyViewEditor *_propertyView;
    int _lastSaveTaskIndex;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
