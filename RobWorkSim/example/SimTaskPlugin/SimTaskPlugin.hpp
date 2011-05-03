#ifndef SimTaskPlugin_HPP
#define SimTaskPlugin_HPP

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <rwsim/rwsim.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwsim/control/BodyController.hpp>
#include "ui_SimTaskPlugin.h"

#include <QObject>
#include <QtGui>
#include <QTimer>
#include <rws/components/propertyview/PropertyViewEditor.hpp>
#include <rw/common/Timer.hpp>


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
public:

    typedef enum Status {UnInitialized = 0, Success, CollisionInitially, ObjectMissed, ObjectDropped, ObjectSlipped, TimeOut, SimulationFailure} TestStatus;

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
    void step(const rw::kinematics::State& state);
    void startSimulation();

    rw::common::PropertyMap& settings();

    //std::vector<rw::sensor::Contact3D> getObjectContacts(const rw::kinematics::State& state);
    struct GraspedObject {
        GraspedObject():object(NULL){}
        rwsim::dynamics::RigidBody* object;
        std::vector<rw::sensor::Contact3D> contacts;
        std::vector<rwsim::dynamics::Body*> bodies;
    };

    GraspedObject getObjectContacts(const rw::kinematics::State& state);
    std::vector<rw::sensor::Contact3D> getObjectContacts(const rw::kinematics::State& state,
                                                         rwsim::dynamics::RigidBody *object,
                                                         rwsim::sensor::BodyContactSensor::Ptr sensor,
                                                         std::vector<rwsim::dynamics::Body*>& bodies);

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
    rw::models::WorkCell* _wc;
    rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
    rwsim::simulator::ThreadSimulator::Ptr _tsim;
    rwsim::simulator::DynamicSimulator::Ptr _sim;
    rwsim::simulator::ODESimulator::Ptr _engine;

    rw::math::Transform3D<> _bTe,
                            _wTe_n,
                            _wTe_home;
                            //_restObjTransform;
    rw::kinematics::State _restObjState, _postLiftObjState, _homeState;

    //rw::math::Q _startQ;
    rw::models::Device* _hand;
    rwsim::dynamics::DynamicDevice *_dhand;
    rw::kinematics::MovableFrame *_mbase;
    rw::kinematics::Frame *_tcp;
    //rwsim::dynamics::RigidBody *_object;
    std::vector<rwsim::dynamics::RigidBody*> _objects;
    rwlibs::control::JointController *_controller;
    //rwsim::sensor::BodyContactSensor::Ptr _bsensor;
    std::vector< rwsim::sensor::BodyContactSensor::Ptr > _bsensors;

    rw::math::Transform3D<> _home, _approach, _approachDef;
    //rw::math::Transform3D<double> _objectBeginLift;

    rwlibs::task::CartesianTask::Ptr _roottask,_currenttask;
    std::vector<rwlibs::task::CartesianTask::Ptr> _taskQueue;
    std::vector<rwlibs::task::CartesianTask::Ptr> *_subtasks;
    std::vector<rwlibs::task::CartesianTarget::Ptr> *_targets;
    int _currentTaskIndex;
    int _currentTargetIndex, _nextTargetIndex, _nextSubTask;
    int _nrOfExperiments, _totalNrOfExperiments;

    int _failed, _success, _slipped, _collision, _timeout, _simfailed;

    typedef enum{GRASPING, LIFTING, NEW_GRASP, APPROACH} SimState;
    SimState _currentState;

    double _graspTime;
    rw::math::Q _closeQ, _openQ;
    rw::math::Q _graspedQ, _liftedQ;
    QTimer *_timer;
    rw::common::Timer _wallTimer;
    TestStatus _status;
    double _restingTime;
    bool _stopped;
    bool _configured;
    bool _calcWrenchQuality;
    double _maxObjectGripperDistance;
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
