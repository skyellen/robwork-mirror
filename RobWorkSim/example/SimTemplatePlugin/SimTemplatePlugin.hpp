#ifndef SimTemplatePlugin_HPP
#define SimTemplatePlugin_HPP

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <rwsim/rwsim.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwsim/control/BodyController.hpp>
#include "ui_SimTemplatePlugin.h"

#include <QObject>
#include <QtGui>
#include <QTimer>
#include <rws/components/propertyview/PropertyViewEditor.hpp>
#include <rw/common/Timer.hpp>


/**
 * @brief A plugin
 */
class SimTemplatePlugin: public rws::RobWorkStudioPlugin, private Ui::SimTemplatePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:

    /**
     * @brief constructor
     */
    SimTemplatePlugin();

    //! destructor
    virtual ~SimTemplatePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

    /**
     * @brief we listen for events regarding opening and closing of dynamic
     * workcell
     */
    void genericEventListener(const std::string& event);

    void makeSimulator();
    void step(rwsim::simulator::ThreadSimulator* sim, const rw::kinematics::State& state);
    void startSimulation();

private slots:
    void btnPressed();
    void stateChangedListener(const rw::kinematics::State& state);

private:
    rw::models::WorkCell* _wc;
    rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
    rwsim::simulator::ThreadSimulator::Ptr _tsim;
    rwsim::simulator::DynamicSimulator::Ptr _sim;
    rwsim::simulator::ODESimulator::Ptr _engine;

    QTimer *_timer;
    rw::common::Timer _wallTimer, _wallTotalTimer;
/*


    rw::math::Transform3D<> _bTe,
                            _wTe_n,
                            _wTe_home;
                            //_restObjTransform;
    rw::kinematics::State _restObjState, _postLiftObjState, _homeState, _simState;

    //rw::math::Q _startQ;
    rw::models::Device* _hand;
    rwsim::dynamics::DynamicDevice *_dhand;
    rwsim::dynamics::RigidDevice *_rhand;
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
    std::vector<std::pair<rwlibs::task::CartesianTask::Ptr, rwlibs::task::CartesianTarget::Ptr> > _alltargets;
    int _currentTaskIndex;
    int _currentTargetIndex, _nextTargetIndex, _nextSubTask;
    int _nrOfExperiments, _totalNrOfExperiments;

    int _failed, _success, _slipped, _collision, _timeout, _simfailed,_skipped;

    typedef enum{GRASPING, LIFTING, NEW_GRASP, APPROACH} SimState;
    SimState _currentState;

    double _graspTime,
           _approachedTime; // the simulation time when the approach has finished
    rw::math::Q _closeQ, _openQ;
    rw::math::Q _graspedQ, _liftedQ;
    TestStatus _status;
    double _restingTime, _simTime;
    bool _stopped;
    bool _configured;
    bool _calcWrenchQuality;
    double _maxObjectGripperDistance;
    rw::proximity::CollisionDetector::Ptr _collisionDetector;
    //rw::common::Timer _wallTimer;

    rw::common::PropertyMap _config;
    PropertyViewEditor *_propertyView;
    int _lastSaveTaskIndex;
    */
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
