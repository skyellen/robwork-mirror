#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <rwsim/rwsim.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwsim/control/BodyController.hpp>
#include "ui_SamplePlugin.h"

#include <QObject>
#include <QtGui>
#include <QTimer>


class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:

    typedef enum Status {UnInitialized = 0, Success, CollisionInitially, ObjectMissed, ObjectDropped, ObjectSlipped, TimeOut, SimulationFailure} TestStatus;

    SamplePlugin();
	virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();
    /**
     * @brief we listen for events regarding opening and closing of dynamic
     * workcell
     */
    void genericEventListener(const std::string& event);

    void loadTasks();

    void makeSimulator();
    void saveTasks();
    void step(const rw::kinematics::State& state);
    void exportMathematica();
    void startSimulation();
private slots:
    void btnPressed();

    void stateChangedListener(const rw::kinematics::State& state);

private:
    rw::models::WorkCell* _wc;
    rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
    rwlibs::task::CartesianTask::Ptr _tasks;
    rwsim::simulator::ThreadSimulator::Ptr _tsim;
    rwsim::simulator::DynamicSimulator::Ptr _sim;
    rwsim::simulator::ODESimulator::Ptr _engine;

    rw::math::Transform3D<> _bTe,
                            _wTe_n,
                            _wTe_home,
                            _restObjTransform;

    std::vector<rwlibs::task::CartesianTarget::Ptr> *_targets;
    rw::math::Q _startQ;
    rw::models::JointDevice* _hand;
    rwsim::dynamics::DynamicDevice *_dhand;
    rw::kinematics::MovableFrame *_mbase;
    rw::math::Transform3D<> _home, _objHome;
    int _nextTaskIndex;

    typedef enum{ GRASPING, LIFTING, NEW_GRASP} SimState;
    SimState _currentState;

    double _graspTime;
    rw::math::Q _closeQ, _openQ;
    rw::math::Q _graspedQ, _liftedQ;
    rwsim::dynamics::RigidBody *_object;
    rwlibs::control::JointController *_controller;
    rwsim::control::BodyController *_bodyController;
    QTimer *_timer;
    TestStatus _status;
    double _restingTime;
    bool _stopped;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
