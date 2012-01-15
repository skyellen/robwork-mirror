#ifndef SimTaskVisPlugin_HPP
#define SimTaskVisPlugin_HPP


#include <rwlibs/task.hpp>
#include <rwsim/rwsim.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwsim/control/BodyController.hpp>
#include "ui_SimTaskVisPlugin.h"
#include <rwlibs/task/GraspTask.hpp>
#include <QObject>
#include <QtGui>
#include <QTimer>
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
class SimTaskVisPlugin: public rws::RobWorkStudioPlugin, private Ui::SimTaskVisPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:

    SimTaskVisPlugin();

    virtual ~SimTaskVisPlugin();

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
    void loadConfig(bool automatic);
    void saveConfig();
    void updateConfig();
    rw::common::PropertyMap& settings();
private slots:

    void btnPressed();

    void stateChangedListener(const rw::kinematics::State& state);

private:
    rw::models::WorkCell* _wc;
    rwsim::dynamics::DynamicWorkCell::Ptr _dwc;

    int _nrOfExperiments, _totalNrOfExperiments;

    QTimer *_timer;
    rwlibs::task::GraspTask::Ptr _graspTask;
    std::vector<std::pair<rwlibs::task::GraspSubTask*, rwlibs::task::GraspTarget*> > _ymtargets;

    rw::graphics::Render::Ptr _render;
};

#endif
