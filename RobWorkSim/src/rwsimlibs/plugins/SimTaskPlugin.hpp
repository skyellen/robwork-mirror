#ifndef SimTaskPlugin_HPP
#define SimTaskPlugin_HPP

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <rwsim/rwsim.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwsim/control/BodyController.hpp>
#include "ui_SimTaskPlugin.h"

#include <rwsim/simulator/GraspTaskSimulator.hpp>

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
    /**
     * @brief constructor
     */
    SimTaskPlugin();

    //! @brief destructor
	virtual ~SimTaskPlugin();

	//! @copydoc rws::RobWorkStudioPlugin::open(rw::models::WorkCell* workcell)
    virtual void open(rw::models::WorkCell* workcell);

    //! @copydoc rws::RobWorkStudioPlugin::close()
    virtual void close();

    //! @copydoc rws::RobWorkStudioPlugin::initialize()
    virtual void initialize();

    /**
     * @brief we listen for events regarding opening and closing of dynamic
     * workcell
     * @param event [in] the event id
     */
    void genericEventListener(const std::string& event);

    void loadTasks(bool automatic);
    void saveTasks(bool automatic);
    void exportMathematica(const std::string& filename);
    void loadConfig(bool automatic);
    void saveConfig();
    void updateConfig();
    void startSimulation();

    rw::common::PropertyMap& settings();

private slots:
    /**
     * @
     */
    void btnPressed();

    void stateChangedListener(const rw::kinematics::State& state);
private:
    void setCurrentTask(GraspTask::Ptr task);
    GraspTask::Ptr generateTasks(int nrTasks);
private:
    rw::models::WorkCell* _wc;
    rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
    rwsim::simulator::GraspTaskSimulator::Ptr _graspSim;

    QTimer *_timer;
    rw::common::Timer _wallTimer, _wallTotalTimer;
    double _restingTime, _simTime;

    rwsim::drawable::SimulatorDebugRender::Ptr _debugRender;
    rw::common::PropertyMap _config;
    PropertyViewEditor *_propertyView;
    rw::kinematics::State _initState;
    GraspTask::Ptr _mergedResult;
    GraspTask::Ptr _seedTargets;

    int _nrOfTargetsToGen;
    int _imgRecordPostfix;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
