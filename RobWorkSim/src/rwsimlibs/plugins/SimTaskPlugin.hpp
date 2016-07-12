#ifndef SimTaskPlugin_HPP
#define SimTaskPlugin_HPP

#include <rws/RobWorkStudioPlugin.hpp>
#include "ui_SimTaskPlugin.h"

#include <rw/common/Timer.hpp>
#include <rwsim/simulator/GraspTaskSimulator.hpp>

#include <QObject>

namespace rwsim { namespace drawable { class SimulatorDebugRender; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace simulator { class GraspTaskSimulator; } }

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
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "SimTaskPlugin.json")
#endif
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
    void setCurrentTask(rwlibs::task::GraspTask::Ptr task);
    rwlibs::task::GraspTask::Ptr generateTasks(int nrTasks);
private:
    rw::models::WorkCell* _wc;
    rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
    rw::common::Ptr<rwsim::simulator::GraspTaskSimulator> _graspSim;

    QTimer *_timer;
    rw::common::Timer _wallTimer, _wallTotalTimer;
    double _restingTime, _simTime;

    rw::common::Ptr<rwsim::drawable::SimulatorDebugRender> _debugRender;
    rw::common::PropertyMap _config;
    PropertyViewEditor *_propertyView;
    rw::kinematics::State _initState;
    rwlibs::task::GraspTask::Ptr _mergedResult;
    rwlibs::task::GraspTask::Ptr _seedTargets;

    int _nrOfTargetsToGen;
    int _imgRecordPostfix;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
