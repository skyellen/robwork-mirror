#ifndef GTaskVisPlugin_HPP
#define GTaskVisPlugin_HPP

#include <rw/common/Timer.hpp>
#include <rw/graphics/Render.hpp>
#include <rwlibs/task.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <QObject>
#include <QtGui>
#include <QTimer>

#include <boost/any.hpp>

#include "ui_GTaskVisPlugin.h"

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
class GTaskVisPlugin: public rws::RobWorkStudioPlugin, private Ui::GTaskVisPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:

    GTaskVisPlugin();

    virtual ~GTaskVisPlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();
    /**
     * @brief we listen for events regarding opening and closing of dynamic
     * workcell
     */
    void genericEventListener(const std::string& event);
    void genericAnyEventListener(const std::string& event, boost::any data);


    void loadTasks(bool automatic);
    void saveTasks(bool automatic);
    void loadConfig(bool automatic);
    void saveConfig();
    //void updateConfig();
    rw::common::PropertyMap& settings();

private slots:
    void updateVis();
    void loadTasks(QString taskFile);
    void btnPressed();
    void stateChangedListener(const rw::kinematics::State& state);
    void selectGrasp(int i);
	void on_btnRecordVideo_clicked();
private:
    rw::models::WorkCell* _wc;
    int _nrOfExperiments, _totalNrOfExperiments;

    QTimer *_timer;
    rwlibs::task::GraspTask::Ptr _graspTask;
    std::vector<std::pair<rwlibs::task::GraspSubTask*, rwlibs::task::GraspTarget*> > _ymtargets;
    rw::graphics::Render::Ptr _render;
};

#endif
