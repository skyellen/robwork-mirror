#ifndef GTaskVisPlugin_HPP
#define GTaskVisPlugin_HPP

#include <rws/RobWorkStudioPlugin.hpp>

#include <QObject>

#include <boost/any.hpp>

#include "ui_GTaskVisPlugin.h"

namespace rw { namespace graphics { class DrawableNode; } }
namespace rw { namespace graphics { class Render; } }
namespace rwlibs { namespace task { class GraspTask; } }
namespace rwlibs { namespace task { class GraspSubTask; } }
namespace rwlibs { namespace task { class GraspTarget; } }

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
class GTaskVisPlugin: public rws::RobWorkStudioPlugin, private Ui::GTaskVisPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
public:
	//! @brief Constructor.
    GTaskVisPlugin();

    //! @brief Destructor.
    virtual ~GTaskVisPlugin();

    //! @copydoc RobWorkStudioPlugin::open
    virtual void open(rw::models::WorkCell* workcell);

    //! @copydoc RobWorkStudioPlugin::close
    virtual void close();

    //! @copydoc RobWorkStudioPlugin::initialize
    virtual void initialize();

    /**
     * @brief Allows sending events from other plugins.
     *
     * The following events are currently understood:
     *
     * If \b event is GVis::LoadFile tasks are loaded from the filename given in \b data.
     *
     * If \b event is GVis::Update the graphics is updated.
     *
     * If \b event is GVis::SelectGrasp the grasp number given in \b data is selected.
     *
     * @param event
     * @param data
     */
    void genericAnyEventListener(const std::string& event, boost::any data);

    /**
     * @brief Load task file.
     * @param automatic [in] if false a dialog is shown to choose the file.
     */
    void loadTasks(bool automatic);

    //void saveTasks(bool automatic);
    //void loadConfig(bool automatic);
    //void saveConfig();
    //void updateConfig();

    /**
     * @brief Get the settings.
     * @return the settings.
     */
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
    rw::common::Ptr<rwlibs::task::GraspTask> _graspTask;
    std::vector<std::pair<rwlibs::task::GraspSubTask*, rwlibs::task::GraspTarget*> > _ymtargets;
    rw::common::Ptr<rw::graphics::Render> _render;
    rw::common::Ptr<rw::graphics::DrawableNode> _targetDrawable;
};

#endif
