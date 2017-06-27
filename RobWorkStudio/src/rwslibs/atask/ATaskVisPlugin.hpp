/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWSLIBS_ATASKVISPLUGIN_HPP_
#define RWSLIBS_ATASKVISPLUGIN_HPP_

/**
 * @file ATaskVisPlugin.hpp
 *
 * \copydoc rwslibs::ATaskVisPlugin
 */

#include "ui_ATaskVisPlugin.h"
#include <rws/RobWorkStudioPlugin.hpp>

#include <boost/any.hpp>

// Forward declarations
class PropertyViewEditor;
namespace rw { namespace models { class WorkCell; } }
namespace rwlibs { namespace assembly {
class AssemblyResult;
class AssemblyTask;
} }
namespace rwlibs { namespace opengl { class RenderForceTorque; } }
namespace rwlibs { namespace opengl { class RenderPointCloud; } }
namespace rwlibs { namespace opengl { class RenderLines; } }

namespace rwslibs {
//! @addtogroup rwslibs
//! @{
/**
 * @brief Plugin for visualization of assembly operations.
 *
 * \image html assembly/ATaskVisPlugin.png "The plugin for visualization of assembly operations."
 *
 * This plugin works best when used together with the rws::PlayBack plugin but can be compiled and used independently.
 * When a workcell is loaded, the assembly task definition and results can be loaded in the plugin. If the result includes trajectory information,
 * this can be shown with the playback plugin. The information under the result section, will be updated based on the time step chosen in the playback plugin.
 * This plugin will also add additional renders to the RobWorkStudio scene, for rendering of contact and force/torque information stored in the assembly task and result.
 *
 * It is possible to show a library of assembly strategies with the StrategyLibraryDialog. This dialog will also help set up a new assembly task definition if needed.
 *
 * To use the plugin, create a RobWorkStudio.ini file in the RobWorkStudio/bin/debug folder with the following content (for debug build):
 * \par RobWorkStudio.ini:
 *
 * \code
 *  [Plugins]
 *  ATaskVisPlugin\DockArea=2
 *  ATaskVisPlugin\Filename=librws_atask
 *  ATaskVisPlugin\Path=../../libs/debug
 *  ATaskVisPlugin\Visible=true
 * \endcode
 */
class ATaskVisPlugin: public rws::RobWorkStudioPlugin, private Ui::ATaskVisPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
public:
	//! @brief Constructor
	ATaskVisPlugin();

	//! @brief Destructor
    virtual ~ATaskVisPlugin();

	//! @copydoc rws::RobWorkStudioPlugin::open
    virtual void open(rw::models::WorkCell* workcell);

	//! @brief When the RobWorkStudio instance is valid, the ATaskVisPlugin will subscribe to events received from the PlayBack plugin.
    virtual void initialize();

private:
    void genericAnyEventListener(const std::string& event, boost::any data);

private slots:
    void btnPressed();

private:
    void loadTasks();
    void loadResults();
    void selectTask(std::size_t i);
    void selectResult(std::size_t i);
    void constructPlayback();
    rw::common::PropertyMap& settings();

private:
    rw::models::WorkCell* _wc;
    PropertyViewEditor* _editor;
    std::vector<rw::common::Ptr<rwlibs::assembly::AssemblyTask> > _tasks;
    std::vector<rw::common::Ptr<rwlibs::assembly::AssemblyResult> > _results;
    rw::common::Ptr<rwlibs::assembly::AssemblyTask> _currentTask;
    rw::common::Ptr<rwlibs::assembly::AssemblyResult> _currentResult;
    bool _showReal;
    rw::common::Ptr<rwlibs::opengl::RenderForceTorque> _maleFTrender;
    rw::common::Ptr<rwlibs::opengl::RenderForceTorque> _femaleFTrender;
    rw::common::Ptr<rwlibs::opengl::RenderPointCloud> _contactPointRender;
    rw::common::Ptr<rwlibs::opengl::RenderLines> _contactNormalRender;
};
//! @}
} /* namespace rwslibs */
#endif /* RWSLIBS_ATASKVISPLUGIN_HPP_ */
