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

// Forward declarations

//! @addtogroup rwslibs
//! @{
/**
 * @brief Plugin for visualization assembly operations.
 *
 * This plugin works best when used together with the Playback plugin but can be compiled and used independently.
 *
 * To use the plugin, create a RobWorkStudio.ini file in the RobWorkStudio/bin/debug folder with the following content (for debug build):
 * \par RobWorkStudio.ini:
 *
 *  [Plugins]
 *
 *  ATaskVisPlugin\DockArea=2
 *
 *  ATaskVisPlugin\Filename=librws_atask
 *
 *  ATaskVisPlugin\Path=../../libs/debug
 *
 *  ATaskVisPlugin\Visible=true
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

	//! @copydoc rws::RobWorkStudioPlugin::close
    virtual void close();

	//! @copydoc rws::RobWorkStudioPlugin::initialize
    virtual void initialize();

private:
    void genericEventListener(const std::string& event);

    void genericAnyEventListener(const std::string& event, boost::any data);

private slots:
    void btnPressed();

    void stateChangedListener(const rw::kinematics::State& state);

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
