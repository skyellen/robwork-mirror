/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_PLUGINS_ENGINETESTPLUGIN_HPP_
#define RWSIMLIBS_PLUGINS_ENGINETESTPLUGIN_HPP_

/**
 * @file EngineTestPlugin.hpp
 *
 * \copydoc rwsimlibs::plugins::EngineTestPlugin
 */

#include <rws/RobWorkStudioPlugin.hpp>
#include <rwsimlibs/test/EngineTest.hpp>

#include <QListWidgetItem>

#include <boost/any.hpp>

class PropertyViewEditor;
namespace rw { namespace common { class ThreadPool; } }
namespace rw { namespace common { class ThreadTask; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace log { class SimulatorLogScope; } }
namespace rwsimlibs { namespace gui { class SimulatorLogWidget; } }

namespace Ui { class EngineTestPlugin; }

namespace rwsimlibs {
namespace plugins {
//! @addtogroup rwsimlibs_plugins

//! @{
/**
 * @brief A plugin for interactive testing of physics engines.
 */
class EngineTestPlugin: public rws::RobWorkStudioPlugin {
	Q_OBJECT
	Q_INTERFACES( rws::RobWorkStudioPlugin )
	#if RWS_USE_QT5
		Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "EngineTestPlugin.json")
	#endif
public:
	//! @brief Constructor.
	EngineTestPlugin();

	//! @brief Destructor.
	virtual ~EngineTestPlugin();

	//! @copydoc rws::RobWorkStudioPlugin::initialize
	void initialize();

private slots:
	void toolBoxChanged(int index);
	void engineChanged(QListWidgetItem* current);
	void testChanged(QListWidgetItem* current);
	void run();
	void verbose();
	void inputChanged();
	void updateInput();
	void predefinedChoice(int choice);
	void resultShow();
	void logCheck(int state);

private:
	void message(const std::string& msg);
	void error(const std::string& msg);

	void simulatorCallBack(double time, bool failure, bool done);
	bool event(QEvent* event);

	void genericAnyEventListener(const std::string& event, boost::any data);

private:
    Ui::EngineTestPlugin* const _ui;
    PropertyViewEditor* const _inputEditor;
    rwsimlibs::test::EngineTest::Ptr _test;
    rwsimlibs::test::EngineTest::TestHandle::Ptr _testHandle;
    std::string _engine;
    rw::common::Ptr<rw::common::PropertyMap> _input;
    rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
    rw::common::Ptr<rwsim::log::SimulatorLogScope> _log;
    rwsimlibs::gui::SimulatorLogWidget* _logWidget;
    rw::common::Ptr<rw::common::ThreadPool> _threadPool;
    rw::common::Ptr<rw::common::ThreadTask> _runTask;
    bool _simFailed;
};
//! @}
} /* namespace plugins */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_PLUGINS_ENGINETESTPLUGIN_HPP_ */
