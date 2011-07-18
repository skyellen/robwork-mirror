/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#define QT_NO_EMIT
 
#ifdef __WIN32
#include <windows.h>
#endif //#ifdef __WIN32
#include <QApplication>
#include <QMainWindow>

#include "RobWorkStudio.hpp"

#include <rw/RobWork.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/common/ProgramOptions.hpp>

#include <RobWorkStudioConfig.hpp>
#include <RobWorkConfig.hpp>
 

#include <fstream>

#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>
#include <rw/loaders/xml/XMLPropertyFormat.hpp>
#include <boost/foreach.hpp>


using namespace rw;
using namespace rw::common;
using namespace rw::loaders;
using namespace rws;

#ifdef RWS_USE_STATIC_LINK_PLUGINS

	#include <rws/plugins/log/ShowLog.hpp>
	#include <rws/plugins/jog/Jog.hpp>
	#include <rws/plugins/treeview/TreeView.hpp>
	#include <rws/plugins/playback/PlayBack.hpp>
	#include <rws/plugins/planning/Planning.hpp>
	#include <rws/plugins/propertyview/PropertyView.hpp>
	#include <rws/plugins/sensors/Sensors.hpp>
	#include <rws/plugins/lua/Lua.hpp>

	#ifdef RWS_HAVE_SANDBOX
	//Plugins which are available in the sandbox
	#endif

	std::vector<rws::RobWorkStudio::PluginSetup> getPlugins()
	{
		typedef rws::RobWorkStudio::PluginSetup Pl;
		std::vector<Pl> plugins;

		plugins.push_back(Pl(new rws::ShowLog(), false, Qt::BottomDockWidgetArea));
		plugins.push_back(Pl(new rws::Jog(), false, Qt::LeftDockWidgetArea));
		plugins.push_back(Pl(new rws::TreeView(), false, Qt::LeftDockWidgetArea));
		plugins.push_back(Pl(new rws::PlayBack(), false, Qt::BottomDockWidgetArea));
		plugins.push_back(Pl(new rws::PropertyView(), false, Qt::LeftDockWidgetArea));
		plugins.push_back(Pl(new rws::Planning(), false, Qt::LeftDockWidgetArea));
		plugins.push_back(Pl(new rws::Sensors(), false, Qt::RightDockWidgetArea));
		#if RWS_HAVE_LUA
			plugins.push_back(Pl(new rws::Lua(), false, Qt::LeftDockWidgetArea));
		#endif

		#if RWS_HAVE_SANDBOX
			//Plugins which are avaible in the sandbox
		#endif

		return plugins;
	}
#else
	std::vector<RobWorkStudio::PluginSetup> getPlugins()
	{
		return std::vector<RobWorkStudio::PluginSetup>();
	}

	std::vector<int> getIntegers() {
		return std::vector<int>();
	}
#endif // RW_STATIC_LINK_PLUGINS 

int main(int argc, char** argv)
{

    Q_INIT_RESOURCE(rwstudio_resources);
    int res = 0;
    ProgramOptions poptions("RobWorkStudio", RW_VERSION);
    poptions.addStringOption("ini-file", "RobWorkStudio.ini", "RobWorkStudio ini-file");
    poptions.addStringOption("input-file", "", "Project/Workcell/Device input file");
    poptions.setPositionalOption("input-file", -1);
    poptions.initOptions();
    poptions.parse(argc, argv);

    PropertyMap map = poptions.getPropertyMap();
    std::string inifile = map.get<std::string>("ini-file", "");
    std::string inputfile = map.get<std::string>("input-file", "");
    QApplication app(argc, argv);
    try {
	
        std::vector<rws::RobWorkStudio::PluginSetup> plugins; 
		
        QPixmap pixmap(":/images/splash.jpg");

        QSplashScreen splash(pixmap); 
        splash.show();
        // Loading some items
        splash.showMessage("Adding static plugins");
        
		plugins = getPlugins();

        app.processEvents();
        // Establishing connections
        splash.showMessage("Loading dynamic plugins");

        RobWork robwork;
        std::string pluginFolder = "./plugins/";

		rws::RobWorkStudio rwstudio(&robwork, plugins, map, inifile);
		
        if(!inputfile.empty()){ 
            rwstudio.openFile(inputfile);
        }

        // load configuration into RobWorkStudio
        splash.showMessage("Loading settings");
		
        rwstudio.show();
        splash.finish(&rwstudio);
        res = app.exec();
        Log::infoLog() << "Application Ready to Terminate" << std::endl;
    } catch (const Exception& e) {
        std::cout << e.what() << std::endl;
        QMessageBox::critical(NULL, "RW Exception", e.what().c_str());
        return -1;
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
        QMessageBox::critical(NULL, "Exception", e.what());
        return -1;
    }

    return 0;
}

