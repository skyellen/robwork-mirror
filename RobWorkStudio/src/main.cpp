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

#ifdef RWS_USE_STATIC_LINK_PLUGINS
    #include <rws/plugins/log/ShowLog.hpp>
    #include <rws/plugins/jog/Jog.hpp>
    #include <rws/plugins/treeview/TreeView.hpp>
    #include <rws/plugins/playback/PlayBack.hpp>
    #include <rws/plugins/planning/Planning.hpp>
    #include <rws/plugins/propertyview/PropertyView.hpp>
    #include <rws/plugins/sensors/Sensors.hpp>
    #include <rws/plugins/lua/Lua.hpp>
#endif


using namespace rw;
using namespace rw::common;
using namespace rw::loaders;
using namespace rws;




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

    {
        QApplication app(argc, argv);
        try {
        
            QPixmap pixmap(":/images/splash.jpg");

            QSplashScreen splash(pixmap);
            splash.show();
            // Loading some items
            splash.showMessage("Adding static plugins");

            app.processEvents();
            // Establishing connections
            splash.showMessage("Loading static plugins");
            std::string pluginFolder = "./plugins/";

            {
                rws::RobWorkStudio rwstudio(map);

                #ifdef RWS_USE_STATIC_LINK_PLUGINS
                    rwstudio.addPlugin(new rws::ShowLog(), false, Qt::BottomDockWidgetArea);
                    rwstudio.addPlugin(new rws::Jog(), false, Qt::LeftDockWidgetArea);
                    rwstudio.addPlugin(new rws::TreeView(), false, Qt::LeftDockWidgetArea);
                    rwstudio.addPlugin(new rws::PlayBack(), false, Qt::BottomDockWidgetArea);
                    rwstudio.addPlugin(new rws::PropertyView(), false, Qt::LeftDockWidgetArea);
                    rwstudio.addPlugin(new rws::Planning(), false, Qt::LeftDockWidgetArea);
                    rwstudio.addPlugin(new rws::Sensors(), false, Qt::RightDockWidgetArea);


                    #if RWS_HAVE_LUA
                    rwstudio.addPlugin(new rws::Lua(), false, Qt::LeftDockWidgetArea);
                    #endif

                    #if RWS_HAVE_SANDBOX
                        //Plugins which are avaible in the sandbox
                    #endif
                #endif

                splash.showMessage("Loading static plugins");
                rwstudio.loadSettingsSetupPlugins( inifile );

                if(!inputfile.empty()){
                    splash.showMessage("Openning dynamic workcell...");
                    rwstudio.openFile(inputfile);
                }

                // load configuration into RobWorkStudio
                splash.showMessage("Loading settings");
                splash.finish(&rwstudio);
                rwstudio.show();
                res = app.exec();
            }
        } catch (const Exception& e) {
            std::cout << e.what() << std::endl;
            QMessageBox::critical(NULL, "RW Exception", e.what().c_str());
            return -1;
        } catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            QMessageBox::critical(NULL, "Exception", e.what());
            return -1;
        }
    }
    return 0;
}

