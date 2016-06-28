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

#include "RobWorkStudioApp.hpp"

#include <QApplication>
#include <QSplashScreen>

#include <rw/common/ProgramOptions.hpp>
#include <RobWorkStudioConfig.hpp>
#ifdef RWS_USE_STATIC_LINK_PLUGINS
    #include <rwslibs/log/ShowLog.hpp>
    #include <rwslibs/jog/Jog.hpp>
    #include <rwslibs/treeview/TreeView.hpp>
    #include <rwslibs/playback/PlayBack.hpp>
    #include <rwslibs/planning/Planning.hpp>
    #include <rwslibs/propertyview/PropertyView.hpp>
    #include <rwslibs/sensors/Sensors.hpp>
#ifdef RWS_HAVE_LUA
    #include <rwslibs/lua/Lua.hpp>
#endif
#endif

#include <boost/program_options/parsers.hpp>

#include <rw/rw.hpp>
USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace rws;

RobWorkStudioApp::RobWorkStudioApp(const std::string& args):
	_rwstudio(NULL),
	_args(args),
	_thread(NULL),
	_isRunning(false)
{
}


RobWorkStudioApp::~RobWorkStudioApp()
{
}

void RobWorkStudioApp::start(){
    _isRunning=true;
    _thread = new boost::thread(boost::bind(&RobWorkStudioApp::run, this));
}


void initReasource(){
    Q_INIT_RESOURCE(rwstudio_resources);
}

 void RobWorkStudioApp::run(){
     rw::common::ProgramOptions poptions("RobWorkStudio", RW_VERSION);
     poptions.addStringOption("ini-file", "RobWorkStudio.ini", "RobWorkStudio ini-file");
     poptions.addStringOption("input-file", "", "Project/Workcell/Device input file");
     poptions.setPositionalOption("input-file", -1);
     poptions.initOptions();
     if(poptions.parse(_args)<0){
         _isRunning = false;
         return;
     }

     initReasource();

     PropertyMap map = poptions.getPropertyMap();

     std::string inifile = map.get<std::string>("ini-file", "");
     std::string inputfile = map.get<std::string>("input-file", "");
     {

         char *argv[30];
         std::vector<std::string> args = boost::program_options::split_unix(_args);
         for(size_t i=0;i<args.size();i++){
             argv[i] = &(args[i][0]);
         }

         int params = (int)args.size();
         QApplication app(params, argv);

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
                 _rwstudio = &rwstudio;

                 // load configuration into RobWorkStudio
                 splash.showMessage("Loading settings");
                 splash.finish(&rwstudio);
                 rwstudio.show();
                 app.exec();
             }
         } catch (const rw::common::Exception& e) {
             std::cout << e.what() << std::endl;
             QMessageBox::critical(NULL, "RW Exception", e.what());

         } catch (std::exception& e) {
             std::cout << e.what() << std::endl;
             QMessageBox::critical(NULL, "Exception", e.what());

         }
     }
     _isRunning = false;
 }
