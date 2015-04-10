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
#include <QSplashScreen>
#include <rw/common/ProgramOptions.hpp>
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
#include <rw/loaders/xml/XMLPathFormat.hpp>
#include <boost/foreach.hpp>

#ifdef __WIN32
#include <omp.h> //Needed because otherwise Visual Studio results in run-time linking problems.
#endif

#ifdef RWS_USE_STATIC_LINK_PLUGINS
    #include <rwslibs/log/ShowLog.hpp>
    #include <rwslibs/jog/Jog.hpp>
    //#include <rwslibs/pointer/PointerPlugin.hpp>
    #include <rwslibs/treeview/TreeView.hpp>
    #include <rwslibs/playback/PlayBack.hpp>
    #include <rwslibs/planning/Planning.hpp>
    #include <rwslibs/propertyview/PropertyView.hpp>
    #include <rwslibs/sensors/Sensors.hpp>
#ifdef RW_HAVE_EIGEN
    //#include <rwlibs/calibration/Calibration.hpp>
#endif
#if RWS_HAVE_LUA
    #include <rwslibs/lua/Lua.hpp>
#endif
#endif

using namespace rw;
using namespace rw::common;
using namespace rw::loaders;
using namespace rws;

class MyQApplication: public QApplication {
public:
    MyQApplication(int& argc, char** argv):QApplication(argc,argv){}

    bool notify(QObject * rec, QEvent * ev)
    {
      try {
          return QApplication::notify(rec,ev);
      } catch(std::exception & e) {
          QMessageBox::warning(0,tr("An error occurred"), e.what());
      } catch(...) {
          QMessageBox::warning(0,
                             tr("An unexpected error occurred"),
                             tr("This is likely a bug."));
      }
      return false;
    }
};


int main(int argc, char** argv)
{
    Q_INIT_RESOURCE(rwstudio_resources);

    // now initialize robwork, such that plugins ad stuff might work
    RobWork::init(argc,argv);

    ProgramOptions poptions("RobWorkStudio", RW_VERSION);
    poptions.addStringOption("ini-file", "RobWorkStudio.ini", "RobWorkStudio ini-file");
    poptions.addStringOption("input-file", "", "Project/Workcell/Device input file");
    poptions.addStringOption("nosplash", "", "If defined the splash screen will not be shown");
    poptions.setPositionalOption("input-file", -1);
    poptions.initOptions();
    poptions.parse(argc, argv);

    PropertyMap map = poptions.getPropertyMap();
    bool showSplash = false; //!map.has("nosplash");
    std::string inifile = map.get<std::string>("ini-file", "");
    std::string inputfile = map.get<std::string>("input-file", "");
    {
        MyQApplication app(argc, argv);
        try {
            QSplashScreen *splash;
            if(showSplash){
                QPixmap pixmap(":/images/splash.jpg");
                splash = new QSplashScreen(pixmap);
                splash->show();
                // Loading some items
                splash->showMessage("Adding static plugins");
            }
            app.processEvents();
            // Establishing connections
            if(showSplash)
                splash->showMessage("Loading static plugins");
            std::string pluginFolder = "./plugins/";

            {
                Timer t;
                rws::RobWorkStudio rwstudio(map);
                #ifdef RWS_USE_STATIC_LINK_PLUGINS
                    rwstudio.addPlugin(new rws::ShowLog(), false, Qt::BottomDockWidgetArea);
                    rwstudio.addPlugin(new rws::Jog(), false, Qt::LeftDockWidgetArea);
                    //rwstudio.addPlugin(new rws::PointerPlugin(), false, Qt::LeftDockWidgetArea);
                    rwstudio.addPlugin(new rws::TreeView(), false, Qt::LeftDockWidgetArea);
                    rwstudio.addPlugin(new rws::PlayBack(), false, Qt::BottomDockWidgetArea);
                    rwstudio.addPlugin(new rws::PropertyView(), false, Qt::LeftDockWidgetArea);
                    rwstudio.addPlugin(new rws::Planning(), false, Qt::LeftDockWidgetArea);
                    rwstudio.addPlugin(new rws::Sensors(), false, Qt::RightDockWidgetArea);

					#ifdef RW_HAVE_EIGEN
                    rwstudio.addPlugin(new rws::Calibration(), false, Qt::RightDockWidgetArea);
					#endif

                    #if RWS_HAVE_LUA
                    rwstudio.addPlugin(new rws::Lua(), false, Qt::LeftDockWidgetArea);
                    #endif

                    #if RWS_HAVE_SANDBOX
                        //Plugins which are avaible in the sandbox
                    #endif
                #endif
                if(showSplash)
                    splash->showMessage("Loading static plugins");

                rwstudio.loadSettingsSetupPlugins( inifile );

				std::cout<<XMLPathFormat::QPathId<<std::endl;
				std::cout<<XMLPropertyFormat::PropertyMapId<<std::endl;

                if(!inputfile.empty()){
                    if(showSplash)
                        splash->showMessage("Opening workcell...");
                    rwstudio.openFile(inputfile);
                }

                // load configuration into RobWorkStudio
                if(showSplash){
                    splash->showMessage("Loading settings");
                    splash->finish(&rwstudio);
                }

                rwstudio.show();

                app.exec();
            }
        } catch (const Exception& e) {
            std::cout << e.what() << std::endl;
            QMessageBox::critical(NULL, "RW Exception", e.what());
            return -1;
        } catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            QMessageBox::critical(NULL, "Exception", e.what());
            return -1;
        }
    }
    //return 0;
}

#ifdef _WIN32
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd) {
	return main(__argc, __argv);
}
#endif
