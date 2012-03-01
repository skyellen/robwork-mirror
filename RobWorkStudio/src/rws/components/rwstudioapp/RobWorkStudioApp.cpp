
#include "RobWorkStudioApp.hpp"

#include <rw/common/ProgramOptions.hpp>
#include <RobWorkStudioConfig.hpp>
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

#include <rw/rw.hpp>
USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace rws;

RobWorkStudioApp::RobWorkStudioApp(const std::string& args):
            _args(args)
        {}


RobWorkStudioApp::~RobWorkStudioApp()
{

}

void RobWorkStudioApp::start(){
    _thread = new boost::thread(boost::bind(&RobWorkStudioApp::run, this));
}


void initReasource(){
    Q_INIT_RESOURCE(rwstudio_resources);
}

 void RobWorkStudioApp::run(){
     initReasource();
     rw::common::ProgramOptions poptions("RobWorkStudio", RW_VERSION);
     poptions.addStringOption("ini-file", "RobWorkStudio.ini", "RobWorkStudio ini-file");
     poptions.addStringOption("input-file", "", "Project/Workcell/Device input file");
     poptions.setPositionalOption("input-file", -1);
     poptions.initOptions();
     poptions.parse(_args);

     PropertyMap map = poptions.getPropertyMap();

     std::string inifile = map.get<std::string>("ini-file", "");
     std::string inputfile = map.get<std::string>("input-file", "");
     {

         char *argv[30];
         std::vector<std::string> args = boost::program_options::split_unix(_args);
         for(size_t i=0;i<args.size();i++){
             argv[i] = &(args[i][0]);
         }

         int params = args.size();
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
 }
