%module rws

%{
#include <rwlibs/lua/RemoteTypes.hpp>
#include <rws/lua/RemoteTypes.hpp>
#include <rw/common/Ptr.hpp>
using namespace rwlibs::rwr;
using namespace rws::rwr;
%}

%import <rwlibs/lua/rw.i>

%{

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

#include <rw/common/ProgramOptions.hpp>
class RobWorkStudioApplication : public QThread
 {
 public:
    RobWorkStudioApplication(const std::string& args):
        _args(args)
    {}

     void run(){
         Q_INIT_RESOURCE(rwstudio_resources);
         int res = 0;
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
             for(int i=0;i<args.size();i++){
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
                     res = app.exec();
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

     RobWorkStudio *_rwstudio;
     std::string _args;
 };

static RobWorkStudioApplication *robApp = NULL;

rw::common::Ptr<RobWorkStudio> getRobWorkStudioInstance(const std::string& args){

    // create a thread that start QApplication and
    if(robApp==NULL){
        robApp =new RobWorkStudioApplication( args );
        robApp->start();
        while(robApp->_rwstudio==NULL){
            rw::common::TimerUtil::sleepMs(100);
        }
    }
    return robApp->_rwstudio;
}

%}

%template (RobWorkStudioPtr) rw::common::Ptr<RobWorkStudio>;
%template (RWStudioView3DPtr) rw::common::Ptr<RWStudioView3D>;


rw::common::Ptr<RobWorkStudio> getRobWorkStudioInstance(const std::string& args);

class RWStudioView3D {
public:
    RWStudioView3D(RobWorkStudio* rwStudio, QWidget* parent);
    void showPivotPoint(bool visible);
    //void setDrawType(rw::graphics::DrawableNode::DrawType drawType);
    Frame* pickFrame(int x, int y);
    rw::common::Ptr<DrawableNode> pick(int x, int y);

    rw::common::Ptr<WorkCellScene> getWorkCellScene();
    rw::common::Ptr<SceneViewer> getSceneViewer();
    void saveBufferToFile(const QString& filename);

};

class RobWorkStudio {
public:
    RobWorkStudio(const PropertyMap& map);

    void openFile(const std::string& filename);

    PropertyMap& getPropertyMap();

    void setWorkcell(rw::common::Ptr<WorkCell> workcell);

    rw::common::Ptr<WorkCell> getWorkcell();

    rw::common::Ptr<CollisionDetector> getCollisionDetector();

    rw::common::Ptr<WorkCellScene> getWorkCellScene();

    rw::common::Ptr<RWStudioView3D> getView();

    const TimedStatePath& getTimedStatePath();

    void setTimedStatePath(const TimedStatePath& path);

    void updateAndRepaint();


    void setState(const State& state);
    void postState(const State& state);
    void postUpdateAndRepaint();
    void postSaveViewGL(const std::string& str);

    void postTimedStatePath(const TimedStatePath& path);
    void postExit();


    //Transform3D getViewTransform();
    //void setViewTransform(Transform3D t3d);

    // events
    //StateChangedEvent& stateChangedEvent();
    //FrameSelectedEvent& frameSelectedEvent();
    //GenericEvent& genericEvent();
    //KeyEvent& keyEvent();
    //MousePressedEvent& mousePressedEvent();
    //StateTrajectoryChangedEvent& stateTrajectoryChangedEvent();
    //PositionSelectedEvent& positionSelectedEvent();

};
