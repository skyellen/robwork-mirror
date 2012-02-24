%module rws

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rws/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>
using namespace rwlibs::swig;
using namespace rws::swig;
%}

%import <rwlibs/swig/rw.i>

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

    rw::common::Ptr<WorkCell> getWorkCell();

    rw::common::Ptr<CollisionDetector> getCollisionDetector();

    rw::common::Ptr<WorkCellScene> getWorkCellScene();

    rw::common::Ptr<RWStudioView3D> getView();

    const TimedStatePath& getTimedStatePath();

    //void updateAndRepaint();
    //void setState(const State& state);
    //void setTimedStatePath(const TimedStatePath& path);
    void postState(const State& state);
    void postUpdateAndRepaint();
    void postSaveViewGL(const std::string& str);
    void postTimedStatePath(const TimedStatePath& path);
    void postExit();

    const State& getState();


    %extend {
        void setTimedStatePath(const TimedStatePath& path){
            $self->postTimedStatePath(path);
        }

        void setState(const State& state){
            $self->postState(state);
        }

        void saveViewGL(const std::string& filename){
            $self->postSaveViewGL( filename );
        }

        Transform3D getViewTransform(){
            return $self->getView()->getSceneViewer()->getTransform();
        }

        void setViewTransform(Transform3D t3d){
            $self->getView()->getSceneViewer()->setTransform(t3d);
            $self->postUpdateAndRepaint();
        }

        void updateAndRepaint(){
            $self->postUpdateAndRepaint();
        }

        void fireGenericEvent(const std::string& str){
            $self->genericEvent().fire(str);
        }
    }
    // events
    //StateChangedEvent& stateChangedEvent();
    //FrameSelectedEvent& frameSelectedEvent();
    //GenericEvent& genericEvent();
    //KeyEvent& keyEvent();
    //MousePressedEvent& mousePressedEvent();
    //StateTrajectoryChangedEvent& stateTrajectoryChangedEvent();
    //PositionSelectedEvent& positionSelectedEvent();

};


RobWorkStudio* getRobWorkStudio();

void setRobWorkStudio(RobWorkStudio* rwstudio);


%inline %{

    const State& getState(){ return getRobWorkStudio()->getState(); }
    void setState(State& state){ return getRobWorkStudio()->postState(state); }
    rw::common::Ptr<Device> findDevice(const std::string& name){
        return getRobWorkStudio()->getWorkCell()->findDevice(name);
    }
    rw::common::Ptr<JointDevice> findJointDevice(const std::string& name){
        return getRobWorkStudio()->getWorkCell()->findDevice<JointDevice>(name);
    }
    rw::common::Ptr<SerialDevice> findSerialDevice(const std::string& name){
        return getRobWorkStudio()->getWorkCell()->findDevice<SerialDevice>(name);
    }
    rw::common::Ptr<TreeDevice> findTreeDevice(const std::string& name){
        return getRobWorkStudio()->getWorkCell()->findDevice<TreeDevice>(name);
    }
    rw::common::Ptr<ParallelDevice> findParallelDevice(const std::string& name){
        return getRobWorkStudio()->getWorkCell()->findDevice<ParallelDevice>(name);
    }
    Frame* findFrame(const std::string& name){
        return getRobWorkStudio()->getWorkCell()->findFrame(name);
    }

    MovableFrame* findMovableFrame(const std::string& name){
        return getRobWorkStudio()->getWorkCell()->findFrame<MovableFrame>(name);
    }

    FixedFrame* findFixedFrame(const std::string& name){
        return getRobWorkStudio()->getWorkCell()->findFrame<FixedFrame>(name);
    }

    void moveTo(MovableFrame* mframe, Transform3D wTframe ){
        State state = getState();
        mframe->moveTo(wTframe, state);
        setState(state);
    }

    void moveTo(Frame* frame, MovableFrame* mframe, Transform3D wTtcp ){
        State state = getState();
        Transform3D tcpTbase = rw::kinematics::Kinematics::frameTframe(frame, mframe, state);
        Transform3D wTbase_target = wTtcp * tcpTbase;
        mframe->moveTo(wTbase_target, state);
        setState(state);
    }

    void moveTo(const std::string& fname, const std::string& mname, Transform3D wTframe ){
        Frame *fframe = findFrame(fname);
        MovableFrame *mframe = findMovableFrame(mname);
        moveTo(fframe, mframe, wTframe);
    }
%}
/*
State& getState();
void setState(State& state);
rw::common::Ptr<Device> findDevice(const std::string& name);
rw::common::Ptr<JointDevice> findJointDevice(const std::string& name);
rw::common::Ptr<SerialDevice> findSerialDevice(const std::string& name);
rw::common::Ptr<TreeDevice> findTreeDevice(const std::string& name);
rw::common::Ptr<ParallelDevice> findParallelDevice(const std::string& name);
*/

#ifdef SWIGLUA
%luacode {

    function getDevice(name)
      local wc = rws.getRobWorkStudio():getWorkCell()
      return wc:findDevice(name)
    end

    function findFrame(name)
      local wc = rws.getRobWorkStudio():getWorkCell()
      return wc:findFrame(name)
    end

    function getFrame(name)
      local wc = rws.getRobWorkStudio():getWorkCell()
      return wc:findFrame(name)
    end

    function getQ(dev)
      local state = rws.getState()
      return dev:getQ(state)
    end

    function setQ(dev, q)
      local state = rws.getState()
      dev:setQ(q, state)
      setState(state)
    end

    function setTransform(frame, trans)
      local state = rws.getState()
      frame:setTransform(trans, state)
      setState(state)
    end


    function wTf(frame)
      return rw.worldTframe(frame, getState() )
    end

    function fTf(frameA, frameB)
      return rw.worldTframe(frameA, frameB, getState() )
    end
}
#endif

