%module rws

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rwslibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>
using namespace rwlibs::swig;
using namespace rws::swig;

using rw::trajectory::Interpolator;
using rw::trajectory::Blend;
using rw::trajectory::Path;
using rw::trajectory::Timed;
using rw::trajectory::Trajectory;
using rw::trajectory::InterpolatorTrajectory;
using rw::pathplanning::PathPlanner;

%}

%import <rwlibs/swig/rw.i> 

%pragma(java) jniclassclassmodifiers="class"

/********************************************
 * General utility functions
 ********************************************/

rw::common::Ptr<RobWorkStudio> getRobWorkStudioInstance();
rw::common::Ptr<RobWorkStudio> getRobWorkStudioInstance(const std::string& args);

RobWorkStudio* getRobWorkStudio();

void setRobWorkStudio(RobWorkStudio* rwstudio);

const State& getState();
void setState(State& state);
rw::common::Ptr<Device> findDevice(const std::string& name);
rw::common::Ptr<JointDevice> findJointDevice(const std::string& name);
rw::common::Ptr<SerialDevice> findSerialDevice(const std::string& name);
rw::common::Ptr<TreeDevice> findTreeDevice(const std::string& name);
rw::common::Ptr<ParallelDevice> findParallelDevice(const std::string& name);
Frame* findFrame(const std::string& name);

MovableFrame* findMovableFrame(const std::string& name);

 FixedFrame* findFixedFrame(const std::string& name);

 void moveTo(MovableFrame* mframe, Transform3D wTframe );

 void moveTo(Frame* frame, MovableFrame* mframe, Transform3D wTtcp );

 void moveTo(const std::string& fname, const std::string& mname, Transform3D wTframe );

 Q getQ(rw::common::Ptr<Device> dev);
 void setQ(rw::common::Ptr<Device> dev, Q);

 void setTransform(Frame* mframe, Transform3D wTframe );

  Transform3D wTf(Frame* frame);
 Transform3D fTf(Frame* frame,Frame* frame);
  Transform3D wTf(const std::string& frame);
 Transform3D fTf(const std::string& frame,const std::string& frame);


/*
State& getState();
void setState(State& state);
rw::common::Ptr<Device> findDevice(const std::string& name);
rw::common::Ptr<JointDevice> findJointDevice(const std::string& name);
rw::common::Ptr<SerialDevice> findSerialDevice(const std::string& name);
rw::common::Ptr<TreeDevice> findTreeDevice(const std::string& name);
rw::common::Ptr<ParallelDevice> findParallelDevice(const std::string& name);
*/

/********************************************
 * Qt
 ********************************************/

%nodefaultctor QString;
class QString
{
}; 

%nodefaultctor QWidget;
class QWidget 
{
};

/********************************************
 * RWS
 ********************************************/

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

%template (RWStudioView3DPtr) rw::common::Ptr<RWStudioView3D>;

class RobWorkStudio { 
public:
    RobWorkStudio(const PropertyMap& map);

    void openFile(const std::string& filename);

    PropertyMap& getPropertyMap();


    rw::common::Ptr<WorkCell> getWorkCell();

    rw::common::Ptr<CollisionDetector> getCollisionDetector();

    rw::common::Ptr<WorkCellScene> getWorkCellScene();

    rw::common::Ptr<RWStudioView3D> getView();

    const Path<Timed<State> >& getTimedStatePath();

    //void updateAndRepaint();
    //void setState(const State& state);
    //void setTimedStatePath(const PathTimedState& path);
    void postState(const State& state);
    void postUpdateAndRepaint();
    void postSaveViewGL(const std::string& str);
    void postTimedStatePath(const Path<Timed<State> >& path);
    void postWorkCell(rw::common::Ptr<WorkCell> workcell);
    void postOpenWorkCell(const std::string& str);
    void postExit();

    const State& getState();


    %extend {
        void setTimedStatePath(rw::common::Ptr<Path<Timed<State> > > path){
            $self->postTimedStatePath(*path);
        }

        void setState(const State& state){
            $self->postState(state);
        }

        void setWorkCell(rw::common::Ptr<WorkCell> workcell){
            $self->postWorkCell(workcell);
        }

        void openWorkCell(const std::string& file){
            $self->postOpenWorkCell(file);
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
        	RW_WARN("fireGenericEvent(const std::string& str) IS DEPRECATED. PLEASE USE send and wait. These use the GenericAnyEvent");
        	
            $self->postGenericEvent(str);
        }

        void send(const std::string& id){
        	boost::any data;
        	std::cout << "Sending event: " << id << std::endl;
            $self->postGenericAnyEvent(id, data);
        }
        void send(const std::string& id, const std::string& val){
            $self->postGenericAnyEvent(id, val);
        }

        void send(const std::string& id, double val){
            $self->postGenericAnyEvent(id, val);
        }

        void send(const std::string& id, Q val){
            $self->postGenericAnyEvent(id, val);
        }

        void send(const std::string& id, const PropertyMap& val){
            $self->postGenericAnyEvent(id, val);
        }

        int wait(const std::string& id){
            try {
            	
                $self->waitForAnyEvent(id);
            } catch ( ... ){
                return 0;
            }
            return 1;
        }

        int wait(const std::string& id, double timeout){
            try {
                $self->waitForAnyEvent(id, timeout);
            } catch ( ... ){ return 0; }
            return 1;
        }

        int wait(const std::string& id, Q& result, double timeout=-1.0){
            try {
                boost::any data = $self->waitForAnyEvent(id, timeout);
                Q* q = boost::any_cast<Q>(&data);
                if(q!=NULL)
                    result = *q;
            } catch ( ... ){ return 0;}
            return 1;
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

%template (RobWorkStudioPtr) rw::common::Ptr<RobWorkStudio>;

/********************************************
 * RWSLIBS GTASK
 ********************************************/
 
/********************************************
 * RWSLIBS JOG
 ********************************************/
 
/********************************************
 * RWSLIBS LOG
 ********************************************/

/********************************************
 * RWSLIBS LUA
 ********************************************/

/********************************************
 * RWSLIBS LUAEDITOR
 ********************************************/

/********************************************
 * RWSLIBS PLANNING
 ********************************************/

/********************************************
 * RWSLIBS PLAYBACK
 ********************************************/

/********************************************
 * RWSLIBS PROPERTYVIEW
 ********************************************/

/********************************************
 * RWSLIBS RWSTUDIOAPP
 ********************************************/
namespace rws{
    class RobWorkStudioApp
     {
     public:
        RobWorkStudioApp(const std::string& args);
        ~RobWorkStudioApp();

        void start();
        void run();

        bool isRunning();
		RobWorkStudio * getRobWorkStudio();
	
     };
}

/********************************************
 * RWSLIBS SENSORS
 ********************************************/

/********************************************
 * RWSLIBS SWIG
 ********************************************/
 
/********************************************
 * RWSLIBS TREEVIEW
 ********************************************/

/********************************************
 * LUA functions
 ********************************************/
 
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
      rws.setState(state)
    end

    function setTransform(frame, trans)
      local state = rws.getState()
      frame:setTransform(trans, state)
      setState(state)
    end
}
#endif

