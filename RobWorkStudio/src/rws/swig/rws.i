%module rws

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rws/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>
using namespace rwlibs::swig;
using namespace rws::swig;
%}

%import <rwlibs/swig/rw.i>


%template (RobWorkStudioPtr) rw::common::Ptr<RobWorkStudio>;
%template (RWStudioView3DPtr) rw::common::Ptr<RWStudioView3D>;

rw::common::Ptr<RobWorkStudio> getRobWorkStudioInstance();
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
    void postWorkCell(rw::common::Ptr<WorkCell> workcell);
    void postOpenWorkCell(const std::string& str);
    void postExit();

    const State& getState();


    %extend {
        void setTimedStatePath(const TimedStatePath& path){
            $self->postTimedStatePath(path);
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


////////////////////////////////////////////////////////////////////////////////////
/// General utility functions that ease the use of robworkstudio
///

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

 void moveTo(const std::string& fname, const std::string& mname, rwlibs::swig::Transform3D wTframe );

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

