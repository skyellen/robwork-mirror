

#include "ScriptTypes.hpp"

#include <rws/components/rwstudioapp/RobWorkStudioApp.hpp>

using namespace rws::swig;
using namespace rwlibs::swig;

rw::common::Ptr<rws::swig::RobWorkStudio> rwstudio_internal;

rws::swig::RobWorkStudio* rws::swig::getRobWorkStudio(){
    if(rwstudio_internal==NULL)
        rwstudio_internal = getRobWorkStudioInstance("");
    return rwstudio_internal.get();
}

void rws::swig::setRobWorkStudio(rws::swig::RobWorkStudio* rwstudio){
    rwstudio_internal = rwstudio;
}

const State& rws::swig::getState(){ return getRobWorkStudio()->getState(); }
void rws::swig::setState(State& state){ return getRobWorkStudio()->postState(state); }
rw::common::Ptr<Device> rws::swig::findDevice(const std::string& name){
    return getRobWorkStudio()->getWorkCell()->findDevice(name);
}
rw::common::Ptr<JointDevice> rws::swig::findJointDevice(const std::string& name){
    return getRobWorkStudio()->getWorkCell()->findDevice<JointDevice>(name);
}
rw::common::Ptr<SerialDevice> rws::swig::findSerialDevice(const std::string& name){
    return getRobWorkStudio()->getWorkCell()->findDevice<SerialDevice>(name);
}
rw::common::Ptr<TreeDevice> rws::swig::findTreeDevice(const std::string& name){
    return getRobWorkStudio()->getWorkCell()->findDevice<TreeDevice>(name);
}
rw::common::Ptr<ParallelDevice> rws::swig::findParallelDevice(const std::string& name){
    return getRobWorkStudio()->getWorkCell()->findDevice<ParallelDevice>(name);
}
Frame* rws::swig::findFrame(const std::string& name){
    return getRobWorkStudio()->getWorkCell()->findFrame(name);
}

MovableFrame* rws::swig::findMovableFrame(const std::string& name){
    return getRobWorkStudio()->getWorkCell()->findFrame<MovableFrame>(name);
}

FixedFrame* rws::swig::findFixedFrame(const std::string& name){
    return getRobWorkStudio()->getWorkCell()->findFrame<FixedFrame>(name);
}

void rws::swig::moveTo(MovableFrame* mframe, Transform3D wTframe ){
    State state = getState();
    mframe->moveTo(wTframe, state);
    setState(state);
}

void rws::swig::moveTo(Frame* frame, MovableFrame* mframe, Transform3D wTtcp ){
    State state = getState();
    Transform3D tcpTbase = rw::kinematics::Kinematics::frameTframe(frame, mframe, state);
    Transform3D wTbase_target = wTtcp * tcpTbase;
    mframe->moveTo(wTbase_target, state);
    setState(state);
}

void rws::swig::moveTo(const std::string& fname, const std::string& mname, Transform3D wTframe ){
    Frame *fframe = findFrame(fname);
    MovableFrame *mframe = findMovableFrame(mname);
    moveTo(fframe, mframe, wTframe);
}


static rws::RobWorkStudioApp *robApp = NULL;

rw::common::Ptr<RobWorkStudio> rws::swig::getRobWorkStudioInstance(){
    return getRobWorkStudioInstance("");
}

rw::common::Ptr<RobWorkStudio> rws::swig::getRobWorkStudioInstance(const std::string& args){

    // create a thread that start QApplication and
    if(robApp==NULL){
        robApp =new RobWorkStudioApp( args );
        robApp->start();
        while(robApp->_rwstudio==NULL){
            rw::common::TimerUtil::sleepMs(100);
        }
        if(rwstudio_internal==NULL)
            rwstudio_internal = robApp->_rwstudio;
    }
    return robApp->_rwstudio;
}

