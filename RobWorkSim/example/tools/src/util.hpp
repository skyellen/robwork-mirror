#ifndef UTILS_TOOLS_HPP
#define UTILS_TOOLS_HPP

#include <vector>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rwlibs/algorithms/kdtree/KDTree.hpp>
#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>

typedef rwlibs::algorithms::KDTreeQ<std::pair<rwlibs::task::GraspSubTask*,rwlibs::task::GraspTarget*> > GTaskNNSearch;

void moveFrameW(const rw::math::Transform3D<>& wTtcp, rw::kinematics::Frame *tcp, rw::kinematics::MovableFrame* base, rw::kinematics::State& state){
    using namespace rw::math;
    using namespace rw::kinematics;

    Transform3D<> tcpTbase = Kinematics::frameTframe(tcp, base, state);
    Transform3D<> wTbase_target = wTtcp * tcpTbase;
    //base->setTransform(wTbase_target, state);
    base->moveTo(wTbase_target, state);
    //std::cout << wTbase_target << std::endl;
    //std::cout << Kinematics::worldTframe(base, state) << std::endl;
}


const rw::math::Q normalize(const rw::math::Q& v)
{
    double length = v.norm2();
    if (length != 0)
        return v/length;
    else
        return rw::math::Q::zero(v.size());
}



GTaskNNSearch*
buildKDTree_pos_zaxis(rwlibs::task::GraspTask::Ptr gtask, std::vector<rwlibs::algorithms::KDTreeQ<std::pair<rwlibs::task::GraspSubTask*,rwlibs::task::GraspTarget*> >::KDNode>& simnodes) {
    using namespace rw::math;
    using namespace rwlibs::task;
    using namespace rwlibs::algorithms;
    typedef KDTreeQ<std::pair<GraspSubTask*,GraspTarget*> > NNSearch;
    // we need the simulated grasps to attach a quality to the experiments
    // first we build a NN-structure for efficient nearest neighbor search
    BOOST_FOREACH(GraspSubTask& stask, gtask->getSubTasks()){
        BOOST_FOREACH(GraspTarget& target,stask.targets ){
            Transform3D<> t3d = target.getResult()->objectTtcpLift;
            //Transform3D<> t3d = target.pose;
            Vector3D<> p = t3d.P();
            Vector3D<> n = t3d.R()*Vector3D<>::z();
            Q key(6, p[0], p[1], p[2], n[0], n[1], n[2]);

            simnodes.push_back( NNSearch::KDNode(key, std::pair<GraspSubTask*,GraspTarget*>(&stask,&target)) );
        }
    }
    return NNSearch::buildTree(simnodes);
}


rwlibs::algorithms::KDTreeQ<std::pair<rwlibs::task::GraspSubTask*,rwlibs::task::GraspTarget*> >*
buildKDTree_zaxis(rwlibs::task::GraspTask::Ptr gtask) {
    using namespace rw::math;
    using namespace rwlibs::task;
    using namespace rwlibs::algorithms;
    typedef KDTreeQ<std::pair<GraspSubTask*,GraspTarget*> > NNSearch;
    // we need the simulated grasps to attach a quality to the experiments
    // first we build a NN-structure for efficient nearest neighbor search
    std::vector<NNSearch::KDNode> *simnodes = new std::vector<NNSearch::KDNode>();
    BOOST_FOREACH(GraspSubTask& stask, gtask->getSubTasks()){
        BOOST_FOREACH(GraspTarget& target,stask.targets ){
            Transform3D<> t3d = target.pose;
            Vector3D<> n = t3d.R()*Vector3D<>::z();
            Q key(3, n[0], n[1], n[2]);
            simnodes->push_back( GTaskNNSearch::KDNode(key, std::pair<GraspSubTask*,GraspTarget*>(&stask,&target)) );
        }
    }
    return NNSearch::buildTree(*simnodes);
}

rwlibs::algorithms::KDTreeQ<std::pair<rwlibs::task::GraspSubTask*,rwlibs::task::GraspTarget*> >*
buildKDTree_eaa(rwlibs::task::GraspTask::Ptr gtask) {
    using namespace rw::math;
    using namespace rwlibs::task;
    using namespace rwlibs::algorithms;
    typedef KDTreeQ<std::pair<GraspSubTask*,GraspTarget*> > NNSearch;
    // we need the simulated grasps to attach a quality to the experiments
    // first we build a NN-structure for efficient nearest neighbor search
    std::vector<NNSearch::KDNode> *simnodes = new std::vector<NNSearch::KDNode>();
    BOOST_FOREACH(GraspSubTask& stask, gtask->getSubTasks()){
        BOOST_FOREACH(GraspTarget& target,stask.targets ){
            Transform3D<> t3d = target.pose;

            EAA<> eaa( t3d.R() );
            Vector3D<> n = eaa.axis();
            Q key(4, n[0], n[1], n[2], eaa.angle());
            simnodes->push_back( NNSearch::KDNode(key, std::pair<GraspSubTask*,GraspTarget*>(&stask,&target)) );
        }
    }
    return NNSearch::buildTree(*simnodes);
}

rwlibs::algorithms::KDTreeQ<std::pair<rwlibs::task::GraspSubTask*,rwlibs::task::GraspTarget*> >*
buildKDTree_pos_eaa(rwlibs::task::GraspTask::Ptr gtask, std::vector<rwlibs::algorithms::KDTreeQ<std::pair<rwlibs::task::GraspSubTask*,rwlibs::task::GraspTarget*> >::KDNode*>& simnodes) {
    using namespace rw::math;
    using namespace rwlibs::task;
    using namespace rwlibs::algorithms;
    typedef KDTreeQ<std::pair<GraspSubTask*,GraspTarget*> > NNSearch;
    // we need the simulated grasps to attach a quality to the experiments
    // first we build a NN-structure for efficient nearest neighbor search
    BOOST_FOREACH(GraspSubTask& stask, gtask->getSubTasks()){
        BOOST_FOREACH(GraspTarget& target,stask.targets ){
            Transform3D<> t3d = target.pose;
            Vector3D<> p = t3d.P();
            //Vector3D<> n = t3d.R()*Vector3D<>::z();
            EAA<> eaa( t3d.R() );
            Vector3D<> n = eaa.axis();

            Q key(7, p[0], p[1], p[2], n[0], n[1], n[2], eaa.angle() );
            simnodes.push_back( new NNSearch::KDNode(key, std::pair<GraspSubTask*,GraspTarget*>(&stask,&target)) );
        }
    }
    return NNSearch::buildTree(simnodes);
}

#endif
