#include "GraspTask.hpp"

#include <rwlibs/task/loader/XMLTaskSaver.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>

#include <rw/rw.hpp>
USE_ROBWORK_NAMESPACE
using namespace robwork;

namespace {
    void writeOutcome( std::ostream& out, rwlibs::task::CartesianTarget::Ptr target){
        int status = target->getPropertyMap().get<int> ("TestStatus");
        Q quality = target->getPropertyMap().get<Q>("QualityAfterLifting", Q::zero(1));

        out << "    <outcome>\n";
        // if success we write all informal qualities

        if(status==GraspTask::Success || status==GraspTask::ObjectSlipped ){
            for(size_t i=0;i<quality.size();i++){
                out << "     <informal uri=\"rwgq" << i << "\" quality=\""<< quality[i] << "\" />\n";
            }
            if(quality[0]>0.01){ // we use a weak fixed quality rule here
                out << "     <success quality=\"" << quality[0] <<"\" />\n";
            } else {
                out << "     <failure cause=\"OBJECTSLIPPED\" />\n";
            }
        } else {
            out << "     <failure cause=\"";
            switch(status){
            case(GraspTask::ObjectDropped): out << "OBJECTDROPPED"; break;
            case(GraspTask::UnInitialized): out << "UNINITIALIZED"; break;
            case(GraspTask::CollisionInitially): out << "COLLISIONINITIALLY"; break;
            case(GraspTask::ObjectMissed): out << "OBJECTMISSED"; break;
            case(GraspTask::InvKinFailure): out << "INVKINFAILURE"; break;
            case(GraspTask::TimeOut): out << "TIMEOUT"; break;
            case(GraspTask::SimulationFailure): out << "SIMULATIONFAILURE"; break;
            default:
                RW_THROW("Not supposed to go here!");
                break;
            }
            out << "\" />\n";
        }

        out << "    </outcome>\n";
    }

    void writePose(std::ostream& out, const Transform3D<>& pose){
        Quaternion<> quat(pose.R());
        out << "<pose>\n"
                << " <position domain=\"R3\">\n"
                << "  <euclidean>" << pose.P()[0] << " " << pose.P()[1] << " " << pose.P()[2] << "</euclidean>\n"
                << " </position>\n"
                << " <orientation domain=\"SO3\">\n"
                << "  <quaternion format=\"wxyz\">" << quat(3) << " " << quat(0) << " " << quat(1) << " "<< quat(2) << "</quaternion>\n"
                << "  <rotmatrix>"
                << pose.R()(0,0) << " " << pose.R()(0,1) << " " << pose.R()(0,2) << "   "
                << pose.R()(1,0) << " " << pose.R()(1,1) << " " << pose.R()(1,2) << "   "
                << pose.R()(2,0) << " " << pose.R()(2,1) << " " << pose.R()(2,2) << "   "
                << "  </rotmatrix>\n"
                << " </orientation>\n"
                << "</pose>\n";
    }

    void writeUIBK(GraspTask::Ptr gtask, const std::string& outfile){

        rwlibs::task::CartesianTask::Ptr grasptask = gtask->getRootTask();

        std::ofstream fstr(outfile.c_str());
        fstr << "<?xml version=\"1.0\"?> \n"
             << "<experiments xmlns=\"http://iis.uibk.ac.at/ns/grasping\">\n";
        fstr << "<notes> Generated with RobWork, rwsim::simulator::GraspTask </notes>\n";

        BOOST_FOREACH(rwlibs::task::CartesianTask::Ptr task, grasptask->getTasks() ){
            Transform3D<> wTe_n = task->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
            Transform3D<> wTe_home = task->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
            Vector3D<> approach = task->getPropertyMap().get<Vector3D<> >("Approach", Vector3D<>(0,0,0));
            Transform3D<> approachDef = Transform3D<>( approach, Rotation3D<>::identity());
            Q openQ = task->getPropertyMap().get<Q>("OpenQ");
            Q closeQ = task->getPropertyMap().get<Q>("CloseQ");
            std::string objectId = task->getPropertyMap().get<std::string>("Object",std::string("Undefined"));
            std::string gripperId = task->getPropertyMap().get<std::string>("Gripper");
            std::string controllerName = task->getPropertyMap().get<std::string>("ControllerName", "GraspController");
            std::string tcpName = task->getPropertyMap().get<std::string>("TCP");

            fstr << " <experiment>\n";
            fstr << "  <notes></notes>\n";
            fstr << "  <object type=\"" << objectId << "\">\n";
            fstr << "   <notes> </notes>\n";
            fstr << "  </object>\n";
            fstr << "  <gripper type=\""<< gripperId << "\">\n";
            fstr << "   <notes> GraspController:"<<controllerName << " TCP:" << tcpName <<" CloseQ:"<< closeQ << "</notes>\n";
            fstr << "   <params>";
            for(size_t i=0;i<openQ.size();i++)
                fstr << openQ[i] << " ";
            fstr << "   </params>\n"
                 << "  </gripper>\n";
            fstr << "  <grasps>\n";
            fstr << "   <notes>  </notes>\n"; // don't have any notes yet

            // we don't add predictiondef
            BOOST_FOREACH( rwlibs::task::CartesianTarget::Ptr target, task->getTargets() ){
                Transform3D<> trans = wTe_n * target->get();
                fstr << "   <grasp>\n";
                writePose(fstr, trans);
                writeOutcome(fstr, target);
                fstr << "   </grasp>\n";
            }
            fstr << "  </grasps>\n";
            fstr << " </experiment>\n";

        }


        // and end with an experiments tag
        fstr << "</experiments>\n";
    }

}


void GraspTask::saveUIBK(GraspTask::Ptr task, const std::string& name ){
    writeUIBK(task, name);
}

void GraspTask::saveRWTask(GraspTask::Ptr task, const std::string& name ){
    std::ofstream outfile(name.c_str());
    try {
        XMLTaskSaver saver;
        saver.save(task->getRootTask(), outfile );
    } catch (const Exception& exp) {
        RW_THROW("Unable to save task: " << exp.what());
    }
    outfile.close();
}

void GraspTask::saveText(GraspTask::Ptr task, const std::string& name ){

}
