
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>

#include <vector>

#include <rw/geometry/STLFile.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/GeometryFactory.hpp>

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>

#include <rwsim/dynamics/ContactManifold.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;

void writeHeader(std::ostream& out){
    out << "<?xml version=\"1.0\"?> \n"
        << "<grasps xmlns=\"http://iis.uibk.ac.at/ns/grasping\">\n";

}

void writeGripper(std::ostream& out, std::string name, Q params ){
    out << "<gripper type=\""<< name << "\">\n"
           " <params>";
    for(size_t i=0;i<params.size();i++) out << params[i] << " ";
    out << " </params>\n"
        << "</gripper>\n";
}

void writePose(std::ostream& out, const Transform3D<>& pose){
    Quaternion<> quat(pose.R());
    out << "<pose>\n"
            << " <position domain=\"R3\">\n"
            << "  <euclidean>" << pose.P()[0] << " " << pose.P()[1] << " " << pose.P()[2] << "</euclidean>\n"
            << " </position>\n"
            << " <orientation domain=\"SO3\">\n"
            << "  <quaternion format=\"wxyz\">" << quat(0) << " " << quat(1) << " "<< quat(2) << "</quaternion>\n"
            << "  <rotmatrix>"
            << pose.R()(0,0) << " " << pose.R()(1,0) << " " << pose.R()(2,0) << "   "
            << pose.R()(0,1) << " " << pose.R()(1,1) << " " << pose.R()(2,1) << "   "
            << pose.R()(0,2) << " " << pose.R()(1,2) << " " << pose.R()(2,2) << "   "
            << "  </rotmatrix>\n"
            << " </orientation>\n"
            << "</pose>\n";
}

int main(int argc, char** argv)
{
    int directoryType = 0, sceneTypeStart = 1, mergeFiles = 0;
    string taskFile = "", outfile = "";
    if (argc > 1) taskFile = std::string(argv[1]);
    if (argc > 2) outfile = std::string(argv[2]);

    std::cout << "\n\n\n";
    std::cout << "*********  Simulate Multi Task Program ****************" << std::endl;
    std::cout << "\n\n\n";


    XMLTaskLoader loader;
    loader.load( taskFile );

    rwlibs::task::CartesianTask::Ptr task = loader.getCartesianTask();
    Transform3D<> wTe_n = task->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
    Transform3D<> wTe_home = task->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
    Vector3D<> approach = task->getPropertyMap().get<Vector3D<> >("Approach", Vector3D<>(0,0,0));
    Transform3D<> approachDef = Transform3D<>( approach, Rotation3D<>::identity());
    Q openQ = task->getPropertyMap().get<Q>("OpenQ");
    Q closeQ = task->getPropertyMap().get<Q>("CloseQ");

    std::ofstream fstr(outfile.c_str());
    writeHeader(fstr);

    BOOST_FOREACH( rwlibs::task::CartesianTarget::Ptr target, task->getTargets() ){
        Transform3D<> trans = wTe_n * target->get();


        fstr << "<grasp>\n";
        writeGripper(fstr, "Schunk Hand", Q::zero(2));
        writePose(fstr, trans);
        //writeOutcome(fstr, );
        fstr << "</grasp>\n";

    }
    return (0);
}
