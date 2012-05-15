
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


int main(int argc, char** argv)
{
    Vector3D<> pos(-0.075, 0, 0.10139);
    Rotation3D<> rot(-0.960294, 0.0857174, -0.265497,
                     -0.000391172, 0.951217, 0.308521,
                     0.278991, 0.296375, -0.913414);

    rwlibs::task::CartesianTask tasks;
    // first set up the configuration
    Vector3D<> d(0,0,-0.10);
    Transform3D<> wTe_n(pos, rot);
    Transform3D<> wTe_home(pos+inverse(rot)*d, rot);
    Q openQ(1,0.0);
    Q closeQ(1,1.0);

    //wTe_n = Transform3D<>::identity();
    //wTe_home = Transform3D<>::identity();
    tasks.getPropertyMap().set<std::string>("GripperName", "GS20");
    tasks.getPropertyMap().set<std::string>("ControllerName", "GraspController");
    tasks.getPropertyMap().set<std::string>("TCP", "TCPGS20");

    tasks.getPropertyMap().set<Transform3D<> >("Nominal", wTe_n);
    tasks.getPropertyMap().set<Transform3D<> >("Home", wTe_home);
    tasks.getPropertyMap().set<Vector3D<> >("Approach", Vector3D<>(0,0,0.02));
    tasks.getPropertyMap().set<Q>("OpenQ", openQ);
    tasks.getPropertyMap().set<Q>("CloseQ", closeQ);

    // next generate the targets
    /*
    while(true){
        double x = Math::ran(-0.2,0.2);
        double y = Math::ran(-0.2,0.2);
        double z = Math::ran(-0.2,0.2);
        Rotation3D<> rot = Math::ranRotation3D();
        Transform3D<> rt3d( Vector3D<>(x,y,z), rot);

        // TODO check collision between cylinder and object
    }
    */

    for(int i=0; i<10; i++){
        for(int j=0; j<10; j++){
            Transform3D<> target = Transform3D<>::identity();
            target.P()[0] -= i*0.01;
            target.P()[1] -= j*0.01;
            CartesianTarget::Ptr ctarget = ownedPtr( new CartesianTarget(target) );
            tasks.addTarget( ctarget );
        }
    }

    try {
        XMLTaskSaver saver;
        saver.save(&tasks, "SuctionCupTaskFile.xml");
    } catch (const Exception& exp) {
       // QMessageBox::information(this, "Task Execution Widget", "Unable to save tasks");
    }


    return (0);
}
