#include <rw/common/Log.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Transform3D.hpp>

using rw::common::Log;
using namespace rw::math;

int main(int argc, char** argv) {
    RPY<> rpy(0, 0, 90*Deg2Rad); // 90 degree rotation around x-axis
    Rotation3D<> rot = rpy.toRotation3D(); // create Rotation3D matrix
    EAA<> eaa( rot ); // construct eaa form rotation3d
    Quaternion<> quat( rot ); // construct quaternion from rotation3d

    // there are streaming operators for all math types
    Log::infoLog() << rpy << std::endl;
    Log::infoLog() << rot << std::endl;
    Log::infoLog() << eaa << std::endl;
    Log::infoLog() << quat << std::endl;


    // rotate a vector (0,1,0) 90 degrees around x-axis
    Log::infoLog() << rot*Vector3D<>(0,1,0) << std::endl;
    // transform a vector
    Transform3D<> t1( Vector3D<>(0,0,1), rot);
    Log::infoLog() << t1*Vector3D<>(0,1,0) << std::endl;
    // calcualte the inverse rotation
    Log::infoLog() << inverse( rot ) << std::endl;
    // calculate the inverse transform
    Log::infoLog() << inverse( t1 ) << std::endl;
    // get the rotation and translation part of a transform
    Log::infoLog() << t1.R() << t1.P() << std::endl;


}
