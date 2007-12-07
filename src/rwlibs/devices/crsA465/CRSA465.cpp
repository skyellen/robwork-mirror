/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "CRSA465.hpp"

#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>

#include <rw/common/macros.hpp>

using namespace rwlibs::devices;

using namespace rw::math;

CRSA465::CRSA465():
    _connected(false)
{}
    
CRSA465::~CRSA465()
{
    if (_connected) {
        disconnect();    
    }
}
    
bool CRSA465::connect(const std::string& port)
{
    std::cout<<"PortName = "<<port<<std::endl;
    _connected = _aci.OpenSession(port, 1, 1);
    
    return _connected;
}

void CRSA465::disconnect()
{
    _aci.CloseSession();
    _connected = false;
}

bool CRSA465::isConnected() const
{
    return _connected;
}

void CRSA465::move(rw::math::Transform3D<>& baseTtool)
{
    Vector3D<> pos = baseTtool.P();
    RPY<> rpy(baseTtool.R());

    std::cout<<"Location = "<<pos<<" and "<<rpy<<std::endl;
    if (_aci.SetLocation("AA", (float)pos(0), (float)pos(1), (float)pos(2), (float)rpy(2), (float)rpy(1), (float)rpy(0))) {
        if (!_aci.Move("AA")) {
            RW_THROW("Failed to move the robot");
        }
    }
    else {                      
        RW_THROW("Failed to deliver Cartesian pose to the robot");
    }
}

Transform3D<> CRSA465::getBaseTtool()
{
    float roll, pitch, yaw;
    float x, y, z;

    if (_aci.Actual(x, y, z, roll, pitch, yaw)) {
        return Transform3D<>(Vector3D<>(x,y,z), RPY<>(roll, pitch, yaw));
    }
    else
        RW_THROW("Failed to read joint configuration");
}

void CRSA465::move(rw::math::Q& q)
{
    if (q.size() != 6) {
        RW_THROW("Configuration for CRSA465 must be exactly 6 long");
    }
    if (!_aci.Ma((float)q(0), (float)q(1), (float)q(2), (float)q(3), (float)q(4), (float)q(5))) {
        RW_THROW("Failed to move joints");
    }
}

Q CRSA465::getQ()
{
    boost::numeric::ublas::vector<float> q;
    if (_aci.GetJointConfig(q(0),q(1),q(2),q(3),q(4),q(5))) {
        Q qres(6);
        for (int i = 0; i<6; i++)
            qres(i) = (float)q(i);
        return qres;
    }
    else
        RW_THROW("Failed to read joint configuration");
}

void CRSA465::grip(float pos)
{
    if (!_aci.Grip(pos))
        RW_THROW("Failed to move gripper");
}
