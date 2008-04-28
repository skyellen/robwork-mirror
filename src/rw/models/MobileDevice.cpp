#include "MobileDevice.hpp"

#include <rw/math/RPY.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/common/macros.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

namespace
{
    std::vector<Joint*> collectJoints(Joint* joint1, Joint* joint2)
    {
        std::vector<Joint*> result;
        result.push_back(joint1);
        result.push_back(joint2);
        return result;
    }
}

MobileDevice::MobileDevice(
    MovableFrame* base,
    RevoluteJoint* wheel1,
    RevoluteJoint* wheel2,
    State& state,
    const std::string& name)
    :
    Device(name),
    _base(base),
    _wheel1(wheel1),
    _wheel2(wheel2),
    _basicDevice(collectJoints(wheel1, wheel2))
{
    setDevicePose(Transform3D<>::identity(), state);
    _width = fabs(wheel1->getTransform(state).P()(1) - wheel2->getTransform(state).P()(1));
    std::cout<<"Width = "<<_width<<std::endl;
}

MobileDevice::~MobileDevice() {}

void MobileDevice::setDevicePose(const Transform3D<>& transform, State& state)
{
    _base->setTransform(transform, state);
}

void MobileDevice::setQ(const Q& q, State& state) const
{
    const Q qold =  _basicDevice.getQ(state);
    _basicDevice.setQ(q, state);

    Q dq = q-qold;
    std::cout<<"q = "<<q<<std::endl;
    std::cout<<"qold = "<<qold<<std::endl;
    std::cout<<"dq = "<<dq<<std::endl;

    double dx, dy, dtheta;
    if (dq(0) == dq(1)) { //Going straigh ahead?
        dx = dq(0);
        dy = 0;
        dtheta = 0;
    } else {
        double radius = _width/2*(dq(1)+dq(0))/(dq(1)-dq(0));
        double phi = (dq(0)-dq(1))/_width;
        dx = sin(phi)*radius;
        dy = (1-cos(phi))*radius;
        dtheta = phi;
        std::cout<<"radius = "<<radius<<std::endl;

    }
    std::cout<<"dtheta = "<<dtheta<<std::endl;
    std::cout<<"dx= "<<dx<<std::endl;
    std::cout<<"dy = "<<dy<<std::endl;
    Transform3D<> delta(Vector3D<>(dx, dy, 0), RPY<>(dtheta, 0, 0));
    Transform3D<> pre = _base->getTransform(state);
    std::cout<<"pre = "<<pre<<std::endl;
    std::cout<<"post = "<<pre*delta<<std::endl;
    _base->setTransform(pre*delta, state);
}

Q MobileDevice::getQ(const State& state) const
{
    return _basicDevice.getQ(state);
}

std::pair<Q, Q> MobileDevice::getBounds() const
{
    return _basicDevice.getBounds();
}

void MobileDevice::setBounds(const std::pair<Q, Q>& bounds)
{
    _basicDevice.setBounds(bounds);
}

Q MobileDevice::getVelocityLimits() const
{
    return _basicDevice.getVelocityLimits();
}

void MobileDevice::setVelocityLimits(const Q& velLimits)
{
    _basicDevice.setVelocityLimits(velLimits);
}

Q MobileDevice::getAccelerationLimits() const
{
    return _basicDevice.getAccelerationLimits();
}

void MobileDevice::setAccelerationLimits(const Q& accLimits)
{
    _basicDevice.setAccelerationLimits(accLimits);
}

size_t MobileDevice::getDOF() const
{ return 2; }

Frame* MobileDevice::getBase()
{ return _base; }

const Frame* MobileDevice::getBase() const
{ return _base; }

Frame* MobileDevice::getEnd()
{
    if (!_axillaryFrames.empty())
        return _axillaryFrames.back();
    return _base;
}

const Frame* MobileDevice::getEnd() const
{
    if (!_axillaryFrames.empty())
        return _axillaryFrames.back();

    return _base;
}

Jacobian MobileDevice::baseJend(const State& state) const
{
    double dtheta1 = -1/_width;
    double dtheta2 = 1/_width;

    Jacobian jac(Jacobian::ZeroBase(6,2));
    jac(0,0) = 0.5;
    jac(1,0) = 0.5;
    jac(5,0) = dtheta1;
    jac(5,1) = dtheta2;

    return jac;
}

Jacobian MobileDevice::baseJframe(
    const Frame* frame,
    const State& state) const
{
    RW_THROW("Not implemented.");
    return Jacobian(Jacobian::ZeroBase(6,2));
}

Jacobian MobileDevice::baseJframes(
    const std::vector<Frame*>& frames,
    const State& state) const
{
    RW_THROW("Not implemented.");
    return Jacobian(Jacobian::ZeroBase(6,2));
}

boost::shared_ptr<DeviceJacobian> MobileDevice::baseDJframes(
    const std::vector<Frame*>& frames,
    const State& state) const
{
    RW_THROW("Not implemented.");
    typedef boost::shared_ptr<DeviceJacobian> T;
    return T();
}
