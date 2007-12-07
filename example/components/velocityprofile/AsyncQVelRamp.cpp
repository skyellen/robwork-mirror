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

#include "AsyncQVelRamp.hpp"

using namespace rwlibs::components;
using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;

using namespace boost::numeric;

AsyncQVelRamp::AsyncQVelRamp(
    const std::string& name,
    DeviceModel* device,
    const State& state,
    double dt)
    :
    RTT::TaskContext(name),
    _qTarget("JointTarget"),
    _qCurrent("JointCurrent"),
    _qdotCurrent("JointVelCurrent"),
    _qOut("JointPathTarget", Q(Q::ZeroBase(device->getDOF()))),
        _qdotOut("JointVelDesired", Q(Q::ZeroBase(device->getDOF()))),
        _device(device), _dt(dt),
        _state(state)
{
    ports()->addPort(&_qTarget);
    ports()->addPort(&_qCurrent);
    ports()->addPort(&_qdotOut);
    ports()->addPort(&_qOut);
    ports()->addPort(&_qdotCurrent);
    _acc = _device->getAccelerationLimits();
    _velmax = _device->getVelocityLimits();
}

AsyncQVelRamp::~AsyncQVelRamp()
{}

bool AsyncQVelRamp::startup() {
    RW_ASSERT(_device != NULL);
    _qdotOut.Set(Q(Q::ZeroBase(_device->getDOF())));
    _qOut.Set(Q(Q::ZeroBase(_device->getDOF())));

    return true;
}

void AsyncQVelRamp::update()
{
    Q qtarget = _qTarget.Get();
    Q qcurrent = _qCurrent.Get();
    Q vcurrent = _qdotCurrent.Get();
    Q qdot(_device->getDOF());

    for (size_t i = 0; i<_device->getDOF(); i++) {
        double dist = qtarget(i)-qcurrent(i);
        if (fabs(dist) < 0.01) //round close to zero values to zero
            dist = 0;
        double sign = (dist < 0)?-1:1;
        double vdist = sqrt(fabs(2*dist*_acc(i)))*sign*0.8;
        double vmin = std::max(vcurrent(i)-fabs(_acc(i)*_dt), -_velmax(i));
        double vmax = std::min(vcurrent(i)+fabs(_acc(i)*_dt), _velmax(i));

        qdot(i) = std::min(vmax, std::max(vmin, vdist));
    }
    _qdotOut.Set(qdot);

    Q qdiff = qtarget-qcurrent;
    Q q = qcurrent+qdot*1.0;

    for(size_t j=0; j<qdot.size();j++){
        if( (qdot(j)>0) && (qtarget(j) < q(j)) ){
            q(j) = qtarget(j);
        } else if ( (qdot(j)<0) && (qtarget(j) > q(j)) ) {
            q(j) = qtarget(j);
        }
    }
    _qOut.Set(q);
}

void AsyncQVelRamp::shutdown()
{}
