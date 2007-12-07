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

#include "QPControllerComponent.hpp"

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <float.h>

using namespace boost::numeric::ublas;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

using namespace rwlibs::components;

QPControllerComponent::QPControllerComponent(
    const std::string& name,
    double h,
    const State& state,
    DeviceModel* device)
    :
    RTT::TaskContext(name),
    _vsIn("ToolVelDesired"),
    _qIn("JointCurrent"),
    _qdotIn("JointVelCurrent"),
    _qdotOut("JointVelDesired", Q(Q::ZeroBase(device->getDOF()))),
    _qpcontroller(h, state, device),
    _state(state)
{
     ports()->addPort(&_vsIn);
     ports()->addPort(&_qIn);
     ports()->addPort(&_qdotIn);
     ports()->addPort(&_qdotOut);
}

QPControllerComponent::~QPControllerComponent() {
}

bool QPControllerComponent::startup() {
    _qdotOut.Set(Q(Q::ZeroBase(_device->getDOF())));
    return true;
}


void QPControllerComponent::update() {
    Q dq = _qpcontroller.solve(_qIn.Get(), _qdotIn.Get(), _vsIn.Get());
    _qdotOut.Set(dq);
}


void QPControllerComponent::shutdown() {
}

