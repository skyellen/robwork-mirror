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

#include "PA10Component.hpp"

#include <rtt/ExecutionEngine.hpp>

#include <iostream>
#include <float.h>

using namespace RTT;

using namespace rw::models;
using namespace rw::math;
using namespace rwlibs::devices;
using namespace rwlibs::components;

PA10Component::PA10Component(PA10* pa10, double dt) : 
    TaskContext("PA10Component"), 
    _qCurrent("JointCurrent", Q(Q::ZeroBase(7))),
    _qdotCurrent("JointVelCurrent", Q(Q::ZeroBase(7))),
    _qdotTarget("JointVelDesired"),
    _errorPort("SafetyError", true),
    _pa10(pa10),
    _initialized(false),
    _dt(dt)
{
    ports()->addPort(&_qCurrent);
    ports()->addPort(&_qdotCurrent);
    ports()->addPort(&_qdotTarget);
    ports()->addPort(&_errorPort);
}

bool PA10Component::startup()
{
    _qCurrent.Set(Q(Q::ZeroBase(7)));
    _qdotCurrent.Set(Q(Q::ZeroBase(7)));

    bool success = false;
    Q q = _pa10->start(success);

    if (success) {
        _qCurrent.Set(q);       
    } else {
        RW_THROW("Could not initialize PA10 driver");
    }

    return true;
}

void PA10Component::update()
{
    if (!_initialized) {
        _pa10->initializeThread();
        _initialized = true;
    }

    Q qdotTarget = _qdotTarget.Get();
    Q q = _pa10->update(qdotTarget);

    _qCurrent.Set(q);
    _qdotCurrent.Set(qdotTarget);
}

void PA10Component::shutdown()
{
    _pa10->stop();
}
