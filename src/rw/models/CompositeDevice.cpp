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

#include "CompositeDevice.hpp"
#include "Accessor.hpp"
#include "Joint.hpp"
#include "RevoluteJoint.hpp"
#include "PrismaticJoint.hpp"
#include "JacobianUtil.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rw/invkin/CCDSolver.hpp>

using namespace boost::numeric;

using namespace rw::invkin;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::common;

//----------------------------------------------------------------------
// CompositeDevice


CompositeDevice::CompositeDevice(
                                 Frame* base,
                                 std::vector<Device*> models,
                                 Frame* end,
                                 const std::string& name,
                                 const State& state):
    Device(name),
    _devices(models),
    _base(base),
    _end(end)
{}

CompositeDevice::~CompositeDevice()
{}

//----------------------------------------------------------------------
// Jacobians for CompositeDevice

Jacobian CompositeDevice::baseJend(const State& state) const
{
    /*FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * _dj.get(fk);
    */
    return Jacobian(6,6);
}

Jacobian CompositeDevice::baseJframe(const Frame* frame, const State& state) const
{
    /*BasicDeviceJacobian dj(_basicDevice, frame, state);

    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * dj.get(fk);
*/  
    return Jacobian(6,6);
}

boost::shared_ptr<BasicDeviceJacobian> 
CompositeDevice::baseJframes(const std::vector<Frame*>& frames,
                            const State& state) const 
{   
    /*
    BasicDeviceJacobian *devjac = new BasicDeviceJacobian(_basicDevice, frames, state);
    return boost::shared_ptr<BasicDeviceJacobian>(devjac);
     */
    //return boost::shared_ptr<BasicDeviceJacobian>(NULL);    
}

//----------------------------------------------------------------------
// The rest is just forwarding to BasicDevice.

std::pair<Q, Q> CompositeDevice::getBounds() const
{
    size_t nrOfDof(getDOF()),offset(0);
    ublas::vector<double> qlow(nrOfDof), qhigh(nrOfDof);
    for(size_t i=0; i<_devices.size(); i++){
        std::pair<Q,Q> bounds = _devices[i]->getBounds();
        for(size_t j=0; j<bounds.first.size(); j++){
            qlow(j+offset) = ( bounds.first(j) );
            qhigh(j+offset) = ( bounds.second(j) );
        }
        offset += _devices[i]->getDOF();
    }
    return std::pair<Q,Q>(Q(qlow),Q(qhigh));
}

void CompositeDevice::setBounds(const std::pair<Q, Q>& bounds)
{
    
}

void CompositeDevice::setQ(const Q& q, State& state) const 
{
    //std::cout << "setQ" << std::endl;
    size_t offset(0);
    for(size_t i=0; i<_devices.size(); i++){
        size_t devDof = _devices[i]->getDOF();
        Q qdev(devDof);
        for(size_t j=0; j<devDof; j++){
            qdev(j) = q(offset+j);
        }
        _devices[i]->setQ(qdev, state);
        offset += devDof;
    }
}

Q CompositeDevice::getQ(const State& state) const
{

    //std::cout << "getQ" << std::endl;

    Q q(getDOF());
    size_t offset(0);
    for(size_t i=0; i<_devices.size(); i++){
        Q qdev = _devices[i]->getQ(state);
        for(size_t j=0; j<qdev.size(); j++){
            q(j+offset) = qdev(j);
        }
        offset += _devices[i]->getDOF();
    }
    return q;
}

Q CompositeDevice::getVelocityLimits() const
{
    Q q(getDOF());
    size_t offset(0);
    for(size_t i=0; i<_devices.size(); i++){
        Q qdev = _devices[i]->getVelocityLimits();
        for(size_t j=0; j<qdev.size(); j++){
            q(j+offset) = qdev(j);
        }
        offset += _devices[i]->getDOF();
    }
    return q;
}

void CompositeDevice::setVelocityLimits(const Q& vellimits)
{
    size_t offset(0);
    for(size_t i=0; i<_devices.size(); i++){
        size_t devDof = _devices[i]->getDOF();
        Q qdev(devDof);
        for(size_t j=0; j<devDof; j++){
            qdev(j) = vellimits(offset+j);
        }
        _devices[i]->setVelocityLimits(qdev);
        offset += devDof;
    }
}

Q CompositeDevice::getAccelerationLimits() const
{
    Q q(getDOF());
    size_t offset(0);
    for(size_t i=0; i<_devices.size(); i++){
        Q qdev = _devices[i]->getAccelerationLimits();
        for(size_t j=0; j<qdev.size(); j++){
            q(j+offset) = qdev(j);
        }
        offset += _devices[i]->getDOF();
    }
    return q;

}

void CompositeDevice::setAccelerationLimits(const Q& q)
{
    size_t offset(0);
    for(size_t i=0; i<_devices.size(); i++){
        size_t devDof = _devices[i]->getDOF();
        Q qdev(devDof);
        for(size_t j=0; j<devDof; j++){
            qdev(j) = q(offset+j);
        }
        _devices[i]->setAccelerationLimits(qdev);
        offset += devDof;
    }
}
