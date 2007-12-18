#include "TreeDevice.hpp"

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
#include <rw/models/Accessor.hpp>

#include <rw/iksolvers/CCDSolver.hpp>

using namespace rw::iksolvers;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::common;

//----------------------------------------------------------------------
// SerialDevice

namespace
{
    bool isActiveJoint(const Frame& frame)
    {
        return Accessor::ActiveJoint().has(frame);
    }

    std::vector<Joint*> getActiveJoints(const std::vector<Frame*>& frames)
    {
        // But how do we know that isActiveJoint() implies Joint*? Why don't we
        // use a dynamic cast here for safety?

        std::vector<Joint*> active;

        typedef std::vector<Frame*>::const_iterator I;
        for (I p = frames.begin(); p != frames.end(); ++p) {
            Frame* frame = *p;
            if (isActiveJoint(*frame))
                active.push_back((Joint*)frame);
        }

        return active;
    }

    // The chain of frames connecting \b first to \b last.
    // \b last is included in the list, but \b first is excluded.
    std::vector<Frame*> getKinematicTree(
        Frame* first,
        std::vector<Frame*>& last,
        const State& state)
    {
        RW_ASSERT(first);
        //RW_ASSERT(last);

        typedef std::vector<Frame*> V;
        
        
        std::map<Frame*, Frame*> fToParent;
        fToParent[first] = NULL;
        V kinematicChain;
        kinematicChain.push_back(first);
        
        for(size_t i=0; i<last.size(); i++){
            V reverseChain;
            
            //std::map<Frame*, Frame*>::iterator iter;
                        
            for(Frame* frame = last[i]; 
                fToParent.find(frame) == fToParent.end();
                frame = frame->getParent(state) ){
                
                if (!frame)
                    RW_THROW(
                        "Frame "
                        << StringUtil::Quote(first->getName())
                        << " is not on the parent chain of "
                        << StringUtil::Quote(last[i]->getName()));
                
                reverseChain.push_back(frame);
            }
            std::copy(
                reverseChain.rbegin(),
                reverseChain.rend(),
                std::back_inserter(kinematicChain));
        }
        return kinematicChain;
    }

    // The chain of frames connecting \b first to \b last.
    // \b last is included in the list, but \b first is excluded.
    std::vector<boost::shared_ptr<BasicDeviceJacobian> > getBasicJDevices(
        BasicDevice& bdev,
        std::vector<Frame*>& last,
        const State& state)
    {        
        std::vector< boost::shared_ptr<BasicDeviceJacobian> > devices;
        for(size_t i=0; i<last.size(); i++){
            boost::shared_ptr<BasicDeviceJacobian> 
                devjac( new BasicDeviceJacobian(bdev, last[i], state));
            devices.push_back( devjac );
        }
        return devices;
    }

    // The chain of frames connecting \b first to \b last.
    // \b last is included in the list, but \b first is excluded.
    boost::shared_ptr<BasicDeviceJacobian> getBasicJDevice(
        BasicDevice& bdev,
        std::vector<Frame*>& last,
        const State& state)
    {        
        boost::shared_ptr<BasicDeviceJacobian> 
            devjac( new BasicDeviceJacobian(bdev, last, state));
        return devjac;
    }    
    
}

TreeDevice::TreeDevice(Frame* base,
                       std::vector< Frame* > last,
                       const std::string& name, 
                       const State& state):
                           Device(name),
                           _base(base),
                           _end(last),
                           _kinematicChain(getKinematicTree(_base, _end, state)),
                           _activeJoints(getActiveJoints(_kinematicChain)),
                           _basicDevice(_activeJoints),
                           _dj(getBasicJDevices(_basicDevice, _end, state)),
                           _djmulti(getBasicJDevice(_basicDevice,_end,state))

{
    // run through the tree structure
    
}

TreeDevice::~TreeDevice()
{
}


//----------------------------------------------------------------------
// Jacobians for TreeDevice

Jacobian TreeDevice::baseJend(const State& state) const
{
    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * _dj.front()->get(fk);
}

Jacobian TreeDevice::baseJframe(const Frame* frame, const State& state) const
{
    BasicDeviceJacobian dj(_basicDevice, frame, state);

    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * dj.get(fk);
}

Jacobian TreeDevice::baseJends(const State& state) const {
    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * _djmulti->get(fk);    
}

boost::shared_ptr<BasicDeviceJacobian> TreeDevice::baseJframes(const std::vector<Frame*>& frames, 
                                            const State& state) const 
{
    
    BasicDeviceJacobian *devjac = new BasicDeviceJacobian(_basicDevice, frames, state);
    return boost::shared_ptr<BasicDeviceJacobian>(devjac);
    /*    
    FKTable fk(state);
    const Transform3D<>& start = fk.get(*getBase());
    return inverse(start.R()) * dj.get(fk);
*/    
}

//----------------------------------------------------------------------
// The rest is just forwarding to BasicDevice.

std::pair<Q, Q> TreeDevice::getBounds() const
{
    return _basicDevice.getBounds();
}

void TreeDevice::setBounds(const std::pair<Q, Q>& bounds)
{
    return _basicDevice.setBounds( bounds);
}

void TreeDevice::setQ(const Q& q, State& state) const
{
    return _basicDevice.setQ( q, state);
}

Q TreeDevice::getQ(const State& state) const
{
    return _basicDevice.getQ( state);
}

Q TreeDevice::getVelocityLimits() const
{
    return _basicDevice.getVelocityLimits();
}

void TreeDevice::setVelocityLimits(const Q& vellimits)
{
    return _basicDevice.setVelocityLimits( vellimits);
}

Q TreeDevice::getAccelerationLimits() const
{
    return _basicDevice.getAccelerationLimits();
}

void TreeDevice::setAccelerationLimits(const Q& q)
{
    return _basicDevice.setAccelerationLimits(q);
}

