#include "Conveyor.hpp"

#include <rw/models/BasicDeviceJacobian.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

namespace {
	std::vector<Joint*> constructJointList(Joint* joint) {
		std::vector<Joint*> result;
		result.push_back(joint);
		return result;
	}

}


Conveyor::Conveyor(const std::string& name, FixedJoint* base, const std::vector<ConveyorSegment*>& segments):
	Device(name),
    _segments(segments),
    _base(base),
    _basicDevice(constructJointList(base))    
{
    for (std::vector<ConveyorSegment*>::const_iterator it = _segments.begin(); it != _segments.end(); ++it) {
        _frame2segment[(*it)->getBaseFrame()] = *it;
    }
}

Conveyor::~Conveyor()
{
}

void Conveyor::addItem(ConveyorItem* item, double time, State& state) {
	_segments.front()->addItem(item, ConveyorSegment::START, state);
	_segments.front()->move(item, time, state);
}

void Conveyor::setQ(const Q& q, State& state) const {
    double qcurrent = _basicDevice.getQ(state)(0);
    double delta = q(0)-qcurrent;
    
    //TODO Construct List of all items
    std::vector<ConveyorItem*> items;
    std::vector<ConveyorSegment*>::const_iterator it = _segments.begin();
    for ( ; it != _segments.end(); ++it) {
    	Frame::const_iterator_pair pair = (*it)->getBaseFrame()->getDafChildren(state);
    	for (Frame::const_iterator itframe = pair.first; itframe != pair.second; ++itframe) {
    		if (dynamic_cast<const ConveyorItem*>(&(*itframe)) != NULL)
    			items.push_back((ConveyorItem*)(&(*itframe)));
    	}
    }
    
     
    //TODO move frames
    std::map<rw::kinematics::Frame*, ConveyorSegment*>& map = const_cast< std::map<rw::kinematics::Frame*, ConveyorSegment*>&>(_frame2segment);
//    for (std::vector<const ConveyorItem*>::iterator it = items.begin(); it != items.end(); ++it) {
    for (std::vector<ConveyorItem*>::iterator it = items.begin(); it != items.end(); ++it) {
    	ConveyorSegment* segment = map[(*it)->getParent(state)];
    	segment->move((*it), delta, state);
    }
    
    
    _basicDevice.setQ(q, state);    
}



Q Conveyor::getQ(const State& state) const {
	return _basicDevice.getQ(state);
}


std::pair<Q, Q> Conveyor::getBounds() const {
	return _basicDevice.getBounds();
}


void Conveyor::setBounds(const std::pair<Q, Q>& bounds) {
	_basicDevice.setBounds(bounds);	
}

Q Conveyor::getVelocityLimits() const {
	return _basicDevice.getVelocityLimits();
}

void Conveyor::setVelocityLimits(const Q& vellimits) {
	_basicDevice.setVelocityLimits(vellimits);
}


Q Conveyor::getAccelerationLimits() const {
	return _basicDevice.getAccelerationLimits();
}


void Conveyor::setAccelerationLimits(const Q& acclimits) {
	_basicDevice.setAccelerationLimits(acclimits);
}


size_t Conveyor::getDOF() const {
	return _basicDevice.getDOF();
}


Frame* Conveyor::getBase() {
	return _base;
}

const Frame* Conveyor::getBase() const {
	return _base;
}


Frame* Conveyor::getEnd() {
	return _base;
}


const Frame* Conveyor::getEnd() const {
	return _base;
}



Jacobian Conveyor::baseJend(const State& state) const {
	BasicDeviceJacobian jac(_basicDevice, _base, state);
	return jac.get(state);
}


Jacobian Conveyor::baseJframe(const Frame* frame, const State& state) const {
	BasicDeviceJacobian jac(_basicDevice, frame, state);
	return jac.get(state);
}


