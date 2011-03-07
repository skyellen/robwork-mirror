#include "LuaModels.hpp"

#include "LuaKinematics.hpp"
#include "LuaMath.hpp"

#include <rw/models.hpp>

using namespace rwlibs::lua;
#include <iostream>
using namespace std;
#include <sstream>

#define NS rw::math

namespace
{
    string eToString(const rw::common::Exception& e)
    {
        ostringstream buf;
        buf << e.getMessage();
        return buf.str();
    }

    template <typename T>
    string toString(const T& x)
    {
        ostringstream buf;
        buf << x;
        return buf.str();
    }
}
WorkCell::WorkCell(){};
WorkCell::WorkCell(rw::models::WorkCell::Ptr wc):
		_wc(wc)
{
}

std::string WorkCell::getName() const {
	return _wc->getName();
}

Frame WorkCell::getWorldFrame() const{
	return Frame(_wc->getWorldFrame());
}
Frame WorkCell::findFrame(const std::string& name) const{
	rw::kinematics::Frame *frame = _wc->findFrame(name);
	return Frame(frame);
}
Device WorkCell::findDevice(const std::string& name) const{
	rw::models::Device::Ptr dev = _wc->findDevice(name);
	return Device(dev);
}
State WorkCell::getDefaultState() const{
	rw::kinematics::State state = _wc->getDefaultState();
	return State(state);
}
rw::models::WorkCell::Ptr WorkCell::get() const{return _wc;}
rw::models::WorkCell::Ptr WorkCell::get(){return _wc;}

std::string WorkCell::__tostring() const{
	return _wc->getName();
}


Device::Device(rw::models::Device::Ptr device):
	_dev(device)
{}

void Device::setQ(const Q& q, State& state) const{_dev->setQ(q, state);};
Q Device::getQ(const State& state) const{ return _dev->getQ(state);}

Q Device::getAccLimits() const{ return _dev->getAccelerationLimits(); }
void Device::setAccLimits(const Q& lim){ _dev->setAccelerationLimits(lim); }

Q Device::getVelLimits() const{ return _dev->getVelocityLimits(); }
void Device::setVelLimits(const Q& lim){ _dev->setVelocityLimits(lim); }

Q Device::getMinPosLimits() const{ return _dev->getBounds().first; }
Q Device::getMaxPosLimits() const{ return _dev->getBounds().second; }

void Device::setPosLimits(const Q& min, const Q& max){
	_dev->setBounds(rw::models::Device::QBox(min,max));
}
unsigned int Device::getDOF() const{return _dev->getDOF();}

std::string Device::getName() const{return _dev->getName();}
void Device::setName(const std::string& name){_dev->setName(name);};

Frame Device::getBase(){ return Frame(_dev->getBase());}
const Frame Device::getBase() const{ return Frame(_dev->getBase());}
Frame Device::getEnd(){ return Frame(_dev->getEnd());}


Transform3D Device::bTf(const Frame* f, const State& state) const{
	return _dev->baseTframe(f->get(), state);
}
Transform3D Device::bTe(const State& state) const{
	return _dev->baseTend(state);
}
Transform3D Device::wTb(const State& state) const{
	return _dev->worldTbase(state);
}
Jacobian Device::bJf(const Frame* f, const State& state) const{
    return _dev->baseJframe(f->get(), state);
}
Jacobian Device::bJe(const State& state) const{
    return _dev->baseJend(state);
}
rw::models::Device::Ptr Device::get() const { return _dev;}


