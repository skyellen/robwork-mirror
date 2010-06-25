#include "LuaModels.hpp"

#include "LuaKinematics.hpp"
#include "LuaMath.hpp"

#include <rw/models.hpp>

using namespace rwlibs::lua::models;
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
WorkCell::WorkCell(rw::models::WorkCellPtr wc):
		_wc(wc)
{
}

std::string WorkCell::getName() const {
	return _wc->getName();
}

kinematics::Frame WorkCell::getWorldFrame() const{
	return kinematics::Frame(_wc->getWorldFrame());
}
kinematics::Frame WorkCell::findFrame(const std::string& name) const{
	rw::kinematics::Frame *frame = _wc->findFrame(name);
	return kinematics::Frame(frame);
}
Device WorkCell::findDevice(const std::string& name) const{
	rw::models::Device *dev = _wc->findDevice(name);
	return Device(dev);
}
kinematics::State WorkCell::getDefaultState() const{
	rw::kinematics::State state = _wc->getDefaultState();
	return kinematics::State(state);
}
rw::models::WorkCellPtr WorkCell::get() const{return _wc;}
rw::models::WorkCellPtr WorkCell::get(){return _wc;}

std::string WorkCell::__tostring() const{
	return _wc->getName();
}


Device::Device(rw::models::DevicePtr device):
		_dev(device)
{}

void Device::setQ(const math::Q& q, kinematics::State& state) const{_dev->setQ(q, state);};
math::Q Device::getQ(const kinematics::State& state) const{ return _dev->getQ(state);}

math::Q Device::getAccLimits() const{ return _dev->getAccelerationLimits(); }
void Device::setAccLimits(const math::Q& lim){ _dev->setAccelerationLimits(lim); }

math::Q Device::getVelLimits() const{ return _dev->getVelocityLimits(); }
void Device::setVelLimits(const math::Q& lim){ _dev->setVelocityLimits(lim); }

math::Q Device::getMinPosLimits() const{ return _dev->getBounds().first; }
math::Q Device::getMaxPosLimits() const{ return _dev->getBounds().second; }

void Device::setPosLimits(const math::Q& min, const math::Q& max){
	_dev->setBounds(rw::models::Device::QBox(min,max));
}
unsigned int Device::getDOF() const{return _dev->getDOF();}

std::string Device::getName() const{return _dev->getName();}
void Device::setName(const std::string& name){_dev->setName(name);};

kinematics::Frame Device::getBase(){ return kinematics::Frame(_dev->getBase());}
const kinematics::Frame Device::getBase() const{ return kinematics::Frame(_dev->getBase());}
kinematics::Frame Device::getEnd(){ return kinematics::Frame(_dev->getEnd());}


math::Transform3D Device::bTf(
    const kinematics::Frame* f, const kinematics::State& state) const{
	return _dev->baseTframe(f->get(), state);
}
math::Transform3D Device::bTe(const kinematics::State& state) const{
	return _dev->baseTend(state);
}
math::Transform3D Device::wTb(const kinematics::State& state) const{
	return _dev->worldTbase(state);
}

rw::models::DevicePtr Device::get() const { return _dev;}

