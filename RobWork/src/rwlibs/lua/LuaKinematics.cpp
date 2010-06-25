#include "LuaKinematics.hpp"



#define rwkin rwlibs::lua::kinematics

#include <iostream>
using namespace std;
#include <sstream>

using namespace rwlibs::lua;

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

rwkin::State::State(const rw::kinematics::State& state):
	rw::kinematics::State(state)
{
}

rwkin::State rwkin::State::copy(){
	return *this;
}

unsigned int rwkin::State::size() const{
	return ((rw::kinematics::State*)(this))->size();
}

std::string rwkin::State::__tostring() const{
	// TODO: write something usefull here
	std::stringstream sstr;
	sstr << "State(" << this->size() ;
	for(size_t i=0;i<this->size();i++)
		sstr << ", " << (*this)[i];
	sstr << ")";
	return sstr.str();
}


rwkin::Frame::Frame(rw::kinematics::Frame* frame):
		_frame(frame)
{}
math::Transform3D rwkin::Frame::getTransform(const rwkin::State& state) const {
	return _frame->getTransform(state);
}

int rwkin::Frame::getDOF() const{
	return _frame->getDOF();
}

rwkin::Frame* rwkin::Frame::getParent(){
	return new rwkin::Frame(_frame->getParent());
}

rwkin::Frame* rwkin::Frame::getParent(const rwkin::State& state){
	return new rwkin::Frame( _frame->getParent(state) );
}

void rwkin::Frame::attachTo(rwkin::Frame* parent, rwkin::State& state){
	_frame->attachTo(parent->get(), state);
}

bool rwkin::Frame::isDAF(){
	return rw::kinematics::Kinematics::isDAF(*_frame);
}

math::Transform3D rwkin::Frame::wTt(const State& state) const{
	return rw::kinematics::Kinematics::worldTframe(_frame, state);
}

math::Transform3D rwkin::Frame::tTf(const rwkin::Frame& frame, const State& state) const{
	return rw::kinematics::Kinematics::frameTframe(_frame, frame.get(), state);
}

const rw::kinematics::Frame* rwkin::Frame::get() const{return _frame;}

rw::kinematics::Frame* rwkin::Frame::get() {return _frame;}

std::string rwkin::Frame::__tostring() const{
	return toString<rw::kinematics::Frame>(*_frame);
}






rwkin::FixedFrame::FixedFrame(rw::kinematics::FixedFrame* frame):
		rwkin::Frame(frame),
		_fframe(frame)
{}
void rwkin::FixedFrame::setTransform(const math::Transform3D& transform){
	_fframe->setTransform( transform );
}
const rw::kinematics::FixedFrame* rwkin::FixedFrame::get() const{ return _fframe;};

std::string rwkin::FixedFrame::__tostring() const{ return toString(*_fframe); }





rwkin::MovableFrame::MovableFrame(rw::kinematics::MovableFrame* frame):
		Frame(frame),
		_mframe(frame)
{}


void rwkin::MovableFrame::setTransform(const math::Transform3D& transform, rwkin::State& state){
	_mframe->setTransform(transform, state);
}

const rw::kinematics::MovableFrame* rwkin::MovableFrame::get() const{return _mframe;}
std::string rwkin::MovableFrame::__tostring() const{return toString(*_mframe);}




math::Transform3D rwkin::frameTframe(const rwkin::Frame* from, const rwkin::Frame* to, const rwkin::State& state){
	return rw::kinematics::Kinematics::frameTframe(from->get(), to->get(), state );
}

math::Transform3D rwkin::worldTframe(const rwkin::Frame* to, const rwkin::State& state){
	return rw::kinematics::Kinematics::worldTframe( to->get(),  state);
}


rwkin::Frame rwkin::worldFrame(rwkin::Frame& frame, const rwkin::State& state) {
    return rwkin::Frame(&rw::kinematics::Kinematics::worldFrame( *frame.get(), state ));
}

void rwkin::gripFrame(rwkin::State& state, rwkin::Frame& item, rwkin::Frame& gripper){
	return rw::kinematics::Kinematics::gripFrame( state, *item.get(), *gripper.get() );
}


bool rwkin::isDAF(const rwkin::Frame& frame){
	return rw::kinematics::Kinematics::isDAF( *frame.get() );
}

