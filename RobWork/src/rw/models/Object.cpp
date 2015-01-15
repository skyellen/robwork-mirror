#include "Object.hpp"


using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::models;

Object::Object(rw::kinematics::Frame* baseframe):
        _base(baseframe),
        _frames(std::vector<Frame*>(1,baseframe))
{

}


Object::Object(std::vector<rw::kinematics::Frame*> frames):
        _base(frames[0]),
        _frames(frames)
{

}

Object::~Object(){

}

rw::kinematics::Frame* Object::getBase(){
    return _base;
}
const rw::kinematics::Frame* Object::getBase() const{
    return _base;
}
const std::vector<rw::kinematics::Frame*>& Object::getFrames(){
    return _frames;
}

void Object::addFrame(rw::kinematics::Frame* frame){
	_frames.push_back(frame);
}
