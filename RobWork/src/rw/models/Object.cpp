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

Object::Object(rw::kinematics::Frame* baseframe, Geometry::Ptr geom):
        _base(baseframe),
        _frames(std::vector<Frame*>(1,baseframe)),
        _geometry(std::vector<Geometry::Ptr>(1,geom))
{
    if(geom->getFrame()==NULL)
        geom->setFrame( baseframe );
}

Object::Object(rw::kinematics::Frame* baseframe, std::vector<Geometry::Ptr> geoms):
        _base(baseframe),
        _frames(std::vector<Frame*>(1,baseframe)),
        _geometry(geoms)
{
    BOOST_FOREACH(Geometry::Ptr geom, geoms){
        if(geom->getFrame()==NULL)
            geom->setFrame( baseframe );
    }
}

Object::Object(std::vector<rw::kinematics::Frame*> frames):
        _base(frames[0]),
        _frames(frames)
{

}

Object::Object(std::vector<rw::kinematics::Frame*> frames, Geometry::Ptr geom):
        _base(frames[0]),
        _frames(frames),
        _geometry(std::vector<Geometry::Ptr>(1,geom))
{
    if(geom->getFrame()==NULL)
        geom->setFrame( _base );
}

Object::Object(std::vector<rw::kinematics::Frame*> frames, std::vector<Geometry::Ptr> geoms):
        _base(frames[0]),
        _frames(frames),
        _geometry(geoms)
{
    BOOST_FOREACH(Geometry::Ptr geom, geoms){
        if(geom->getFrame()==NULL)
            geom->setFrame( _base );
    }
}

Object::~Object(){

}

void Object::addGeometry(Geometry::Ptr geom){
    if(geom->getFrame()==NULL)
        geom->setFrame(_base);
    _geometry.push_back(geom);
}

void Object::addModel(Model3D::Ptr model){
    _models.push_back(model);
}

void Object::addFrame(rw::kinematics::Frame* frame){
    _frames.push_back(frame);
}

void Object::removeGeometry(Geometry::Ptr geom){
    std::vector<Geometry::Ptr>::iterator iter = std::find(_geometry.begin(),_geometry.end(),geom);
    if(iter!=_geometry.end())
        _geometry.erase(iter);
}

void Object::removeModel(rw::graphics::Model3D::Ptr model)
{
	std::vector<Model3D::Ptr>::iterator iter = std::find(_models.begin(),_models.end(),model);
    if(iter!=_models.end())
        _models.erase(iter);
}

const std::vector<Geometry::Ptr>& Object::getGeometry(){
    return _geometry;
}

const std::vector<Model3D::Ptr>& Object::getModels(){
    return _models;
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
