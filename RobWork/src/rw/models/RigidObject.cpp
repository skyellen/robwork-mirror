#include "RigidObject.hpp"


using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::models;

RigidObject::RigidObject(rw::kinematics::Frame* baseframe):
        Object(baseframe),
        _mass(1.0),
        _Ibody(rw::math::InertiaMatrix<>::makeSolidSphereInertia(1.0,0.1))
{

}

RigidObject::RigidObject(rw::kinematics::Frame* baseframe, Geometry::Ptr geom):
		Object(baseframe),
        _geometry(std::vector<Geometry::Ptr>(1,geom)),
        _mass(1.0),
        _Ibody(rw::math::InertiaMatrix<>::makeSolidSphereInertia(1.0,0.1))
{
    if(geom->getFrame()==NULL)
        geom->setFrame( baseframe );
}

RigidObject::RigidObject(rw::kinematics::Frame* baseframe, std::vector<Geometry::Ptr> geoms):
		Object(baseframe),
        _geometry(geoms),
        _mass(1.0),
        _Ibody(rw::math::InertiaMatrix<>::makeSolidSphereInertia(1.0,0.1))
{
    BOOST_FOREACH(Geometry::Ptr geom, geoms){
        if(geom->getFrame()==NULL)
            geom->setFrame( baseframe );
    }
}

RigidObject::RigidObject(std::vector<rw::kinematics::Frame*> frames):
        Object(frames),
        _mass(1.0),
        _Ibody(rw::math::InertiaMatrix<>::makeSolidSphereInertia(1.0,0.1))
{

}

RigidObject::RigidObject(std::vector<rw::kinematics::Frame*> frames, Geometry::Ptr geom):
		Object(frames),
        _geometry(std::vector<Geometry::Ptr>(1,geom)),
        _mass(1.0),
        _Ibody(rw::math::InertiaMatrix<>::makeSolidSphereInertia(1.0,0.1))
{
    if(geom->getFrame()==NULL)
        geom->setFrame( getBase() );
}

RigidObject::RigidObject(std::vector<rw::kinematics::Frame*> frames, std::vector<Geometry::Ptr> geoms):
		Object(frames),
        _geometry(geoms),
        _mass(1.0),
        _Ibody(rw::math::InertiaMatrix<>::makeSolidSphereInertia(1.0,0.1))
{
    BOOST_FOREACH(Geometry::Ptr geom, geoms){
        if(geom->getFrame()==NULL)
            geom->setFrame( getBase() );
    }
}


RigidObject::~RigidObject(){

}

void RigidObject::addGeometry(Geometry::Ptr geom){
    if(geom->getFrame()==NULL)
        geom->setFrame(  getBase() );
    _geometry.push_back(geom);
}

void RigidObject::addModel(Model3D::Ptr model){
    _models.push_back(model);
}

void RigidObject::removeGeometry(Geometry::Ptr geom){
    std::vector<Geometry::Ptr>::iterator iter = std::find(_geometry.begin(),_geometry.end(),geom);
    if(iter!=_geometry.end())
        _geometry.erase(iter);
}

void RigidObject::removeModel(rw::graphics::Model3D::Ptr model)
{
	std::vector<Model3D::Ptr>::iterator iter = std::find(_models.begin(),_models.end(),model);
    if(iter!=_models.end())
        _models.erase(iter);
}

const std::vector<Geometry::Ptr>& RigidObject::getGeometry() const {
    return _geometry;
}

const std::vector<Model3D::Ptr>& RigidObject::getModels() const {
    return _models;
}

void RigidObject::approximateInertia(){
	RW_THROW(" Not implemented yet! ");
}


void RigidObject::approximateInertiaCOM(){
	RW_THROW(" Not implemented yet! ");
}
