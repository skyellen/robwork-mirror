/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "DynamicWorkCellBuilder.hpp"

#include <rw/geometry/Plane.hpp>
#include <rw/geometry/Sphere.hpp>
#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Tube.hpp>
#include <rw/geometry/Box.hpp>
#include <rw/graphics/Model3D.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/RigidObject.hpp>
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/proximity/ProximitySetup.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using namespace rwsim::dynamics;
using namespace rwsimlibs::test;

#define PAH_DARKGRAY Vector3D<float>(98.f,98.f,98.f)/255.f
#define PAH_LIGHTGRAY Vector3D<float>(149.f,149.f,149.f)/255.f
#define PAH_BLUE Vector3D<float>(64.f,116.f,142.f)/255.f
#define PAH_ORANGE Vector3D<float>(227.f,104.f,12.f)/255.f

DynamicWorkCellBuilder::ColorScheme::ColorScheme():
	fixedBodies(Vector3D<float>(0.5f,0.5f,0.5f)),
	kinematicBodies(fixedBodies),
	dynamicBodies(fixedBodies)
{
}

DynamicWorkCellBuilder::ColorScheme::ColorScheme(const rw::math::Vector3D<float>& color):
	fixedBodies(color), kinematicBodies(color), dynamicBodies(color)
{
}

DynamicWorkCellBuilder::ColorScheme::ColorScheme(const rw::math::Vector3D<float>& fixedBodies, const rw::math::Vector3D<float>& kinematicBodies, const rw::math::Vector3D<float>& dynamicBodies):
	fixedBodies(fixedBodies), kinematicBodies(kinematicBodies), dynamicBodies(dynamicBodies)
{
}

DynamicWorkCellBuilder::PaHColors::PaHColors():
	ColorScheme(PAH_BLUE, PAH_LIGHTGRAY, PAH_ORANGE)
{
}

DynamicWorkCellBuilder::DynamicWorkCellBuilder(const ColorScheme& colors):
	_colors(colors)
{
}

DynamicWorkCellBuilder::~DynamicWorkCellBuilder() {
}

void DynamicWorkCellBuilder::addFloor(const DynamicWorkCell::Ptr dwc, const std::string& name, const bool trimesh) const {
	addPlane(dwc,Vector3D<>::z(),0,name, trimesh);
}

void DynamicWorkCellBuilder::addPlane(const DynamicWorkCell::Ptr dwc, const rw::math::Vector3D<>& n, const double d, const std::string& name, const bool trimesh) const {
	const WorkCell::Ptr wc = dwc->getWorkcell();

	FixedFrame* const frame = new FixedFrame(name,Transform3D<>::identity());
	wc->addFrame(frame,wc->getWorldFrame());

	const Plane::Ptr geoData = ownedPtr(new Plane(n,d));
	Geometry::Ptr geo;
	if (trimesh)
		geo = ownedPtr(new Geometry(geoData->createMesh(0,5), name));
	else
		geo = ownedPtr(new Geometry(geoData, name));
	const RigidObject::Ptr robject = ownedPtr(new RigidObject(frame));
	robject->addGeometry(geo);

	const Model3D::Material material(name+"Material",_colors.fixedBodies[0],_colors.fixedBodies[1],_colors.fixedBodies[2]);
	const Model3D::Ptr model = ownedPtr(new Model3D(name));
	model->addTriMesh(material,*geoData->createMesh(0,5));
	robject->addModel(model);

	wc->add(robject);

	const Body::Ptr body = ownedPtr(new FixedBody(defaultInfo(),robject));
	dwc->addBody(body);
}

void DynamicWorkCellBuilder::addBall(const DynamicWorkCell::Ptr dwc, const double radius, const double density, const std::string& name, const std::string& parent) const {
	const WorkCell::Ptr wc = dwc->getWorkcell();

	Frame* const parentFrame = wc->findFrame(parent);
	if (parentFrame == NULL)
		RW_THROW("The given parent frame \"" << parent << "\"does not exist!");
	MovableFrame* const frame = new MovableFrame(name);
	wc->addFrame(frame,parentFrame);

	const GeometryData::Ptr geoData = ownedPtr(new Sphere(radius,6));
	Geometry::Ptr geo = ownedPtr(new Geometry(geoData, "Sphere"));
	const RigidObject::Ptr robject = ownedPtr(new RigidObject(frame));
	robject->addGeometry(geo);

	const Model3D::Material material("BallMaterial",_colors.dynamicBodies[0],_colors.dynamicBodies[1],_colors.dynamicBodies[2]);
	const Model3D::Ptr model = ownedPtr(new Model3D("Sphere"));
	model->addTriMesh(material,*geoData->getTriMesh());
	robject->addModel(model);

	wc->add(robject);

	BodyInfo info;
	defaultInfo(info);
	ballInfo(info,radius,density);
	const Body::Ptr body = ownedPtr(new RigidBody(info,robject));
	dwc->addBody(body);
}

void DynamicWorkCellBuilder::addBallFixed(const DynamicWorkCell::Ptr dwc, const double radius, const std::string& name, const std::string& parent) const {
	const WorkCell::Ptr wc = dwc->getWorkcell();

	Frame* const parentFrame = wc->findFrame(parent);
	if (parentFrame == NULL)
		RW_THROW("The given parent frame \"" << parent << "\"does not exist!");
	FixedFrame* const frame = new FixedFrame(name,Transform3D<>());
	wc->addFrame(frame,parentFrame);

	const GeometryData::Ptr geoData = ownedPtr(new Sphere(radius,6));
	Geometry::Ptr geo = ownedPtr(new Geometry(geoData, "Sphere"));
	const RigidObject::Ptr robject = ownedPtr(new RigidObject(frame));
	robject->addGeometry(geo);

	const Model3D::Material material("BallMaterial",_colors.fixedBodies[0],_colors.fixedBodies[1],_colors.fixedBodies[2]);
	const Model3D::Ptr model = ownedPtr(new Model3D("Sphere"));
	model->addTriMesh(material,*geoData->getTriMesh());
	robject->addModel(model);

	wc->add(robject);

	BodyInfo info;
	defaultInfo(info);
	const Body::Ptr body = ownedPtr(new FixedBody(info,robject));
	dwc->addBody(body);
}

void DynamicWorkCellBuilder::addCylinder(const DynamicWorkCell::Ptr dwc, const double radius, const double height, const double density, const std::string& name, const std::string& parent, const bool trimesh) const {
	const WorkCell::Ptr wc = dwc->getWorkcell();

	Frame* const parentFrame = wc->findFrame(parent);
	if (parentFrame == NULL)
		RW_THROW("The given parent frame \"" << parent << "\"does not exist!");
	MovableFrame* const frame = new MovableFrame(name);
	wc->addFrame(frame,parentFrame);

	GeometryData::Ptr geoData = ownedPtr(new Cylinder(radius,height));
	if (trimesh)
		geoData = geoData->getTriMesh(true);
	Geometry::Ptr geo = ownedPtr(new Geometry(geoData, "Cylinder"));
	const RigidObject::Ptr robject = ownedPtr(new RigidObject(frame));
	robject->addGeometry(geo);

	const Model3D::Material material("CylinderMaterial",_colors.dynamicBodies[0],_colors.dynamicBodies[1],_colors.dynamicBodies[2]);
	const Model3D::Ptr model = ownedPtr(new Model3D("Cylinder"));
	model->addTriMesh(material,*geoData->getTriMesh());
	robject->addModel(model);

	wc->add(robject);

	BodyInfo info;
	defaultInfo(info);
	cylinderInfo(info,radius,height,density);
	const Body::Ptr body = ownedPtr(new RigidBody(info,robject));
	dwc->addBody(body);
}

void DynamicWorkCellBuilder::addCylinderFixed(const DynamicWorkCell::Ptr dwc, const double radius, const double height, const std::string& name, const std::string& parent, const bool trimesh) const {
	const WorkCell::Ptr wc = dwc->getWorkcell();

	Frame* const parentFrame = wc->findFrame(parent);
	if (parentFrame == NULL)
		RW_THROW("The given parent frame \"" << parent << "\"does not exist!");
	FixedFrame* const frame = new FixedFrame(name,Transform3D<>());
	wc->addFrame(frame,parentFrame);

	GeometryData::Ptr geoData = ownedPtr(new Cylinder(radius,height));
	if (trimesh)
		geoData = geoData->getTriMesh(true);
	Geometry::Ptr geo = ownedPtr(new Geometry(geoData, "Cylinder"));
	const RigidObject::Ptr robject = ownedPtr(new RigidObject(frame));
	robject->addGeometry(geo);

	const Model3D::Material material("CylinderMaterial",_colors.dynamicBodies[0],_colors.dynamicBodies[1],_colors.dynamicBodies[2]);
	const Model3D::Ptr model = ownedPtr(new Model3D("Cylinder"));
	model->addTriMesh(material,*geoData->getTriMesh());
	robject->addModel(model);

	wc->add(robject);

	BodyInfo info;
	defaultInfo(info);
	const Body::Ptr body = ownedPtr(new FixedBody(info,robject));
	dwc->addBody(body);
}

void DynamicWorkCellBuilder::addTube(const DynamicWorkCell::Ptr dwc, const double radius, const double thickness, const double height, const double density, const std::string& name, const bool trimesh) const {
	const WorkCell::Ptr wc = dwc->getWorkcell();

	MovableFrame* const frame = new MovableFrame(name);
	wc->addFrame(frame,wc->getWorldFrame());

	GeometryData::Ptr geoData = ownedPtr(new Tube(radius,thickness,height));
	if (trimesh)
		geoData = geoData->getTriMesh(true);
	Geometry::Ptr geo = ownedPtr(new Geometry(geoData, "Tube"));
	const RigidObject::Ptr robject = ownedPtr(new RigidObject(frame));
	robject->addGeometry(geo);

	const Model3D::Material material("TubeMaterial",_colors.dynamicBodies[0],_colors.dynamicBodies[1],_colors.dynamicBodies[2]);
	const Model3D::Ptr model = ownedPtr(new Model3D("Tube"));
	model->addTriMesh(material,*geoData->getTriMesh());
	robject->addModel(model);

	wc->add(robject);

	BodyInfo info;
	defaultInfo(info);
	tubeInfo(info,radius,thickness,height,density);
	const Body::Ptr body = ownedPtr(new RigidBody(info,robject));
	dwc->addBody(body);
}

void DynamicWorkCellBuilder::addBox(const DynamicWorkCell::Ptr dwc, const double x, const double y, const double z, const double density, const std::string& name, const bool trimesh) const {
	const WorkCell::Ptr wc = dwc->getWorkcell();

	MovableFrame* const frame = new MovableFrame(name);
	wc->addFrame(frame,wc->getWorldFrame());

	GeometryData::Ptr geoData = ownedPtr(new Box(x, y, z));
	if (trimesh)
		geoData = geoData->getTriMesh(true);
	Geometry::Ptr geo = ownedPtr(new Geometry(geoData, "Box"));
	const RigidObject::Ptr robject = ownedPtr(new RigidObject(frame));
	robject->addGeometry(geo);

	const Model3D::Material material("BoxMaterial",_colors.dynamicBodies[0],_colors.dynamicBodies[1],_colors.dynamicBodies[2]);
	const Model3D::Ptr model = ownedPtr(new Model3D("Box"));
	model->addTriMesh(material,*geoData->getTriMesh());
	robject->addModel(model);

	wc->add(robject);

	Body::Ptr body;
	BodyInfo info;
	defaultInfo(info);
	if (density > 0) {
		boxInfo(info,x,y,z,density);
		body = ownedPtr(new RigidBody(info,robject));
	} else {
		body = ownedPtr(new KinematicBody(info,robject));
	}

	dwc->addBody(body);
}

void DynamicWorkCellBuilder::addBoxKin(const DynamicWorkCell::Ptr dwc, const double x, const double y, const double z, const std::string& name, const bool trimesh) const {
	addBox(dwc,x,y,z,-1,name,trimesh);
	std::vector<Model3D::Material>& material = dwc->getWorkcell()->findObject(name)->getModels()[0]->getMaterials();
	material[0].rgb[0] = _colors.kinematicBodies[0];
	material[0].rgb[1] = _colors.kinematicBodies[1];
	material[0].rgb[2] = _colors.kinematicBodies[2];
}

BodyInfo DynamicWorkCellBuilder::defaultInfo() {
	BodyInfo info;
	defaultInfo(info);
	return info;
}

void DynamicWorkCellBuilder::defaultInfo(BodyInfo& info) {
    info.material = "Plastic";
    info.objectType = "hardObj";
	info.integratorType = "Heun";
}

void DynamicWorkCellBuilder::ballInfo(BodyInfo& info, const double radius, const double density) {
	defaultInfo(info);
	info.mass = 4./3.*Pi*radius*radius*radius*density;
	const double inertia = 2.*info.mass*radius*radius/5.;
	info.inertia = InertiaMatrix<>(inertia,inertia,inertia);
}

void DynamicWorkCellBuilder::cylinderInfo(BodyInfo& info, const double radius, const double height, const double density) {
	defaultInfo(info);
	info.mass = Pi*radius*radius*height*density;
	const double iZ = info.mass*radius*radius/2.;
	const double i = info.mass*(3*radius*radius+height*height)/12.;
	info.inertia = InertiaMatrix<>(i,i,iZ);
}

void DynamicWorkCellBuilder::tubeInfo(BodyInfo& info, const double radius, const double thickness, const double height, const double density) {
	defaultInfo(info);
	const double radius2 = radius*radius;
	const double rOut = radius+thickness;
	const double rOut2 = rOut*rOut;
	info.mass = Pi*(rOut2-radius2)*height*density;
	const double iZ = info.mass*(rOut2+radius2)/2.;
	const double i = info.mass*(3*(rOut2+radius2)+height*height)/12.;
	info.inertia = InertiaMatrix<>(i,i,iZ);
}

void DynamicWorkCellBuilder::boxInfo(BodyInfo& info, const double x, const double y, const double z, const double density) {
	defaultInfo(info);
	info.mass = x*y*z*density;
	const double iX = info.mass*(y*y+z*z)/12.;
	const double iY = info.mass*(x*x+z*z)/12.;
	const double iZ = info.mass*(x*x+y*y)/12.;
	info.inertia = InertiaMatrix<>(iX,iY,iZ);
}

void DynamicWorkCellBuilder::addMaterialData(const DynamicWorkCell::Ptr dwc, const double mu, const double restitution) {
	dwc->getMaterialData().add("Plastic","");
	FrictionData friction;
	friction.type = Custom;
	friction.typeName = "MicroSlip";
	friction.parameters.push_back(std::make_pair("gamma",Q(1,250.)));
	friction.parameters.push_back(std::make_pair("r",Q(1,0.)));
	friction.parameters.push_back(std::make_pair("mu",Q(1,mu)));
	dwc->getMaterialData().addFrictionData("Plastic","Plastic",friction);
	friction.type = Coulomb;
	friction.typeName = "Coulomb";
	friction.parameters.clear();
	friction.parameters.push_back(std::make_pair("mu",Q(1,mu)));
	dwc->getMaterialData().addFrictionData("Plastic","Plastic",friction);
	dwc->getContactData().add("hardObj","A hard object. with low elasticity");
	ContactDataMap::NewtonData ndata;
	ndata.cr = restitution;
	dwc->getContactData().addNewtonData("hardObj","hardObj",ndata);
}

void DynamicWorkCellBuilder::contactsExclude(const DynamicWorkCell::Ptr dwc, const std::string& bodyA, const std::string& bodyB) {
	const WorkCell::Ptr wc = dwc->getWorkcell();
	ProximitySetup proximitySetup = ProximitySetup::get(wc);
	proximitySetup.addProximitySetupRule(ProximitySetupRule::makeExclude(bodyA,bodyB));
	ProximitySetup::set(proximitySetup, wc);
}
