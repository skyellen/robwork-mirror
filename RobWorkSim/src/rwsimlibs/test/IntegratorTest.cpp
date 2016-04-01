/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "IntegratorTest.hpp"
#include "DynamicWorkCellBuilder.hpp"

#include <rw/geometry/Cylinder.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/RigidObject.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rwsim::dynamics;
using namespace rwsimlibs::test;

#define DEFAULT_DT 10 // in ms
#define DEFAULT_INTEGRATOR "Heun"
#define MASS 1.554 // in kg
#define GRAVITY -9.82

IntegratorTest::IntegratorTest() {
}

IntegratorTest::~IntegratorTest() {
}

bool IntegratorTest::isEngineSupported(const std::string& engineID) const {
	if (engineID == "RWPhysics")
		return false;
	return true;
}

rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> IntegratorTest::getDWC(const rw::common::PropertyMap& map) {
	const std::string integrator = map.get<std::string>("IntegratorType");
	if (_integratorTypeToDWC.find(integrator) == _integratorTypeToDWC.end()) {
		_integratorTypeToDWC[integrator] = makeIntegratorDWC(integrator);
	}
	return _integratorTypeToDWC[integrator];
}

rw::common::Ptr<rw::common::PropertyMap> IntegratorTest::getDefaultParameters() const {
	const PropertyMap::Ptr map = EngineTest::getDefaultParameters();
	map->add<double>("Timestep","Timestep size in ms.",DEFAULT_DT);
	map->add<std::string>("IntegratorType","Integrator type.",DEFAULT_INTEGRATOR);
	return map;
}

DynamicWorkCell::Ptr IntegratorTest::makeIntegratorDWC(const std::string& integratorType) {
	MovableFrame* const frame = new MovableFrame("Object");
	const GeometryData::Ptr cylinderData = ownedPtr(new Cylinder(0.015,0.1));
	const GeometryData::Ptr cylinderEndData = ownedPtr(new Cylinder(0.045,0.02));
	Geometry::Ptr cylinder = ownedPtr(new Geometry(cylinderData, "CylinderGeo"));
	Geometry::Ptr cylinderEnd = ownedPtr(new Geometry(cylinderEndData, "CylinderEndGeo"));
	const Transform3D<> Tcyl(Vector3D<>(0,0,-27./700.));
	const Transform3D<> TcylEnd(Vector3D<>(0,0,0.06-27./700.));
	cylinder->setTransform(Tcyl);
	cylinderEnd->setTransform(TcylEnd);
	RigidObject::Ptr object = ownedPtr(new RigidObject(frame));
	object->addGeometry(cylinder);
	object->addGeometry(cylinderEnd);

	// Visualization
	const Model3D::Material material("Material",227.f/255.f,104.f/255.f,12.f/255.f);
	const Model3D::Ptr cylinderModel = ownedPtr(new Model3D("CylinderGeo"));
	const Model3D::Ptr cylinderEndModel = ownedPtr(new Model3D("CylinderEndGeo"));
	cylinderModel->addTriMesh(material,*cylinderData->getTriMesh());
	cylinderEndModel->addTriMesh(material,*cylinderEndData->getTriMesh());
	cylinderModel->setTransform(Tcyl);
	cylinderEndModel->setTransform(TcylEnd);
	object->addModel(cylinderModel);
	object->addModel(cylinderEndModel);

	const WorkCell::Ptr wc = ownedPtr(new WorkCell("FreeMotionTestWorkCell"));
	wc->addFrame(frame,wc->getWorldFrame()); // takes ownership of the MovableFrame
	wc->add(object);

	const StateStructure::Ptr stateStructure = wc->getStateStructure();
	State state = stateStructure->getDefaultState();
	frame->setTransform(Transform3D<>(Vector3D<>::zero(),RPY<>(0,Pi/4,0).toRotation3D()),state);
	stateStructure->setDefaultState(state);

	BodyInfo info;
	DynamicWorkCellBuilder::defaultInfo(info);
	info.mass = MASS;
	info.inertia = InertiaMatrix<>(0.0023164,0,0,0,0.0023164,0,0,0,0.0011243);
	if (integratorType == "")
		info.integratorType = "Heun";
	else
		info.integratorType = integratorType;
	RigidBody::Ptr rbody = ownedPtr(new RigidBody(info,object));

	const DynamicWorkCell::Ptr dwc = ownedPtr(new DynamicWorkCell(wc));
	DynamicWorkCellBuilder::addMaterialData(dwc,0,0);
	dwc->addBody(rbody);
	dwc->setGravity(Vector3D<>(0,0,GRAVITY));

	return dwc;
}
