#include "Gripper.hpp"

#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;



Gripper::Gripper(const std::string& name) :
	_name(name),
	_jaw(new JawPrimitive),
	_tcp(Transform3D<>(Vector3D<>(0, 0, 0.05))),
	_jawdist(0),
	_opening(0.05),
	_force(50)
{
	setBaseGeometry(Q(3, 0.15, 0.1, 0.05));
	setJawGeometry(Q(10, 0, 0.1, 0.025, 0.02, 0, 0, 0.05, 0, 90, 0));
}



void Gripper::updateGripper(rw::models::WorkCell::Ptr wc, rwsim::dynamics::DynamicWorkCell::Ptr dwc,
	rw::models::TreeDevice::Ptr dev, rwsim::dynamics::RigidDevice::Ptr ddev, rw::kinematics::State& state)
{
	// remove existing objects
	cout << "- Removing objects..." << endl;
	wc->removeObject(wc->findObject("gripper.Base").get());
	wc->removeObject(wc->findObject("gripper.LeftFinger").get());
	wc->removeObject(wc->findObject("gripper.RightFinger").get());
	cout << "- Objects removed." << endl;
	
	// create and add new objects
	cout << "- Adding new objects..." << endl;
	
	// if base is parametrized, the box has to be moved from origin by half its height
	Transform3D<> baseT;
	if (_isBaseParametrized) {
		baseT = Transform3D<>(-0.5*_baseParameters(2)*Vector3D<>::z());
	}
	
	Object* baseobj = new Object(wc->findFrame("gripper.Base"));
	Model3D* basemodel = new Model3D("BaseModel");
	basemodel->addTriMesh(Model3D::Material("stlmat",0.4f,0.4f,0.4f), *_baseGeometry->getGeometryData()->getTriMesh() );
	basemodel->setTransform(baseT);
	_baseGeometry->setTransform(baseT);
	baseobj->addModel(basemodel);
	baseobj->addGeometry(_baseGeometry);
	wc->add(baseobj);
	dwc->findBody("gripper.Base")->setObject(baseobj);
	
	Object* leftobj = new Object(wc->findFrame("gripper.LeftFinger"));
	//Geometry::Ptr leftgeo = ownedPtr(new Geometry(_jaw, string("LeftFingerGeo")));
	Model3D* leftmodel = new Model3D("LeftModel");
	leftmodel->addTriMesh(Model3D::Material("stlmat",0.4f,0.4f,0.4f), *_leftGeometry->getGeometryData()->getTriMesh() );
	leftmodel->setTransform(Transform3D<>());
	_leftGeometry->setTransform(Transform3D<>());
	leftobj->addModel(leftmodel);
	leftobj->addGeometry(_leftGeometry);
	wc->add(leftobj);
	dwc->findBody("gripper.LeftFinger")->setObject(leftobj);
	
	Object* rightobj = new Object(wc->findFrame("gripper.RightFinger"));
	//Geometry::Ptr rightgeo = ownedPtr(new Geometry(_jaw, string("RightFingerGeo")));
	Model3D* rightmodel = new Model3D("RightModel");
	rightmodel->addTriMesh(Model3D::Material("stlmat",0.4f,0.4f,0.4f), *_rightGeometry->getGeometryData()->getTriMesh() );
	rightmodel->setTransform(Transform3D<>(Vector3D<>(), RPY<>(0, 180*Deg2Rad, 180*Deg2Rad).toRotation3D()));
	_rightGeometry->setTransform(Transform3D<>(Vector3D<>(), RPY<>(0, 180*Deg2Rad, 180*Deg2Rad).toRotation3D()));
	rightobj->addModel(rightmodel);
	rightobj->addGeometry(_rightGeometry);
	wc->add(rightobj);
	dwc->findBody("gripper.RightFinger")->setObject(rightobj);
	cout << "Objects added." << endl;
	
	// set tcp
	string tcpFrameName = wc->getPropertyMap().get<string>("gripperTCP");
	MovableFrame* tcp = wc->findFrame<MovableFrame>(tcpFrameName);
	tcp->setTransform(_tcp, state);
	
	//cout << "LOL" << tcp->getName() << endl;
	
	// set bounds
	dev->setBounds(make_pair(Q(1, _jawdist), Q(1, _opening)));
	dev->setQ(Q(1, _jawdist), state);
	
	// set force
	ddev->setMotorForceLimits(Q(2, _force, _force));
	
	cout << "Gripper updated!" << endl;
}



/*void Gripper::loadTasks(std::string filename)
{
	if (filename.empty()) return;
	
	//_dataFilename = filename;
		
	cout << "Loading tasks from: " << filename << "\n";
	_tasks = GraspTask::load(filename);
	cout << "Tasks loaded!" << endl;
}



void Gripper::saveTasks(std::string filename)
{
	if (filename.empty()) return;
	
	if (!_tasks) return;
	
	//_dataFilename = filename;
	
	try {
		cout << "Saving tasks to: " << filename << "\n";
		GraspTask::saveRWTask(_tasks, filename);
		cout << "Tasks saved!" << endl;
	} catch (...)
	{
	}
}*/
