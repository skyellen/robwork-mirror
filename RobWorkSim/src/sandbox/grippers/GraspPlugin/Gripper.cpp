#include "Gripper.hpp"

#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;



Gripper::Gripper(const std::string& name) :
	_name(name),
	_baseGeometry(NULL),
	_leftGeometry(NULL),
	_rightGeometry(NULL),
	_tcp(Transform3D<>(Vector3D<>(0, 0, 0.075))),
	_jawdist(0),
	_opening(0.05),
	_force(50)
{
	setBaseGeometry(Q(3, 0.15, 0.1, 0.05));
	setJawGeometry(Q(10, 0, 0.1, 0.025, 0.02, 0, 0, 0.05, 0, 90*Deg2Rad, 0));
}



void Gripper::updateGripper(rw::models::WorkCell::Ptr wc, rwsim::dynamics::DynamicWorkCell::Ptr dwc,
	rw::models::TreeDevice::Ptr dev, rwsim::dynamics::RigidDevice::Ptr ddev, rw::kinematics::State& state,
	TaskDescription::Ptr td)
{
	Geometry::Ptr baseGeometry = getBaseGeometry();
	Geometry::Ptr leftGeometry = getJawGeometry();
	Geometry::Ptr rightGeometry = getJawGeometry();
	
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
	basemodel->addTriMesh(Model3D::Material("stlmat",0.4f,0.4f,0.4f), *baseGeometry->getGeometryData()->getTriMesh() );
	basemodel->setTransform(baseT);
	baseGeometry->setTransform(baseT);
	baseobj->addModel(basemodel);
	baseobj->addGeometry(baseGeometry);
	wc->add(baseobj);
	dwc->findBody("gripper.Base")->setObject(baseobj);
	
	Object* leftobj = new Object(wc->findFrame("gripper.LeftFinger"));
	Model3D* leftmodel = new Model3D("LeftModel");
	leftmodel->addTriMesh(Model3D::Material("stlmat",0.4f,0.4f,0.4f), *leftGeometry->getGeometryData()->getTriMesh() );
	leftmodel->setTransform(Transform3D<>());
	leftGeometry->setTransform(Transform3D<>());
	leftobj->addModel(leftmodel);
	leftobj->addGeometry(leftGeometry);
	wc->add(leftobj);
	dwc->findBody("gripper.LeftFinger")->setObject(leftobj);
	
	Object* rightobj = new Object(wc->findFrame("gripper.RightFinger"));
	Model3D* rightmodel = new Model3D("RightModel");
	rightmodel->addTriMesh(Model3D::Material("stlmat",0.4f,0.4f,0.4f), *rightGeometry->getGeometryData()->getTriMesh() );
	rightmodel->setTransform(Transform3D<>(Vector3D<>(), Rotation3D<>(1, 0, 0, 0, 1, 0, 0, 0, -1)));
	rightGeometry->setTransform(Transform3D<>(Vector3D<>(), Rotation3D<>(1, 0, 0, 0, 1, 0, 0, 0, -1)));
	rightobj->addModel(rightmodel);
	rightobj->addGeometry(rightGeometry);
	wc->add(rightobj);
	dwc->findBody("gripper.RightFinger")->setObject(rightobj);
	cout << "Objects added." << endl;
	
	// set tcp
	//string tcpFrameName = wc->getPropertyMap().get<string>("gripperTCP");
	MovableFrame* tcp = (MovableFrame*)td->getGripperTCP(); //wc->findFrame<MovableFrame>(tcpFrameName);
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
