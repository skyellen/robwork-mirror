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
	_force(50),
	_tasks(NULL),
	_dataFilename(name + "_data.xml")
{
}



void Gripper::updateGripper(rw::models::WorkCell::Ptr wc, rwsim::dynamics::DynamicWorkCell::Ptr dwc,
	rw::models::TreeDevice::Ptr dev, rwsim::dynamics::RigidDevice::Ptr ddev, rw::kinematics::State& state)
{
	// remove existing objects
	cout << "Removing objects..." << endl;
	wc->removeObject(wc->findObject("gripper.LeftFinger").get());
	wc->removeObject(wc->findObject("gripper.RightFinger").get());
	cout << "Objects removed." << endl;
	
	// create and add new objects
	cout << "Adding new objects..." << endl;
	Object* leftobj = new Object(wc->findFrame("gripper.LeftFinger"));
	Geometry::Ptr leftgeo = ownedPtr(new Geometry(_jaw, string("LeftFingerGeo")));
	Model3D* leftmodel = new Model3D("LeftModel");
	leftmodel->addTriMesh(Model3D::Material("stlmat",0.8f,0.8f,0.6f), *leftgeo->getGeometryData()->getTriMesh() );
	leftmodel->setTransform(Transform3D<>());
	leftgeo->setTransform(Transform3D<>());
	leftobj->addModel(leftmodel);
	leftobj->addGeometry(leftgeo);
	wc->add(leftobj);
	dwc->findBody("gripper.LeftFinger")->setObject(leftobj);
	
	Object* rightobj = new Object(wc->findFrame("gripper.RightFinger"));
	Geometry::Ptr rightgeo = ownedPtr(new Geometry(_jaw, string("RightFingerGeo")));
	Model3D* rightmodel = new Model3D("RightModel");
	rightmodel->addTriMesh(Model3D::Material("stlmat",0.8f,0.8f,0.6f), *rightgeo->getGeometryData()->getTriMesh() );
	rightmodel->setTransform(Transform3D<>(Vector3D<>(), RPY<>(0, 180*Deg2Rad, 180*Deg2Rad).toRotation3D()));
	rightgeo->setTransform(Transform3D<>(Vector3D<>(), RPY<>(0, 180*Deg2Rad, 180*Deg2Rad).toRotation3D()));
	rightobj->addModel(rightmodel);
	rightobj->addGeometry(rightgeo);
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



void Gripper::loadTasks(std::string filename)
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
}



/*void Gripper::updateGripper(TreeDevice::Ptr dev, rw::graphics::WorkCellScene::Ptr wcs, rw::proximity::CollisionDetector::Ptr cd)
{
	cout << "!" << endl;
	cout << dev->getName() << endl;
	//if (_device == 0) return;
	
	// update geometry
	// remove drawables from jaws
	wcs->removeDrawables(_wc->findFrame("gripper.LeftFinger"));
	wcs->removeDrawables(_wc->findFrame("gripper.RightFinger"));

	// remove models from collision detector
	BOOST_FOREACH (string st, cd->getGeometryIDs(_wc->findFrame("gripper.LeftFinger"))) {
		cout << "$$$" << st << endl;
	}
	
	cd->removeGeometry(_wc->findFrame("gripper.LeftFinger"), "LeftFingerGeo");
	cd->removeGeometry(_wc->findFrame("gripper.RightFinger"), "RightFingerGeo");
	
	// load stl files
	//rw::geometry::PlainTriMeshN1F::Ptr leftMesh = STLFile::load(directory + "/left.stl");
	//rw::geometry::PlainTriMeshN1F::Ptr rightMesh = STLFile::load(directory + "/right.stl");
	
	// set the proper pose of the fingers
	Geometry::Ptr leftGeo = ownedPtr(new Geometry(_jaw->createMesh()));
	leftGeo->setName("LeftFingerGeo");
	leftGeo->setTransform(Transform3D<>(Vector3D<>(), RPY<>(0.0, 0.0*Deg2Rad, 0.0).toRotation3D()));
	Geometry::Ptr rightGeo = ownedPtr(new Geometry(_jaw->createMesh()));
	rightGeo->setName("RightFingerGeo");
	rightGeo->setTransform(Transform3D<>(Vector3D<>(), RPY<>(0, 180.0*Deg2Rad, 180.0*Deg2Rad).toRotation3D()));
	
	// add new geometry
	wcs->addGeometry(
		"LeftFingerGeo",
		leftGeo,
		_wc->findFrame("gripper.LeftFinger"),
		Geometry::PhysicalGroup);
	wcs->addGeometry(
		"RightFingerGeo",
		rightGeo,
		_wc->findFrame("gripper.RightFinger"),
		Geometry::PhysicalGroup);
	
	// add new collision models
	cd->addGeometry(_wc->findFrame("gripper.LeftFinger"), leftGeo);
	cd->addGeometry(_wc->findFrame("gripper.RightFinger"), rightGeo);
	
	cout << "UPDATED" << endl;
}*/
