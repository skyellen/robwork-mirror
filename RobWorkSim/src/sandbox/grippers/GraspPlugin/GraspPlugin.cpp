#include "GraspPlugin.hpp"

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rw/graspplanning/GWSMeasure3D.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwsim/util/SurfacePoseSampler.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/common/Exception.hpp>
#include <boost/lexical_cast.hpp>
#include <rwlibs/algorithms/kdtree/KDTree.hpp>
#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>
#include <fstream>
#include <iostream>
#include "JawPrimitive.hpp"
#include "RenderTarget.hpp"
#include "FeatureTaskGenerator.hpp"
#include "RayTaskGenerator.hpp"
#include "DesignDialog.hpp"
#include "GripperTaskSimulator.hpp"
#include "GripperXMLLoader.hpp"



using namespace std;

USE_ROBWORK_NAMESPACE
using namespace robwork;

USE_ROBWORKSIM_NAMESPACE
using namespace robworksim;

using namespace rws;
using namespace rwlibs::proximitystrategies;
using namespace rwlibs::simulation;
using namespace rwlibs::task;



GraspPlugin::GraspPlugin() :
    RobWorkStudioPlugin("GraspPlugin", QIcon(":/pa_icon.png")),
    _wc(NULL),
    _dwc(NULL),
    _graspSim(NULL),
    _slowMotion(false),
    _showTasks(true),
    _silentMode(false),
    _nOfTargetsToGen(10),
    _tasks(NULL),
    _wd(""),
    _interferenceLimit(0.1),
    _wrenchLimit(0.0)
{
    setupGUI();
    
    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(updateSim()));
    
    _gripper = ownedPtr(new Gripper);
}



GraspPlugin::~GraspPlugin()
{
}



void GraspPlugin::initialize()
{
    getRobWorkStudio()->genericEvent().add(
		boost::bind(&GraspPlugin::genericEventListener, this, _1), this);
}



void GraspPlugin::startSimulation()
{
    _graspSim = ownedPtr(new GripperTaskSimulator(_dwc));
    _interferenceLimit = boost::lexical_cast<double>(_wc->getPropertyMap().get<string>("interferenceLimit"));
    _wrenchLimit = boost::lexical_cast<double>(_wc->getPropertyMap().get<string>("wrenchLimit"));
    _graspSim->setInterferenceLimit(_interferenceLimit);
    _graspSim->setWrenchLimit(_wrenchLimit);
    //_wc->getPropertyMap().get<string>("intLimit");

	// add interference objects
	istringstream objList(_wc->getPropertyMap().get<string>("interferenceObjects"));
	
	do {
		string objName;
		objList >> objName;
		cout << objName;
		Object::Ptr object = _wc->findObject(objName);
		if (object != NULL) {
			_graspSim->addInterferenceObject(object);
		} else {
			cout << "Null object" << endl;
		}
	} while (objList);
	
    //_graspSim->addInterferenceObject(_wc->findObject("ob1"));
   // _graspSim->addInterferenceObject(_wc->findObject("ob2"));
    
    if (!_graspSim->isRunning() && _gripper->getTasks() != NULL) {
		if (_gripper->getTasks() != NULL) setCurrentTask(_gripper->getTasks());
		
		try {
			_graspSim->startSimulation(_initState);
		} catch(...) {
			return;
		}
    
		_timer->start();
    }
}



void GraspPlugin::open(WorkCell* workcell)
{
    try {
		Math::seed(TimerUtil::currentTimeUs());
		
		_wc = workcell;
		
		_initState = getRobWorkStudio()->getState();
		
		_render = ownedPtr( new RenderTargets() );
		getRobWorkStudio()->getWorkCellScene()->addRender("pointRender", _render, workcell->getWorldFrame());
		
		
		
    } catch (const rw::common::Exception& e) {
		QMessageBox::critical(NULL, "RW Exception", e.what());
    }
    
    if (_dwc) {
		_dev = _wc->findDevice<TreeDevice>(_wc->getPropertyMap().get<string>("gripper"));
		_ddev = _dwc->findDevice<RigidDevice>(_wc->getPropertyMap().get<string>("gripper"));
        _graspSim = ownedPtr(new GripperTaskSimulator(_dwc));
        _generator = new RayTaskGenerator(_dwc, _wc->getPropertyMap().get<string>("target"), _wc->getPropertyMap().get<string>("gripper"));
	}
    
    try {
		//cout << "!!!!!" << _wc->getPropertyMap().size();
		//cout << "!!!!!" << _wc->getPropertyMap().get<string>("target");
		/*BOOST_FOREACH (PropertyBase::Ptr prop, _wc->getPropertyMap().getProperties()) {
			cout << prop->getIdentifier() << endl;
		}*/
	} catch (...) {}
}



void GraspPlugin::close()
{
    _timer->stop();
    
    if (_graspSim != NULL) {
        _graspSim->pauseSimulation();
    }
    
    _graspSim = NULL;
    _wc = NULL;
    _dwc = NULL;
    
    //_startButton->setEnabled(false);
    //_stopButton->setEnabled(false);
}



void GraspPlugin::guiEvent()
{
    QObject *obj = sender();
    
    if (obj == _startButton) {        
        startSimulation();
        
        //_startButton->setEnabled(false);
        //_stopButton->setEnabled(true);
    }
    
    else if (obj == _stopButton) {
		if (_graspSim->isRunning()) {
			_graspSim->pauseSimulation(); // there should be some way to stop the simulation
		}
		
		//_startButton->setEnabled(true);
		//_stopButton->setEnabled(false);
	}
	
	else if (obj == _slowCheck) {
		_slowMotion = _slowCheck->isChecked();
		
		// introduce step delay of 10ms when in slow motion state
		if (_graspSim == NULL) return;
		
		if (_slowMotion) {
			_graspSim->setStepDelay(10);
		} else {
			_graspSim->setStepDelay(0);
		}
	}
	
	else if (obj == _loadTaskButton) {		
		QString taskfile = QFileDialog::getOpenFileName(this,
			"Open file", "", tr("Task files (*.xml)"));
			
		if (taskfile.isEmpty()) return;
			
		log().info() << "Loading tasks from: " << taskfile.toStdString() << "\n";
		
		_gripper->setTasks(GraspTask::load(taskfile.toStdString()));
		_progressBar->setValue(0);
		_progressBar->setMaximum(_gripper->getTasks()->getAllTargets().size());
		
		if (_showCheck->isChecked()) showTasks(_gripper->getTasks());
	}
	
	else if (obj == _saveTaskButton) {
		QString taskfile = QFileDialog::getSaveFileName(this,
			"Save file", "", tr("Task files (*.xml)"));
			
		if (taskfile.isEmpty()) return;
				
		log().info() << "Saving tasks to: " << taskfile.toStdString() << endl;
		
		GraspTask::saveRWTask(_gripper->getTasks(), taskfile.toStdString());
	}
	
	else if (obj == _initialButton) { // return the workcell to the initial state
		getRobWorkStudio()->setState(_initState);
	}
	
	else if (obj == _approachButton) { // set the approach pose of the gripper
		_wTapproach = Kinematics::worldTframe(_wc->findFrame("TCPgripper"), getRobWorkStudio()->getState());
		log().info() << "Approach: " << inverse(_wTapproach) * _wTtarget << endl;
		
		_gripper->setTasks(NULL);
	}
	
	else if (obj == _targetButton) { // set the target pose of the gripper
		_wTtarget = Kinematics::worldTframe(_wc->findFrame("TCPgripper"), getRobWorkStudio()->getState());
		log().info() << "Target: " << _wTtarget << endl;
		
		_gripper->setTasks(NULL);
	}
	
	else if (obj == _generateButton) { // generate manual tasks
		_nOfTargetsToGen = _nManualEdit->text().toInt();
		
		log().info() << "Generating " << _nOfTargetsToGen << " tasks..." << endl;
		
		_gripper->setTasks(generateTasks(_nOfTargetsToGen));
		_progressBar->setValue(0);
		_progressBar->setMaximum(_nOfTargetsToGen);
		
		if (_showCheck->isChecked()) showTasks(_gripper->getTasks());
	}
	
	else if (obj == _nManualEdit) {
		// normalize the number in lineedit
		_nManualEdit->setText(QString::number(_nManualEdit->text().toInt()));
	}
	
	else if (obj == _planButton) {
		planTasks();
	}
	
	else if (obj == _showCheck) {
		_showTasks = _showCheck->isChecked();
		showTasks(_gripper->getTasks());
	}
	
	else if (obj == _silentCheck) {
		_silentMode = _silentCheck->isChecked();
		
		if (_silentMode) {
			BOOST_FOREACH (ThreadSimulator::Ptr sim, _graspSim->getSimulators()) {
				sim->setRealTimeScale(0.0);
			}
		} else {
			BOOST_FOREACH (ThreadSimulator::Ptr sim, _graspSim->getSimulators()) {
				sim->setRealTimeScale(0.0);
			}
		}
	}
	
	else if (obj == _loadGripperButton) {
		QString filename = QFileDialog::getOpenFileName(this,
			"Open file", QString::fromStdString(_wd), tr("Gripper files (*.xml)"));
			
		if (filename.isEmpty()) return;
			
		_wd = QFileInfo(filename).path().toStdString();
			
		_gripper = GripperXMLLoader::load(filename.toStdString());
		
		_progressBar->setValue(0);
		_progressBar->setMaximum(_gripper->getTasks()->getAllTargets().size());
		
		if (_showCheck->isChecked()) showTasks(_gripper->getTasks());
	}
	
	else if (obj == _saveGripperButton) {
		QString filename = QFileDialog::getSaveFileName(this,
			"Save file", QString::fromStdString(_wd), tr("Gripper files (*.xml)"));
			
		if (filename.isEmpty()) return;
			
		_wd = QFileInfo(filename).path().toStdString();
		string name = QFileInfo(filename).fileName().toStdString();
			
		GripperXMLLoader::save(_gripper, _wd, name);
	}
}



void GraspPlugin::designEvent()
{
	DesignDialog* ddialog = new DesignDialog(this, _gripper, _wd);
	ddialog->exec();
	
	_wd = ddialog->getWorkingDirectory();
	
	_gripper = ddialog->getGripper();
	
	//State state = _initState;
	_gripper->updateGripper(_wc, _dwc, _dev, _ddev, _initState);
	State tstate = _initState;
	
	getRobWorkStudio()->getWorkCellScene()->clearCache();
	getRobWorkStudio()->getWorkCellScene()->updateSceneGraph(_initState);
	getRobWorkStudio()->setWorkcell(_wc);
	//_graspSim = ownedPtr(new GripperTaskSimulator(_dwc));
	_initState = tstate;
	getRobWorkStudio()->setState(_initState);
	
	if (_autoPlanCheck->isChecked()) planTasks();
	
	_progressBar->setValue(0);
	if (_gripper->getTasks() != NULL) {
		_progressBar->setMaximum(_gripper->getTasks()->getAllTargets().size());
	} else {
		_progressBar->setMaximum(0);
	}
}



void GraspPlugin::updateSim()
{
	if (_graspSim == NULL || _wc == NULL || _dwc == NULL) return;
	
	if (!_silentMode) getRobWorkStudio()->setState(_graspSim->getSimulator()->getState());
	
	// check out the number of tasks already performed and update progress bar accordingly
	_progressBar->setValue(_graspSim->getNrTargetsDone());

	if (!_graspSim->isRunning()) {
		_timer->stop();
		
		//_startButton->setEnabled(true);
		//_stopButton->setEnabled(false);
		
		double quality = calculateQuality(_gripper->getTasks(), _generator->getAllSamples());
		
		//TaskGenerator::filterTasks(_tasks, Q(7, 0.5, 0.5, 0.5, 1.5, 1.5, 1.5, 1000*Deg2Rad));
		
		//log().info() << "GripperQ= " << quality << endl;
		
		// we should print out the results of simulation here:
		/*int n = 0;
		
		BOOST_FOREACH(GraspTarget tgt, _tasks->getSubTasks()[0].getTargets()) {
			log().info() << "Task " << ++n << ". res: " << tgt.getResult()->testStatus;
			log().info() << " qbefore: " << tgt.getResult()->qualityBeforeLifting << " qafter: " << tgt.getResult()->qualityAfterLifting << endl;
		}*/
	}
	
	if (_showCheck->isChecked()) showTasks(_gripper->getTasks());
}



GraspTask::Ptr GraspPlugin::generateTasks(int nTasks)
{
    GraspTask::Ptr task = ownedPtr(new GraspTask());

    task->setTCPID(_wc->getPropertyMap().get<string>("gripperTCP"));
    task->setGripperID(_wc->getPropertyMap().get<string>("gripper"));
    task->setGraspControllerID(_wc->getPropertyMap().get<string>("controller"));

    task->getSubTasks().resize(1);
    GraspSubTask &subtask = task->getSubTasks()[0];
 
    subtask.openQ = Q(1, 0.0);
    subtask.closeQ = Q(1, 0.05);	
    
    subtask.approach = inverse(_wTapproach) * _wTtarget;
    subtask.retract = inverse(_wTapproach) * _wTtarget;
    
    for (int i = 0; i < nTasks; ++i) {		
		subtask.targets.push_back(GraspTarget(_wTtarget));
    }

    return task;
}



void GraspPlugin::planTasks()
{
	_nOfTargetsToGen = _nAutoEdit->text().toInt();
		
	log().info() << "Generating " << _nOfTargetsToGen << " tasks..." << endl;
	
	try {
		_gripper->setTasks(_generator->generateTask(_nOfTargetsToGen, getRobWorkStudio()->getCollisionDetector(), _initState));
		_gripper->getQuality()->nOfSampledParSurfaces = _generator->getSamples();
	} catch (rw::common::Exception& e) {
		QMessageBox::critical(NULL, "RW Exception", e.what());
	}
	
	log().info() << "Done." << endl;
	
	_progressBar->setValue(0);
	_progressBar->setMaximum(_nOfTargetsToGen);
	
	//TaskGenerator::filterTasks(_tasks, Q(7, 0.5, 0.5, 0.5, 1.5, 1.5, 1.5, 1000*Deg2Rad));
	//TaskGenerator::filterTasks(_tasks, Q(7, 10, 10, 10, 20, 20, 20, 500*Deg2Rad));
	//TaskGenerator::filterTasks(_tasks, Q(7, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 25*Deg2Rad));
	
	if (_showCheck->isChecked()) showTasks(_gripper->getTasks());
}



void GraspPlugin::setCurrentTask(GraspTask::Ptr task)
{
    try {
        _graspSim->load(task);
    } catch(const Exception& e) {
        QMessageBox::information(this, "GraspPlugin", e.what());
        return;
    } catch(...) {
		cout << "Unable to load tasks!" << endl;
	}
}



void GraspPlugin::genericEventListener(const std::string& event)
{
    if (event == "DynamicWorkcellLoadet") { // why is that in Danish?
        // get the dynamic workcell from the propertymap
        RW_DEBUG("Getting dynamic workcell from propertymap!");
	 
        DynamicWorkCell::Ptr dwc =
			getRobWorkStudio()->getPropertyMap().get<DynamicWorkCell::Ptr>("DynamicWorkcell", NULL);

        if (dwc == NULL) {
            log().error() << "Could not load dynamic workcell from propertymap!" << std::endl;
            return;
        }
        
        _dwc = dwc;
        
        
        
        //cout << "Obiekty w WC:" << _wc->getObjects().size() << endl;
        /*BOOST_FOREACH (Object::Ptr obj, _wc->getObjects()) {
			cout << "!" << obj->getName() << endl;
		}*/
        
        _startButton->setEnabled(true);
    } 
}



double GraspPlugin::calculateQuality(rwlibs::task::GraspTask::Ptr tasks, rwlibs::task::GraspTask::Ptr allTasks)
{
	int all = 0;
	int success = 0;
	Q gws(3);
	
	// calculate coverage
	BOOST_FOREACH(GraspTarget& tgt, tasks->getSubTasks()[0].getTargets()) {
		if (tgt.getResult()->testStatus != GraspTask::UnInitialized) // only take into account performed grasps
			all++;
		
		if (tgt.getResult()->testStatus == GraspTask::Success) {
			success++;
			gws += tgt.getResult()->qualityAfterLifting;
		}
	}
	
	double quality = 0.0;
	double sim = 1.0 * success/all;
	gws = gws / success;
	double cvg = calculateCoverage(tasks, allTasks, Q(7, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 25*Deg2Rad));
	
	log().info() << "SIM= " << sim << endl;
	log().info() << "GWS= " << gws << endl;
	log().info() << "CVG= " << cvg << endl;
	
	// set gripper data appropriately
	_gripper->getQuality()->successQ = sim;
	_gripper->getQuality()->coverageQ = cvg;
	
	return quality;
}



double GraspPlugin::calculateCoverage(rwlibs::task::GraspTask::Ptr tasks, rwlibs::task::GraspTask::Ptr allTasks, rw::math::Q diff)
{
	double coverage = 0.0;
	
	int okTargets = filterAndCount(tasks, diff);
	int allTargets = filterAndCount(allTasks, diff);
	
	log().info() << "N of tasks: " << tasks->getSubTasks()[0].getTargets().size() << " / N of all samples: " << allTasks->getSubTasks()[0].getTargets().size() << endl;
	log().info() << "Filtered grasps: " << okTargets << " / Parallel samples: " << allTargets << endl;
	coverage = 1.0 * okTargets / allTargets;
	
	return coverage;
}



int GraspPlugin::filterAndCount(rwlibs::task::GraspTask::Ptr tasks, rw::math::Q diff) const
{
	// create nodes for succesful grasps
	typedef GraspResult::Ptr ValueType;
	typedef KDTreeQ<ValueType> NNSearch;
	vector<NNSearch::KDNode> nodes;
	
	BOOST_FOREACH(GraspTarget& target, tasks->getSubTasks()[0].getTargets()) {
		if (target.getResult()->testStatus == GraspTask::Success) {
			Q key(7);
            key[0] = target.pose.P()[0];
            key[1] = target.pose.P()[1];
            key[2] = target.pose.P()[2];
            EAA<> eaa(target.pose.R());
            key[3] = eaa.axis()(0);
            key[4] = eaa.axis()(1);
            key[5] = eaa.axis()(2);
            key[6] = eaa.angle();
            //key[7] = 0; // this is to remove neighbouring grasps
            
            //cout << key << endl;
            
			nodes.push_back(NNSearch::KDNode(key, target.getResult()));
		}
	}
	
	NNSearch *nntree = NNSearch::buildTree(nodes);
	//Q diff(7, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 15*Deg2Rad);
    std::list<const NNSearch::KDNode*> result;
    
    int nFiltered = 0;
    BOOST_FOREACH (NNSearch::KDNode& node, nodes) {
		if (node.value->testStatus != GraspTask::TimeOut) {
			result.clear();
			Q key = node.key;
			nntree->nnSearchRect(key-diff, key+diff, result);
			//cout << result.size() << " of neighbours" << endl;

			int g = 0;
			BOOST_FOREACH (const NNSearch::KDNode* n, result) {
				if (n->value->testStatus != GraspTask::TimeOut) ++g;
				const_cast<NNSearch::KDNode*>(n)->value->testStatus = GraspTask::TimeOut;
				//cout << n->value->testStatus << endl;
				
			}
			nFiltered += g;
			//cout << "Deleted " << g << " neighbours; left: " << nodes.size() - nFiltered << endl;
			
			//++nFiltered;
		}
	}
	
	return nodes.size() - nFiltered;
}



/*void GraspPlugin::loadGeometry(std::string directory)
{
	// remove drawables from jaws
	getRobWorkStudio()->getWorkCellScene()->removeDrawables(_wc->findFrame("gripper.LeftFinger"));
	getRobWorkStudio()->getWorkCellScene()->removeDrawables(_wc->findFrame("gripper.RightFinger"));

	// remove models from collision detector
	CollisionDetector::Ptr detector = getRobWorkStudio()->getCollisionDetector();
	detector->removeGeometry(_wc->findFrame("gripper.LeftFinger"), "LeftFingerGeo");
	detector->removeGeometry(_wc->findFrame("gripper.RightFinger"), "RightFingerGeo");
	
	// load stl files
	rw::geometry::PlainTriMeshN1F::Ptr leftMesh = STLFile::load(directory + "/left.stl");
	rw::geometry::PlainTriMeshN1F::Ptr rightMesh = STLFile::load(directory + "/right.stl");
	
	// set the proper pose of the fingers
	Geometry::Ptr leftGeo = ownedPtr(new Geometry(leftMesh));
	leftGeo->setName("LeftFingerGeo");
	leftGeo->setTransform(Transform3D<>(Vector3D<>(), RPY<>(0.0, 0.0*Deg2Rad, 0.0).toRotation3D()));
	Geometry::Ptr rightGeo = ownedPtr(new Geometry(rightMesh));
	rightGeo->setName("RightFingerGeo");
	rightGeo->setTransform(Transform3D<>(Vector3D<>(), RPY<>(0, 180.0*Deg2Rad, 180.0*Deg2Rad).toRotation3D()));
	
	// add new geometry
	getRobWorkStudio()->getWorkCellScene()->addGeometry(
		"LeftFingerGeo",
		leftGeo,
		_wc->findFrame("gripper.LeftFinger"),
		Geometry::PhysicalGroup);
	getRobWorkStudio()->getWorkCellScene()->addGeometry(
		"RightFingerGeo",
		rightGeo,
		_wc->findFrame("gripper.RightFinger"),
		Geometry::PhysicalGroup);
	
	// add new collision models
	detector->addGeometry(_wc->findFrame("gripper.LeftFinger"), leftGeo);
	detector->addGeometry(_wc->findFrame("gripper.RightFinger"), rightGeo);
}*/



void GraspPlugin::showTasks(rwlibs::task::GraspTask::Ptr tasks)
{
	vector<RenderTargets::Target> rtargets;
	Transform3D<> wTo = Kinematics::worldTframe(_wc->findFrame(_wc->getPropertyMap().get<string>("target")), _wc->getDefaultState());
	
	if (tasks == NULL) { return; }
	
	if (_showTasks) {
		BOOST_FOREACH (GraspTarget& target, tasks->getSubTasks()[0].getTargets()) {
			RenderTargets::Target rt;
			rt.ctask = &tasks->getSubTasks()[0];
			rt.ctarget = target;
			
			if (target.getResult()->testStatus == GraspTask::UnInitialized) {
				rt.color[0] = 1.0;
				rt.color[1] = 1.0;
				rt.color[2] = 1.0;
				rt.color[3] = 0.5;
			}
			else if (target.getResult()->testStatus == GraspTask::Success) {
				rt.color[0] = 0.0;
				rt.color[1] = 1.0;
				rt.color[2] = 0.0;
				rt.color[3] = 0.5;
			} else {
				rt.color[0] = 1.0;
				rt.color[1] = 0.0;
				rt.color[2] = 0.0;
				rt.color[3] = 0.5;
			}
			
			rt.trans = wTo * target.pose;
			
			rtargets.push_back(rt);
		}
	}
	
	//cout << "Painting!" << endl;
	
	((RenderTargets*)_render.get())->setTargets(rtargets);
    getRobWorkStudio()->postUpdateAndRepaint();
}



void GraspPlugin::setupGUI()
{
	int row = 0;
	    
	// setup base widget
	QWidget* base = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(base);
    
    base->setLayout(layout);
    this->setWidget(base);
    
    /* setup geometry group */
    _geometryBox = new QGroupBox("Gripper parametrization");
    QGridLayout* geoLayout = new QGridLayout(_geometryBox);
    _geometryBox->setLayout(geoLayout);
    
    row = 0;
    
    _designButton = new QPushButton("Design gripper");
    geoLayout->addWidget(_designButton, row++, 0, 1, 2);
    connect(_designButton, SIGNAL(clicked()), this, SLOT(designEvent()));
    
    _loadGripperButton = new QPushButton("Load gripper");
    geoLayout->addWidget(_loadGripperButton, row, 0);
    connect(_loadGripperButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _saveGripperButton = new QPushButton("Save gripper");
    geoLayout->addWidget(_saveGripperButton, row++, 1);
    connect(_saveGripperButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _autoPlanCheck = new QCheckBox("Auto. plan");
    geoLayout->addWidget(_autoPlanCheck, row++, 0, 1, 2);
    
    /* setup manual group */
    /*_manualBox = new QGroupBox("Manual task setup");
    QGridLayout* manualLayout = new QGridLayout(_manualBox);
    _manualBox->setLayout(manualLayout);
    
    row = 0;
    
    _approachButton = new QPushButton("Set approach");
    manualLayout->addWidget(_approachButton, row, 0);
    connect(_approachButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _targetButton = new QPushButton("Set target");
    manualLayout->addWidget(_targetButton, row++, 1);
    connect(_targetButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _generateButton = new QPushButton("Generate");
    manualLayout->addWidget(_generateButton, row, 0);
    connect(_generateButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _nManualEdit = new QLineEdit("10");
    manualLayout->addWidget(_nManualEdit, row++, 1);
    connect(_nManualEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));*/
    
    /* setup auto group */
    _autoBox = new QGroupBox("Auto task setup");
    QGridLayout* autoLayout = new QGridLayout(_autoBox);
    _autoBox->setLayout(autoLayout);
    
    row = 0;
    
    _planButton = new QPushButton("Plan");
    autoLayout->addWidget(_planButton, row, 0);
    connect(_planButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _nAutoEdit = new QLineEdit("100");
    autoLayout->addWidget(_nAutoEdit, row++, 1);
    connect(_nAutoEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
    
    /* setup sim control group */
    _simBox = new QGroupBox("Simulation control");
    QGridLayout* simLayout = new QGridLayout(_simBox);
    _simBox->setLayout(simLayout);
    
    row = 0;
    
    _initialButton = new QPushButton("Set initial state");
    simLayout->addWidget(_initialButton, row++, 0, 1, 2);
    connect(_initialButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _loadTaskButton = new QPushButton("Load tasks");
    simLayout->addWidget(_loadTaskButton, row, 0);
    connect(_loadTaskButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _saveTaskButton = new QPushButton("Save tasks");
    simLayout->addWidget(_saveTaskButton, row++, 1);
    connect(_saveTaskButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _showCheck = new QCheckBox("Show tasks");
    _showCheck->setChecked(true);
    simLayout->addWidget(_showCheck, row++, 0, 1, 2);
    connect(_showCheck, SIGNAL(clicked()), this, SLOT(guiEvent()) );
    
    _progressBar = new QProgressBar;
    _progressBar->setFormat("%v of %m");
    simLayout->addWidget(_progressBar, row++, 0, 1, 2);
    
    _startButton = new QPushButton("START");
    simLayout->addWidget(_startButton, row, 0);
    connect(_startButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _stopButton = new QPushButton("STOP");
    simLayout->addWidget(_stopButton, row++, 1);
    connect(_stopButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _slowCheck = new QCheckBox("Slow motion");
    simLayout->addWidget(_slowCheck, row, 0);
    connect(_slowCheck, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _silentCheck = new QCheckBox("Silent mode");
    simLayout->addWidget(_silentCheck, row++, 1);
    connect(_silentCheck, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    /* add groups to the base layout */
    layout->addWidget(_geometryBox);
    //layout->addWidget(_manualBox);
    layout->addWidget(_autoBox);
    layout->addWidget(_simBox);
    layout->addStretch(1);
}



/*void GraspPlugin::addGripper(rw::math::Transform3D<> transform)
{
	Frame* world = _wc->getWorldFrame();
	StateStructure::Ptr tree = _wc->getStateStructure();
	
	// remove existing gripper
	//Device::Ptr 
	
	// make base
	MovableFrame* base = new MovableFrame("Base");
	
	// make joints
	PrismaticJoint* left = new PrismaticJoint("LeftFinger", Transform3D<>(Vector3D<>(), RPY<>(0, -90*Deg2Rad, 0).toRotation3D()));
	DependentPrismaticJoint* right = new DependentPrismaticJoint(
		"RightFinger", Transform3D<>(Vector3D<>(), RPY<>(0, 90*Deg2Rad, 0).toRotation3D()),
		left, 1, 0);
		
	// make end effectors
	FixedFrame* leftend = new FixedFrame("TCPLeftFinger", Transform3D<>());
	FixedFrame* rightend = new FixedFrame("TCPRightFinger", Transform3D<>());
	
	
	
	// add objects to frames
	Geometry::Ptr basegeo = ownedPtr(new Geometry(new Box(0.15, 0.1, 0.05), string("BaseGeo")));
	Geometry::Ptr leftgeo = ownedPtr(new Geometry(new JawPrimitive(), string("LeftFingerGeo")));
	Geometry::Ptr rightgeo = ownedPtr(new Geometry(new JawPrimitive(), string("RightFingerGeo")));
	
	Object* baseobj = new Object(base);
	Model3D* basemodel = new Model3D("BaseModel");
	basemodel->addTriMesh(Model3D::Material("stlmat",0.6f,0.6f,0.6f), *basegeo->getGeometryData()->getTriMesh() );
	basemodel->setTransform(Transform3D<>(Vector3D<>::z()*-0.025, Rotation3D<>()));
	basegeo->setTransform(Transform3D<>(Vector3D<>::z()*-0.025, Rotation3D<>()));
	basegeo->setFrame(base);
	baseobj->addModel(basemodel);
	baseobj->addGeometry(basegeo);
	_wc->add(baseobj);
	
	Object* leftobj = new Object(left);
	Model3D* leftmodel = new Model3D("LeftModel");
	leftmodel->addTriMesh(Model3D::Material("stlmat",0.6f,0.6f,0.6f), *leftgeo->getGeometryData()->getTriMesh() );
	leftmodel->setTransform(Transform3D<>());
	leftgeo->setTransform(Transform3D<>());
	leftgeo->setFrame(left);
	leftobj->addModel(leftmodel);
	leftobj->addGeometry(leftgeo);
	_wc->add(leftobj);
	
	Object* rightobj = new Object(right);
	Model3D* rightmodel = new Model3D("RightModel");
	rightmodel->addTriMesh(Model3D::Material("stlmat",0.6f,0.6f,0.6f), *rightgeo->getGeometryData()->getTriMesh() );
	rightmodel->setTransform(Transform3D<>(Vector3D<>(), RPY<>(0, 180*Deg2Rad, 180*Deg2Rad).toRotation3D()));
	rightgeo->setTransform(Transform3D<>(Vector3D<>(), RPY<>(0, 180*Deg2Rad, 180*Deg2Rad).toRotation3D()));
	rightgeo->setFrame(right);
	rightobj->addModel(rightmodel);
	rightobj->addGeometry(rightgeo);
	_wc->add(rightobj);
	
	
	// add frames to the state structure
	tree->addFrame(base, world);
	tree->addFrame(left, base);
	tree->addFrame(right, base);
	tree->addFrame(leftend, left);
	tree->addFrame(rightend, right);
	
	getRobWorkStudio()->getWorkCellScene()->addGeometry(
		"BaseGeo",
		basegeo,
		base,
		Geometry::PhysicalGroup);
	getRobWorkStudio()->getWorkCellScene()->addGeometry(
		"LeftFingerGeo",
		leftgeo,
		left,
		Geometry::PhysicalGroup);
	getRobWorkStudio()->getWorkCellScene()->addGeometry(
		"RightFingerGeo",
		rightgeo,
		right,
		Geometry::PhysicalGroup);
	
	getRobWorkStudio()->getCollisionDetector()->addGeometry(base, basegeo);
	getRobWorkStudio()->getCollisionDetector()->addGeometry(left, leftgeo);
	getRobWorkStudio()->getCollisionDetector()->addGeometry(right, rightgeo);
	
	// create device
	vector<Frame*> ends;
	ends.push_back(leftend);
	ends.push_back(rightend);
	
	State state = tree->getDefaultState();
	TreeDevice::Ptr gripper = new TreeDevice(base, ends, "gripper", state);
	Q x1(1); x1[0] = 0;
	Q x2(1); x2[0] = 0.05;
	gripper->setBounds(make_pair(x1, x2));
	_wc->addDevice(gripper);
	
	
	
	
	base->setTransform(Transform3D<>(Vector3D<>(0, 0, 0.3), RPY<>(0, 0, 180*Deg2Rad).toRotation3D()), state);
	
	
	getRobWorkStudio()->setWorkCell(_wc);
	
	// update collision detector
	CollisionDetector::Ptr cd = getRobWorkStudio()->getCollisionDetector();
	cd->addRule(ProximitySetupRule::makeExclude("Base", "LeftFinger"));
	cd->addRule(ProximitySetupRule::makeExclude("Base", "RightFinger"));
	cd->addRule(ProximitySetupRule::makeExclude("RightFinger", "LeftFinger"));
	
	
	getRobWorkStudio()->setState(state);
	
	//getRobWorkStudio()->updateAndRepaint();
}*/



Q_EXPORT_PLUGIN(GraspPlugin);
