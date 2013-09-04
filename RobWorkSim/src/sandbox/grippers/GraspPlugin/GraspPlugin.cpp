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
#include <fstream>
#include <iostream>



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
    _nOfTargetsToGen(10),
    _tasks(NULL)
{
    setupGUI();
    
    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(updateSim()));
    
    // set up the initial gripper position
    // IN THE WORKCELL
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
    if(_graspSim == NULL) {
        QMessageBox::information(this, "SimTaskPlugin", "Grasp simulator has not been created yet!");
        return;
    }
    
    if (!_graspSim->isRunning()) {
		if (_tasks != NULL) setCurrentTask(_tasks);
		
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
		_graspSim = ownedPtr(new GraspTaskSimulator(_dwc));
    
		_initState = getRobWorkStudio()->getState();
		_detector = getRobWorkStudio()->getCollisionDetector();
    } catch (const rw::common::Exception& e) {
		QMessageBox::critical(NULL, "RW Exception", e.what());
    }
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
    
    _startButton->setEnabled(false);
    _stopButton->setEnabled(false);
}



void GraspPlugin::guiEvent()
{
    QObject *obj = sender();
    
    if (obj == _startButton) {        
        startSimulation();
        
        _startButton->setEnabled(false);
        _stopButton->setEnabled(true);
    }
    
    else if (obj == _stopButton) {
		if (_graspSim->isRunning()) {
			_graspSim->pauseSimulation(); // there should be some way to stop the simulation
		}
		
		_startButton->setEnabled(true);
		_stopButton->setEnabled(false);
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
	
	else if (obj == _loadGeoButton) {
		// here we are going to open file dialog to choose stl directory for fingers
		QString directory = QFileDialog::getExistingDirectory(this,
			"Open gripper geometry directory", "", QFileDialog::ShowDirsOnly);
		
		if (directory.isEmpty()) return;
		
		log().info() << "Loading STL files from: " << directory.toStdString() << "\n";
		
		loadGeometry(directory.toStdString());
	}
	
	else if (obj == _loadTaskButton) {		
		QString taskfile = QFileDialog::getOpenFileName(this,
			"Open file", "", tr("Task files (*.xml)"));
			
		if (taskfile.isEmpty()) return;
			
		log().info() << "Loading tasks from: " << taskfile.toStdString() << "\n";
		
		_tasks = GraspTask::load(taskfile.toStdString());
		_progressBar->setValue(0);
		_progressBar->setMaximum(_tasks->getAllTargets().size());
	}
	
	else if (obj == _saveTaskButton) {
		QString taskfile = QFileDialog::getSaveFileName(this,
			"Save file", "", tr("Task files (*.xml)"));
			
		if (taskfile.isEmpty()) return;
				
		log().info() << "Saving tasks to: " << taskfile.toStdString() << endl;
		
		GraspTask::saveRWTask(_tasks, taskfile.toStdString());
	}
	
	else if (obj == _initialButton) { // return the workcell to the initial state
		getRobWorkStudio()->setState(_initState);
	}
	
	else if (obj == _approachButton) { // set the approach pose of the gripper
		_wTapproach = Kinematics::worldTframe(_wc->findFrame("TCPgripper"), getRobWorkStudio()->getState());
		log().info() << "Approach: " << inverse(_wTapproach) * _wTtarget << endl;
		
		_tasks = NULL;
	}
	
	else if (obj == _targetButton) { // set the target pose of the gripper
		_wTtarget = Kinematics::worldTframe(_wc->findFrame("TCPgripper"), getRobWorkStudio()->getState());
		log().info() << "Target: " << _wTtarget << endl;
		
		_tasks = NULL;
	}
	
	else if (obj == _generateButton) { // generate manual tasks
		_nOfTargetsToGen = _nManualEdit->text().toInt();
		
		log().info() << "Generating " << _nOfTargetsToGen << " tasks..." << endl;
		
		_tasks = generateTasks(_nOfTargetsToGen);
		_progressBar->setValue(0);
		_progressBar->setMaximum(_nOfTargetsToGen);
	}
	
	else if (obj == _nManualEdit) {
		// normalize the number in lineedit
		_nManualEdit->setText(QString::number(_nManualEdit->text().toInt()));
	}
	
	else if (obj == _planButton) {
		_nOfTargetsToGen = _nAutoEdit->text().toInt();
		
		log().info() << "Generating " << _nOfTargetsToGen << " tasks..." << endl;
		
		_tasks = _generator->generateTask(_nOfTargetsToGen);
		
		log().info() << "Done." << endl;
		
		_progressBar->setValue(0);
		_progressBar->setMaximum(_nOfTargetsToGen);
	}
}



void GraspPlugin::updateSim()
{
	if (_graspSim == NULL || _wc == NULL || _dwc == NULL) return;
	
	getRobWorkStudio()->setState(_graspSim->getSimulator()->getState());
	
	// check out the number of tasks already performed and update progress bar accordingly
	_progressBar->setValue(_graspSim->getNrTargetsDone());
	
	
	if (_graspSim->isFinished()) {
		_timer->stop();
		
		_startButton->setEnabled(true);
		_stopButton->setEnabled(false);
		
		// we should print out the results of simulation here:
		int n = 0;
		
		BOOST_FOREACH(GraspTarget tgt, _tasks->getSubTasks()[0].getTargets()) {
			log().info() << "Task " << ++n << ". res: " << tgt.getResult()->testStatus;
			log().info() << " qbefore: " << tgt.getResult()->qualityBeforeLifting << " qafter: " << tgt.getResult()->qualityAfterLifting << endl;
		}
	}
}



GraspTask::Ptr GraspPlugin::generateTasks(int nTasks)
{
    GraspTask::Ptr task = ownedPtr(new GraspTask());

    task->setTCPID("TCPgripper");
    task->setGripperID("gripper");
    task->setGraspControllerID("graspController");

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
        _generator = new TaskGenerator(_dwc, "object", "gripper");
        
        _startButton->setEnabled(true);
    } 
}



void GraspPlugin::loadGeometry(std::string directory)
{
	// remove drawables from jaws
	getRobWorkStudio()->getWorkCellScene()->removeDrawables(_wc->findFrame("gripper.LeftFinger"));
	getRobWorkStudio()->getWorkCellScene()->removeDrawables(_wc->findFrame("gripper.RightFinger"));
	
	BOOST_FOREACH (string id, _detector->getGeometryIDs(_wc->findFrame("gripper.LeftFinger"))) {
		cout << id << endl;
	}
	
	// remove models from collision detector
	_detector->removeGeometry(_wc->findFrame("gripper.LeftFinger"), "LeftFingerGeo");
	_detector->removeGeometry(_wc->findFrame("gripper.RightFinger"), "RightFingerGeo");
	
	// load stl files
	rw::geometry::PlainTriMeshN1F::Ptr leftMesh = STLFile::load(directory + "/left.stl");
	rw::geometry::PlainTriMeshN1F::Ptr rightMesh = STLFile::load(directory + "/right.stl");
	
	// set the proper pose of the fingers
	Geometry::Ptr leftGeo = ownedPtr(new Geometry(leftMesh));
	leftGeo->setName("LeftFingerGeo");
	leftGeo->setTransform(Transform3D<>(Vector3D<>(), RPY<>(0.0, 90.0*Deg2Rad, 0.0).toRotation3D()));
	Geometry::Ptr rightGeo = ownedPtr(new Geometry(rightMesh));
	rightGeo->setName("RightFingerGeo");
	rightGeo->setTransform(Transform3D<>(Vector3D<>(), RPY<>(180.0*Deg2Rad, -90.0*Deg2Rad, 0.0).toRotation3D()));
	
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
	_detector->addGeometry(_wc->findFrame("gripper.LeftFinger"), leftGeo);
	_detector->addGeometry(_wc->findFrame("gripper.RightFinger"), rightGeo);
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
    connect(_designButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _loadGeoButton = new QPushButton("Load gripper");
    geoLayout->addWidget(_loadGeoButton, row, 0);
    connect(_loadGeoButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _saveGeoButton = new QPushButton("Save gripper");
    geoLayout->addWidget(_saveGeoButton, row++, 1);
    connect(_saveGeoButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    /* setup manual group */
    _manualBox = new QGroupBox("Manual task setup");
    QGridLayout* manualLayout = new QGridLayout(_manualBox);
    _manualBox->setLayout(manualLayout);
    
    row = 0;
    
    _initialButton = new QPushButton("Set initial state");
    manualLayout->addWidget(_initialButton, row++, 0, 1, 2);
    connect(_initialButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
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
    connect(_nManualEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
    
    /* setup auto group */
    _autoBox = new QGroupBox("Auto task setup");
    QGridLayout* autoLayout = new QGridLayout(_autoBox);
    _autoBox->setLayout(autoLayout);
    
    row = 0;
    
    _planButton = new QPushButton("Plan");
    autoLayout->addWidget(_planButton, row, 0);
    connect(_planButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _nAutoEdit = new QLineEdit("10");
    autoLayout->addWidget(_nAutoEdit, row++, 1);
    connect(_nAutoEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
    
    /* setup sim control group */
    _simBox = new QGroupBox("Simulation control");
    QGridLayout* simLayout = new QGridLayout(_simBox);
    _simBox->setLayout(simLayout);
    
    row = 0;
    
    _loadTaskButton = new QPushButton("Load tasks");
    simLayout->addWidget(_loadTaskButton, row, 0);
    connect(_loadTaskButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _saveTaskButton = new QPushButton("Save tasks");
    simLayout->addWidget(_saveTaskButton, row++, 1);
    connect(_saveTaskButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
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
    simLayout->addWidget(_slowCheck, row++, 0, 1, 2);
    connect(_slowCheck, SIGNAL(clicked()), this, SLOT(guiEvent()) );
    
    /* add groups to the base layout */
    layout->addWidget(_geometryBox);
    layout->addWidget(_manualBox);
    layout->addWidget(_autoBox);
    layout->addWidget(_simBox);
}



Q_EXPORT_PLUGIN(GraspPlugin);
