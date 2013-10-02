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
#include "TaskGenerator.hpp"
#include "DesignDialog.hpp"
#include "GripperTaskSimulator.hpp"
#include "GripperXMLLoader.hpp"
#include "TaskDialog.hpp"



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
    _td(NULL),
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
	/* Initialization is basically only adding an event listener to the plugin,
	 * so we know when another DynamicWorkCell is loaded.
	 */
    getRobWorkStudio()->genericEvent().add(
		boost::bind(&GraspPlugin::genericEventListener, this, _1), this);
}



void GraspPlugin::startSimulation()
{
    _graspSim = ownedPtr(new GripperTaskSimulator(_gripper, _tasks, _samples, _td));
    
	try {
		_graspSim->startSimulation(_initState);
	} catch(...) {
		return;
	}
    
	_timer->start();
}



void GraspPlugin::open(WorkCell* workcell)
{
    try {
		Math::seed(TimerUtil::currentTimeUs());
		
		_wc = workcell;
		if (_wc->getPropertyMap().has("taskDescription")) {
			string filename = _wc->getPropertyMap().get<string>("taskDescription");
			log().info() << "Loading task description from: " << filename << endl;
			cout << "Loading task description from: " << filename << endl;
			_td = TaskDescriptionLoader::load(filename, _dwc);
			cout << "Loaded task description." << endl;
		}
		
		_initState = getRobWorkStudio()->getState();
		
		_render = ownedPtr( new RenderTargets() );
		getRobWorkStudio()->getWorkCellScene()->addRender("pointRender", _render, workcell->getWorldFrame());
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
}



void GraspPlugin::addHint()
{
	Transform3D<> target = Kinematics::worldTframe(_td->getGripperTCP(), getRobWorkStudio()->getState());
	log().info() << "Added hint grasp: " << target << endl;
	if (_td) {
		_td->addHint(target);
	}
}



void GraspPlugin::clearHints()
{
	_td->getHints().clear();
}



void GraspPlugin::guiEvent(int i)
{
	_gripper = _gripperList[i];
	
	updateGripper();
}



void GraspPlugin::guiEvent()
{
    QObject *obj = sender();
    
    if (obj == _startButton) {        
        startSimulation();
    }
    
    else if (obj == _stopButton) {
		if (_graspSim->isRunning()) {
			_graspSim->pauseSimulation(); // there should be some way to stop the simulation
		}
	}
	
	else if (obj == _loadTaskButton) {		
		QString taskfile = QFileDialog::getOpenFileName(this,
			"Open file", "", tr("Task files (*.xml)"));
			
		if (taskfile.isEmpty()) return;
			
		log().info() << "Loading tasks from: " << taskfile.toStdString() << "\n";
		
		_tasks = GraspTask::load(taskfile.toStdString());
		_progressBar->setValue(0);
		_progressBar->setMaximum(_tasks->getAllTargets().size());
		
		if (_showCheck->isChecked()) showTasks(_tasks);
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
		
		//_gripper->setTasks(NULL);
	}
	
	else if (obj == _targetButton) { // set the target pose of the gripper
		_wTtarget = Kinematics::worldTframe(_wc->findFrame("TCPgripper"), getRobWorkStudio()->getState());
		log().info() << "Target: " << _wTtarget << endl;
		
		//_gripper->setTasks(NULL);
	}
	
	else if (obj == _planButton) {
		planTasks();
	}
	
	else if (obj == _showCheck) {
		_showTasks = _showCheck->isChecked();
		showTasks(_tasks);
	}
	
	else if (obj == _loadGripperButton) {		
		QStringList files = QFileDialog::getOpenFileNames(this,
			"Open file", QString::fromStdString(_wd), tr("Gripper files (*.grp.xml)"));
		if (files.isEmpty()) return;
		
		_gripperCombo->disconnect(this);
		QStringList f = files;
		QStringList::Iterator it = f.begin();
		while (it != f.end()) {
			cout << "Loading: " << it->toStdString() << endl;
			loadGripper(it->toStdString());
			_wd = QFileInfo(*it).path().toStdString();
			++it;
		}
		connect(_gripperCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(guiEvent(int)));
		_gripperCombo->setCurrentIndex(0);
		
		/*_progressBar->setValue(0);
		_progressBar->setMaximum(_gripper->getTasks()->getAllTargets().size());
		
		if (_showCheck->isChecked()) showTasks(_gripper->getTasks());*/
	}
	
	else if (obj == _saveGripperButton) {
		QString filename = QFileDialog::getSaveFileName(this,
			"Save file", QString::fromStdString(_wd), tr("Gripper files (*.grp.xml)"));
			
		if (filename.isEmpty()) return;
		
		QFileInfo file(filename);
		if (file.suffix().isEmpty()) filename += ".grp.xml";	
		_wd = file.path().toStdString();
			
		GripperXMLLoader::save(_gripper, filename.toStdString());
	}
	
	else if (obj == _clearButton) {
		_gripperList.clear();
		_gripperCombo->disconnect(this);
		cout << "Number of items in combo box: " << _gripperCombo->count() << endl;
		//for (int i = 0; i < _gripperCombo->count(); ++i) {
		//	_gripperCombo->removeItem(0);
		//}
		_gripperCombo->clear();
		connect(_gripperCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(guiEvent(int)));
	}
	
	else if (obj == _loadSetupButton) {
		if (!_dwc) {
			RW_WARN("You first have to open proper dynamic workcell!");
			return;
		}
		
		QString filename = QFileDialog::getOpenFileName(this,
			"Open file", QString::fromStdString(_wd), tr("Task description files (*.td.xml)"));
			
		if (filename.isEmpty()) return;
			
		_wd = QFileInfo(filename).path().toStdString();
			
		_td = TaskDescriptionLoader::load(filename.toStdString(), _dwc);
	}
	
	else if (obj == _saveSetupButton) {
		QString filename = QFileDialog::getSaveFileName(this,
			"Open file", QString::fromStdString(_wd), tr("Task description files (*.td.xml)"));
			
		if (filename.isEmpty()) return;
			
		_wd = QFileInfo(filename).path().toStdString();
			
		TaskDescriptionLoader::save(_td, filename.toStdString());
	}
	
	else if (obj == _testButton) {
		test();
	}
}



void GraspPlugin::designEvent()
{
	cout << "Design dialog opened..." << endl;
	DesignDialog* ddialog = new DesignDialog(this, _gripper, _wd);
	ddialog->exec();
	
	_wd = ddialog->getWorkingDirectory();
	
	_gripper = ddialog->getGripper();
	
	if (ddialog->isChanged()) {
		_gripperList.insert(_gripperList.begin(), _gripper);
		_gripperCombo->insertItem(-1, QString::fromStdString(_gripper->getName()));
		_gripperCombo->setCurrentIndex(0);
	}
	
	updateGripper();
	
	_progressBar->setValue(0);
	if (_tasks) {
		_progressBar->setMaximum(_tasks->getAllTargets().size());
	} else {
		_progressBar->setMaximum(0);
	}
}



void GraspPlugin::setupEvent()
{
	TaskDialog* sdialog = new TaskDialog(this, _td, _wd);
	sdialog->exec();
}



void GraspPlugin::loadGripper(const std::string& filename)
{
	cout << "Loading gripper from: " << filename << endl;
	Gripper::Ptr gripper = GripperXMLLoader::load(filename);
	_gripperList.push_back(gripper);
	_gripperCombo->addItem(QString::fromStdString(gripper->getName()));
}



void GraspPlugin::updateGripper()
{
	cout << "Updating gripper..." << endl;
	_gripper->updateGripper(_td->getWorkCell(),
		_td->getDynamicWorkCell(),
		_td->getGripperDevice(),
		_td->getGripperDynamicDevice(),
		_td->getInitState(),
		_td);
	
	cout << "Refreshing RWS..." << endl;
	getRobWorkStudio()->getWorkCellScene()->clearCache();
	getRobWorkStudio()->getWorkCellScene()->updateSceneGraph(_td->getInitState());
	getRobWorkStudio()->setWorkcell(_wc);
	getRobWorkStudio()->setState(_td->getInitState());
}



void GraspPlugin::updateSim()
{
	if (_graspSim == NULL || _wc == NULL || _dwc == NULL) return;
	
	if (!_silentMode) getRobWorkStudio()->setState(_graspSim->getSimulator()->getState());
	
	// check out the number of tasks already performed and update progress bar accordingly
	_progressBar->setValue(_graspSim->getNrTargetsDone());

	if (!_graspSim->isRunning()) {
		_timer->stop();

		//calculateQuality(_gripper->getTasks(), _generator->getSamples());
	}
	
	if (_showCheck->isChecked()) showTasks(_tasks);
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
	
	cout << "Planning tasks..." << endl;
	log().info() << "Generating " << _nOfTargetsToGen << " tasks..." << endl;
	
	if (_gripper == NULL) {
		cout << "NULL gripper" << endl;
	}
	
	try {
		_generator = new TaskGenerator(_td);
		
		_generator->generateTask(_nOfTargetsToGen, getRobWorkStudio()->getCollisionDetector(), _initState);
		//_gripper->setTasks(_generator->getTasks());
		//_gripper->setSamples(_generator->getSamples());
		_tasks = _generator->getTasks();
		_samples = _generator->getSamples();
	} catch (rw::common::Exception& e) {
		QMessageBox::critical(NULL, "RW Exception", e.what());
	}
	
	log().info() << "Done." << endl;
	
	_progressBar->setValue(0);
	_progressBar->setMaximum(_nOfTargetsToGen);

	if (_showCheck->isChecked()) showTasks(_tasks);
}



void GraspPlugin::genericEventListener(const std::string& event)
{
    if (event == "DynamicWorkCellLoaded") { 
	 
        DynamicWorkCell::Ptr dwc =
			getRobWorkStudio()->getPropertyMap().get<DynamicWorkCell::Ptr>("DynamicWorkcell", NULL);

        if (dwc == NULL) {
            log().error() << "Could not load dynamic workcell from propertymap!" << std::endl;
            return;
        }
        
        _dwc = dwc;

        _startButton->setEnabled(true);
    }
}



void GraspPlugin::showTasks(rwlibs::task::GraspTask::Ptr tasks)
{
	//return;
	vector<RenderTargets::Target> rtargets;
	Transform3D<> wTo = Kinematics::worldTframe(_td->getTargetFrame(), _wc->getDefaultState());
	
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
    
    /* setup setup group */
    _setupBox = new QGroupBox("Setup");
    QGridLayout* setupLayout = new QGridLayout(_setupBox);
    _setupBox->setLayout(setupLayout);
    
    _editSetupButton = new QPushButton("Open setup dialog");
    setupLayout->addWidget(_editSetupButton, row++, 0, 1, 2);
    connect(_editSetupButton, SIGNAL(clicked()), this, SLOT(setupEvent()));
    
    _loadSetupButton = new QPushButton("Load setup");
    setupLayout->addWidget(_loadSetupButton, row, 0);
    connect(_loadSetupButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _saveSetupButton = new QPushButton("Save setup");
    setupLayout->addWidget(_saveSetupButton, row++, 1);
    connect(_saveSetupButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _addHintButton = new QPushButton("Teach grasp");
    setupLayout->addWidget(_addHintButton, row, 0);
    connect(_addHintButton, SIGNAL(clicked()), this, SLOT(addHint()));
    
    _clearHintsButton = new QPushButton("Clear hints");
    setupLayout->addWidget(_clearHintsButton, row++, 1);
    connect(_clearHintsButton, SIGNAL(clicked()), this, SLOT(clearHints()));
    
    /* setup geometry group */
    row = 0;
    
    _geometryBox = new QGroupBox("Gripper parametrization");
    QGridLayout* geoLayout = new QGridLayout(_geometryBox);
    _geometryBox->setLayout(geoLayout);
    
    _designButton = new QPushButton("Design gripper");
    geoLayout->addWidget(_designButton, row++, 0, 1, 2);
    connect(_designButton, SIGNAL(clicked()), this, SLOT(designEvent()));
    
    _loadGripperButton = new QPushButton("Load grippers");
    geoLayout->addWidget(_loadGripperButton, row, 0);
    connect(_loadGripperButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _saveGripperButton = new QPushButton("Save gripper");
    geoLayout->addWidget(_saveGripperButton, row++, 1);
    connect(_saveGripperButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _gripperCombo = new QComboBox;
    geoLayout->addWidget(_gripperCombo, row, 0);
    connect(_gripperCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(guiEvent(int)));
     
    _clearButton = new QPushButton("Clear list");
    geoLayout->addWidget(_clearButton, row++, 1);
    connect(_clearButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
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
    
    _planButton = new QPushButton("Plan");
    simLayout->addWidget(_planButton, row, 0);
    connect(_planButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _nAutoEdit = new QLineEdit("100");
    simLayout->addWidget(_nAutoEdit, row++, 1);
    connect(_nAutoEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
    
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
    
    _testButton = new QPushButton("TEST");
    //simLayout->addWidget(_testButton, row++, 0, 1, 2);
    connect(_testButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    /* add groups to the base layout */
    layout->addWidget(_setupBox);
    layout->addWidget(_geometryBox);
    //layout->addWidget(_manualBox);
    layout->addWidget(_simBox);
    layout->addStretch(1);
}



void GraspPlugin::test()
{
}



Q_EXPORT_PLUGIN(GraspPlugin);
