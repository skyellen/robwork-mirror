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
    _graspSim = ownedPtr(new GripperTaskSimulator(_gripper, _gripper->getTasks(), _gripper->getSamples(), _td));
    
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
		
		_initState = getRobWorkStudio()->getState();
		
		_render = ownedPtr( new RenderTargets() );
		getRobWorkStudio()->getWorkCellScene()->addRender("pointRender", _render, workcell->getWorldFrame());
		
		
		
    } catch (const rw::common::Exception& e) {
		QMessageBox::critical(NULL, "RW Exception", e.what());
    }
    
    if (_dwc) {
		_dev = _wc->findDevice<TreeDevice>(_wc->getPropertyMap().get<string>("gripper"));
		_ddev = _dwc->findDevice<RigidDevice>(_wc->getPropertyMap().get<string>("gripper"));
        //_graspSim = ownedPtr(new GripperTaskSimulator(_td));
	}
}



void GraspPlugin::close()
{
    _timer->stop();
    
    if (_graspSim != NULL) {
        _graspSim->pauseSimulation();
    }
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
	
	else if (obj == _planButton) {
		planTasks();
	}
	
	else if (obj == _showCheck) {
		_showTasks = _showCheck->isChecked();
		showTasks(_gripper->getTasks());
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
	
	//State state = _initState;
	cout << "Updating gripper..." << endl;
	_gripper->updateGripper(_wc, _dwc, _dev, _ddev, _initState);
	
	cout << "Refreshing RWS..." << endl;
	State tstate = _initState;
	getRobWorkStudio()->getWorkCellScene()->clearCache();
	getRobWorkStudio()->getWorkCellScene()->updateSceneGraph(_initState);
	getRobWorkStudio()->setWorkcell(_wc);
	
	_initState = tstate;
	getRobWorkStudio()->setState(_initState);
	
	//cout << "Refreshing progress bar" << endl;
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

		calculateQuality(_gripper->getTasks(), _generator->getSamples());
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
	
	cout << "Planning tasks..." << endl;
	log().info() << "Generating " << _nOfTargetsToGen << " tasks..." << endl;
	
	if (_gripper == NULL) {
		cout << "NULL gripper" << endl;
	}
	
	try {
		_generator = new TaskGenerator(_td);
		
		_generator->generateTask(_nOfTargetsToGen, getRobWorkStudio()->getCollisionDetector(), _initState);
		_gripper->setTasks(_generator->getTasks());
		_gripper->setSamples(_generator->getSamples());
	} catch (rw::common::Exception& e) {
		QMessageBox::critical(NULL, "RW Exception", e.what());
	}
	
	log().info() << "Done." << endl;
	
	_progressBar->setValue(0);
	_progressBar->setMaximum(_nOfTargetsToGen);

	if (_showCheck->isChecked()) showTasks(_gripper->getTasks());
}



void GraspPlugin::setCurrentTask(GraspTask::Ptr task)
{
    /*try {
        _graspSim->load(task);
        //_graspSim->loadSamples(
    } catch(const Exception& e) {
        QMessageBox::information(this, "GraspPlugin", e.what());
        return;
    } catch(...) {
		cout << "Unable to load tasks!" << endl;
	}*/
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
    
    /*else if (event == "WorkcellUpdated") { // only testing
		_dev = _wc->findDevice<TreeDevice>(_wc->getPropertyMap().get<string>("gripper"));
		_ddev = _dwc->findDevice<RigidDevice>(_wc->getPropertyMap().get<string>("gripper"));
        _graspSim = ownedPtr(new GripperTaskSimulator(_td));
	}*/
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
	//_gripper->getQuality()->successQ = sim;
	//_gripper->getQuality()->coverageQ = cvg;
	
	return quality;
}



double GraspPlugin::calculateCoverage(rwlibs::task::GraspTask::Ptr tasks, rwlibs::task::GraspTask::Ptr allTasks, rw::math::Q diff)
{
	double coverage = 0.0;
	
	//int okTargets = filterAndCount(tasks, diff);
	int okTargets = TaskGenerator::countTasks(TaskGenerator::filterTasks(tasks, diff), GraspTask::Success);
	int allTargets = TaskGenerator::countTasks(TaskGenerator::filterTasks(allTasks, diff), GraspTask::Success);
	
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

void GraspPlugin::showTasks(rwlibs::task::GraspTask::Ptr tasks)
{
	//return;
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
    
    /* setup setup group */
    _setupBox = new QGroupBox("Setup");
    QGridLayout* setupLayout = new QGridLayout(_setupBox);
    _setupBox->setLayout(setupLayout);
    
    _editSetupButton = new QPushButton("Open setup dialog");
    setupLayout->addWidget(_editSetupButton, row++, 0, 1, 2);
    connect(_editSetupButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _loadSetupButton = new QPushButton("Load setup");
    setupLayout->addWidget(_loadSetupButton, row, 0);
    connect(_loadSetupButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _saveSetupButton = new QPushButton("Save setup");
    setupLayout->addWidget(_saveSetupButton, row++, 1);
    connect(_saveSetupButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    /* setup geometry group */
    row = 0;
    
    _geometryBox = new QGroupBox("Gripper parametrization");
    QGridLayout* geoLayout = new QGridLayout(_geometryBox);
    _geometryBox->setLayout(geoLayout);
    
    _designButton = new QPushButton("Design gripper");
    geoLayout->addWidget(_designButton, row++, 0, 1, 2);
    connect(_designButton, SIGNAL(clicked()), this, SLOT(designEvent()));
    
    _loadGripperButton = new QPushButton("Load gripper");
    geoLayout->addWidget(_loadGripperButton, row, 0);
    connect(_loadGripperButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
    _saveGripperButton = new QPushButton("Save gripper");
    geoLayout->addWidget(_saveGripperButton, row++, 1);
    connect(_saveGripperButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
    
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
    simLayout->addWidget(_testButton, row++, 0, 1, 2);
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
