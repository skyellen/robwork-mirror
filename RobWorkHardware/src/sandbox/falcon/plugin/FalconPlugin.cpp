#include "FalconPlugin.hpp"

#include <iostream>
#include <fstream>
#include <rw/rw.hpp>
#include <rw/common/Ptr.hpp>

using namespace std;
USE_ROBWORK_NAMESPACE
using namespace robwork;
USE_ROBWORKSIM_NAMESPACE
using namespace robworksim;
using namespace rwhw;



FalconPlugin::FalconPlugin() :
	RobWorkStudioPlugin("FalconPlugin", QIcon(":/pa_icon.png")),
	_tsim(NULL),
	_sim(NULL),
	_engine(NULL),
	_mode(WorldMode),
	_recordingEnabled(false),
	//_r(0), _p(0), _y(0),
	_grasping1(false), _grasping2(false)
{
	setupUi(this);
	
	_timer = new QTimer;
	_timer->setInterval(100);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timerEvent()));
    
    // connect stuff
    connect(_startButton, SIGNAL(clicked()), this, SLOT(startSimulation()));
    connect(_stopButton, SIGNAL(clicked()), this, SLOT(stopSimulation()));
    connect(_recordingButton, SIGNAL(clicked()), this, SLOT(startRecording()));
    
    _falcon = new FalconInterface;
    //_timer->start();
    _falcon->setCenteringMode(true, 75.0);
	_falcon->start();
}



FalconPlugin::~FalconPlugin()
{
}



void FalconPlugin::open(rw::models::WorkCell* workcell)
{
	_wc = workcell;
}



void FalconPlugin::close()
{
}



void FalconPlugin::initialize()
{
	 getRobWorkStudio()->genericEvent().add(
          boost::bind(&FalconPlugin::genericEventListener, this, _1), this);
          
     getRobWorkStudio()->keyEvent().add(
          boost::bind(&FalconPlugin::keyEventListener, this, _1, _2), this);
}



void FalconPlugin::genericEventListener(const std::string& event)
{
	if (event == "DynamicWorkCellLoaded") {
		DynamicWorkCell::Ptr dwc =
            getRobWorkStudio()->getPropertyMap().get<DynamicWorkCell::Ptr>("DynamicWorkcell", NULL);

        if (dwc == NULL) {
            cout << "Could not load dynamic workcell from propertymap!" << std::endl;
            return;
        }
        
        _dwc = dwc;
        setupGui();
	}
}



void FalconPlugin::keyEventListener(int key, Qt::KeyboardModifiers modifier){
    switch (key) {
		case 'G':
			_mode = WorldMode;
			_modeLabel->setText("GLOBAL MODE");
			break;
			
		case 'T':
			_mode = ToolMode;
			_modeLabel->setText("TOOL MODE");
			break;
			
		case 'R':
			_mode = RotationMode;
			_modeLabel->setText("ROTATION MODE");
			break;
			
		case 'V':
			_mode = RelativeMode;
			_modeLabel->setText("RELATIVE MODE");
			break;
			
		case 'W':
			_rpy(1) += angularVel; break;
			
		case 'S':
			_rpy(1) -= angularVel; break;
			
		case 'A':
			_rpy(0) += angularVel; break;
			
		case 'D':
			_rpy(0) -= angularVel; break;
			
		case 'Q':
			_rpy(2) += angularVel; break;
			
		case 'E':
			_rpy(2) -= angularVel; break;
			
		case 'O':
			_grasping2 = true; break;
			
		case 'P':
			_grasping2 = false; break;
			
		default:
			break;
	}
}



void FalconPlugin::timerEvent()
{
	if (_tsim != NULL) {
		State state = _tsim->getState();
		Transform3D<> baseTTCP = _dev->baseTframe(_tcpFrame, state);
		
		Vector3D<> pos = _falcon->getPosition();
		if (pos.norm2() < deadZoneRadius) pos = Vector3D<>::zero();
		
		if (_falcon->getButtonState() & FalconInterface::ButtonDown) {
			_grasping1 = true;
		} else {
			_grasping1 = false;
		}
		
		if (_falcon->getButtonState() & FalconInterface::ButtonUp) {
			_target = baseTTCP;
			_rpy = RPY<>(baseTTCP.R());
		}
		
		switch (_mode) {
			case WorldMode:
				pos = RPY<>(180*Deg2Rad, 0*Deg2Rad, 90*Deg2Rad).toRotation3D() * pos;
				_target.P() += pos;
				_target.R() = _rpy.toRotation3D(); //RPY<>(_r, _p, _y).toRotation3D();
				
				break;
				
			case ToolMode:
				pos = RPY<>(0*Deg2Rad, 180*Deg2Rad, 0*Deg2Rad).toRotation3D() * pos;
				_target.P() += baseTTCP.R() * pos;
				_target.R() = baseTTCP.R();
			
				break;
				
			case RelativeMode:
				break;
				
			case RotationMode:
				RPY<> rpy(_target.R());
				rpy(0) += pos(0);
				rpy(1) += pos(1);
				rpy(2) += pos(2);
				_target.R() = rpy.toRotation3D();
				
				break;
		}
		
		_pointerFrame->setTransform(_target, state);
		getRobWorkStudio()->setState(state);
	}
}



void FalconPlugin::guiEvent()
{
}



void FalconPlugin::startSimulation()
{
	RW_WARN("");
	State state = getRobWorkStudio()->getState();
	
	//TRY
	_dev = _dwc->getWorkcell()->findDevice( "UR-6-85-5-A" );
	_tcpFrame = _dwc->getWorkcell()->findFrame( "UR-6-85-5-A.TCP" );
	_robotController = _dwc->findController<SerialDeviceController>("URController");
	_gripperController = _dwc->findController<PDController>("PG70GraspController");
	_pointerFrame = _dwc->getWorkcell()->findFrame<MovableFrame>("Pointer");
	_bodies = _dwc->getBodies();
	_dev->setQ(Q(6, 0.1, -1.5, 1.5, -1.5, 0, 0), state);
	WorkCellScene::Ptr scene = getRobWorkStudio()->getWorkCellScene();
	scene->setFrameAxisVisible(true, _pointerFrame.get());
	getRobWorkStudio()->updateAndRepaint();
	
	FKTable fktable(state);
	Transform3D<> T = fktable.get(_tcpFrame);
	//_pointerFrame->setTransform(T, state);
	// /TRY
    
    RW_WARN("");
    if(_sim == NULL) {
        _engine = ownedPtr(new ODESimulator(_dwc));
        RW_WARN("");
    	//_engine->setContactLoggingEnabled( true );
        _sim = ownedPtr(new DynamicSimulator(_dwc, _engine));
        RW_WARN("");
  		
        try {
            _sim->init(state);
        } catch(...) {}
        RW_WARN("");
        
        _tsim = ownedPtr(new ThreadSimulator(_sim, state));
        ThreadSimulator::StepCallback cb(boost::bind(&FalconPlugin::step, this, _1, _2));
        _tsim->setStepCallBack(cb);
        _tsim->setRealTimeScale(1.0);
        _tsim->setTimeStep(0.01);
        RW_WARN("");
    } else {
        _tsim->reset(state);
	}

	// start the falcon device	
	//_falcon->start();

    _timer->start();

    cout << "Starting simulation..." << endl;
    _trajectory = ownedPtr(new SimulationTrajectory);
    _tsim->start();
}



void FalconPlugin::stopSimulation()
{
	_tsim->stop();
	_timer->stop();
	//_falcon->stop();
	
	if (_trajectory && _recordingEnabled && !_trajectoryFilename.empty()) {
		ofstream pathFile;
		pathFile.open(_trajectoryFilename.c_str());
		pathFile << *_trajectory;
		pathFile.close();
	}
}



void FalconPlugin::startRecording()
{
	std::string previousOpenDirectory = getRobWorkStudio()->getSettings().get<std::string>("PreviousOpenDirectory", "");
    const QString dir(previousOpenDirectory.c_str());
	
	QString filename = QFileDialog::getSaveFileName(this,
		"Save file", dir, tr("Trajectory files (*.path.txt)"));
		
	_trajectoryFilename = filename.toStdString();
	
	_recordingEnabled = true;
	_trajectory = ownedPtr(new SimulationTrajectory);
}



void FalconPlugin::step(rwsim::simulator::ThreadSimulator* sim, const rw::kinematics::State& state)
{
	/* record simulation state */
	if (_trajectory) {
		FKTable fk(state);
		vector<SimulationTrajectory::SimulationStep::ObjectPose> objPoses;
		
		BOOST_FOREACH (Body::Ptr body, _bodies) {
			objPoses.push_back(make_pair(body->getName(), fk.get(body->getBodyFrame())));
		}
		
		_trajectory->addStep(SimulationTrajectory::SimulationStep(
			sim->getTime(), _dev->getQ(state), Q(), objPoses));
	}
	
	if (sim->isInError()) {
	    std::cout << "Simulator in error... what to do!" << std::endl;
		sim->reset(state);
		return;
	}
    _robotController->stop();
	Transform3D<> baseToTCP = _dev->baseTframe(_tcpFrame, state);
    Transform3D<> target = _pointerFrame->getTransform(state);
    
    
    /* move the robot */
    // only move if we are far enough from the target pose
    Metric<Transform3D<> >::Ptr t3d_metric = MetricFactory::makeTransform3DMetric<double>(1.0, 1.0);
    if (t3d_metric->distance(baseToTCP, _target) > 0.001) {
        _robotController->movePTP_T(_target, 70, 0.1);
    }
    
    /* close the gripper */
    if (_grasping1 || _grasping2) {
		_gripperController->setTargetPos(Q(1, 0.0));
	} else {
		_gripperController->setTargetPos(Q(1, 0.035));
	}
}



void FalconPlugin::setupGui()
{
	if (!_dwc) return;
	
	// fill the robot & gripper combo boxes
	_controllers = _dwc->getControllers();
	
	_robotCombo->clear();
	_gripperCombo->clear();
	BOOST_FOREACH (SimulatedController::Ptr ctrlr, _controllers) {
		_robotCombo->addItem(QString::fromStdString(ctrlr->getControllerName()));
		_gripperCombo->addItem(QString::fromStdString(ctrlr->getControllerName()));
	}
}



Q_EXPORT_PLUGIN(FalconPlugin);
