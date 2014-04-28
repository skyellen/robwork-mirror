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
	_wsad(RotationMode),
	_stickyMode(false),
	_moveJoints(false),
	_recordingEnabled(false),
	//_r(0), _p(0), _y(0),
	_grasping(false)
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
    _falcon->setCenteringMode(true, 60.0);
    //_falcon->setStickyMode(true);
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



void FalconPlugin::keyEventListener(int key, Qt::KeyboardModifiers modifier)
{
	State state = _tsim->getState();
	Transform3D<> baseTTCP = _dev->baseTframe(_tcpFrame, state);
	Transform3D<> worldTTCP = Kinematics::worldTframe(_tcpFrame, state);
	
	/* let's get the view */
	Transform3D<> viewT = getRobWorkStudio()->getViewTransform();
	Rotation3D<> rot;
	Transform3D<> nviewT;
	
    switch (key) {
		/* choose modes section */
		case 'G':
			_mode = WorldMode;
			_modeLabel->setText("GLOBAL");
			
			//_target = baseTTCP;
			//_rpy = RPY<>(baseTTCP.R());
	
			break;
			
		
		case 'T':
			_mode = ToolMode;
			_modeLabel->setText("TOOL");
			break;
			
		/*case 'R':
			_mode = RotationMode;
			_modeLabel->setText("ROTATION MODE");
			break;*/
			
		case 'F':
			_wsad = ViewMode;
			_wsadLabel->setText("VIEW");
			break;
			
		case 'R':
			_wsad = RotationMode;
			_wsadLabel->setText("ROTATION");
			break;
			
		/* rotate view section */
		case 'A':
			if (_wsad == ViewMode) {
				rot = EAA<>(Vector3D<>::z(), -5.0*Deg2Rad).toRotation3D();
				nviewT = Transform3D<>(rot*(viewT.P()-worldTTCP.P())+worldTTCP.P(), rot * viewT.R());
				getRobWorkStudio()->setViewTransform(nviewT);
				getRobWorkStudio()->updateAndRepaint();
			} else if (_wsad == RotationMode) {
				_p = -0.05;
				_moveJoints = true;
			}
			break;
			
		case 'D':
			if (_wsad == ViewMode) {
				rot = EAA<>(Vector3D<>::z(), 5.0*Deg2Rad).toRotation3D();
				nviewT = Transform3D<>(rot*(viewT.P()-worldTTCP.P())+worldTTCP.P(), rot * viewT.R());
				getRobWorkStudio()->setViewTransform(nviewT);
				getRobWorkStudio()->updateAndRepaint();
			} else if (_wsad == RotationMode) {
				_p = 0.05;
				_moveJoints = true;
			}
			break;
			
		case 'W':
			if (_wsad == ViewMode) {
				rot = EAA<>(viewT.R() * Vector3D<>::x(), -5.0*Deg2Rad).toRotation3D();
				nviewT = Transform3D<>(rot*(viewT.P()-worldTTCP.P())+worldTTCP.P(), rot * viewT.R());
				getRobWorkStudio()->setViewTransform(nviewT);
				getRobWorkStudio()->updateAndRepaint();
			} else if (_wsad == RotationMode) {
				_r = 0.05;
				_moveJoints = true;
			}
			break;
			
		case 'S':
			if (_wsad == ViewMode) {
				rot = EAA<>(viewT.R() * Vector3D<>::x(), 5.0*Deg2Rad).toRotation3D();
				nviewT = Transform3D<>(rot*(viewT.P()-worldTTCP.P())+worldTTCP.P(), rot * viewT.R());
				getRobWorkStudio()->setViewTransform(nviewT);
				getRobWorkStudio()->updateAndRepaint();
			} else if (_wsad == RotationMode) {
				_r = -0.05;
				_moveJoints = true;
			}
			break;
		
		/* zoom view */	
		case 'Q':
			if (_wsad == ViewMode) {
				nviewT = Transform3D<>(1.1*viewT.P(), viewT.R());
				getRobWorkStudio()->setViewTransform(nviewT);
				getRobWorkStudio()->updateAndRepaint();
			} else if (_wsad == RotationMode) {
				_y = -0.05;
				_moveJoints = true;
			}
			break;
			
		case 'E':
			if (_wsad == ViewMode) {
				nviewT = Transform3D<>(1.0/1.1*viewT.P(), viewT.R());
				getRobWorkStudio()->setViewTransform(nviewT);
				getRobWorkStudio()->updateAndRepaint();
			} else if (_wsad == RotationMode) {
				_y = 0.05;
				_moveJoints = true;
			}
			break;
			
		/*case 'O':
			_grasping = true; break;
			
		case 'P':
			_grasping = false; break;*/
			
		default:
			break;
	}
}



void FalconPlugin::timerEvent()
{
	if (_tsim != NULL) {
		State state = _tsim->getState();
		Transform3D<> baseTTCP = _dev->baseTframe(_tcpFrame, state);
		Transform3D<> worldTTCP = Kinematics::worldTframe(_tcpFrame, state);
		
		/* get the view transform from RWS */
		Transform3D<> viewT = getRobWorkStudio()->getView()->getSceneViewer()->getViewCamera()->getTransform();
		RPY<> viewRPY = RPY<>(viewT.R());
		Rotation3D<> rot;
		Transform3D<> nviewT;
		
		Vector3D<> pos = _falcon->getPosition();
		if (pos.norm2() < deadZoneRadius) pos = Vector3D<>::zero();
		
		/* check if we should enable grasping */		
		if (_falcon->getButtonState() & FalconInterface::ButtonLeft) {
			_grasping = true;
		} else if (_falcon->getButtonState() & FalconInterface::ButtonRight) {
			_grasping = false;
		}
		
		/* this is for moving pointer to robot TCP */
		if (_falcon->getButtonState() & FalconInterface::ButtonDown) {
			_target = worldTTCP;
			_rpy = RPY<>(worldTTCP.R());
		}
		
		/* check if to enable the rotation mode from the grip */
		//static bool alreadyPressed = false;
		/*if (_falcon->getButtonState() & FalconInterface::ButtonDown) {
			_mode = ViewMode;
			
			/*if (!alreadyPressed) {
				alreadyPressed = true;
				Q robotQ = _dev->getQ(state);
				_r = robotQ(3); _p = robotQ(4); _y = robotQ(5);
			}*/
		/*} else {
			_mode = WorldMode;
			//alreadyPressed = false;
		}*/
		
		/* check the mode and proceed accordingly */
		switch (_mode) {
			case WorldMode:				
				/* translate falcon position to world coordinates */
				//pos = RPY<>(180*Deg2Rad, 0*Deg2Rad, 90*Deg2Rad).toRotation3D() * pos;
				_target.P() += viewRPY.toRotation3D() * pos;
				_target.R() = _rpy.toRotation3D();
				
				break;
				
			case ToolMode:
				if (fabs(pos[0]) > fabs(pos[1]) && fabs(pos[0]) > fabs(pos[2])) pos[1] = pos[2] = 0.0;
				if (fabs(pos[1]) > fabs(pos[0]) && fabs(pos[1]) > fabs(pos[2])) pos[0] = pos[2] = 0.0;
				if (fabs(pos[2]) > fabs(pos[0]) && fabs(pos[2]) > fabs(pos[1])) pos[0] = pos[1] = 0.0;
			
				//pos = RPY<>(0*Deg2Rad, 180*Deg2Rad, 0*Deg2Rad).toRotation3D() * pos;
				
				pos(2) *= -1;
				
				_target.P() += worldTTCP.R() * pos;
				_target.R() = worldTTCP.R();
			
				break;
				
			/*case ViewMode:
				rot = EAA<>(Vector3D<>::z(), pos[0]).toRotation3D();
				nviewT = Transform3D<>(rot*(viewT.P()-worldTTCP.P())+worldTTCP.P(), rot * viewT.R());
				rot = EAA<>(Vector3D<>::x(), pos[1]).toRotation3D();
				nviewT = Transform3D<>(rot*(nviewT.P()-worldTTCP.P())+worldTTCP.P(), rot * nviewT.R());
				getRobWorkStudio()->setViewTransform(nviewT);
				getRobWorkStudio()->updateAndRepaint();
				break;
				
			case RotationMode:
				//RPY<> rpy(_target.R());
				
				/*_rpy(0) += pos(1);
				_rpy(1) += pos(0);
				_rpy(2) += pos(2);
				//_r += pos(1); _p += pos(0); _y += 
				//_target.R() = _rpy.toRotation3D();
				
				break;*/
		}
		
		/* this is for manually rotating final joints */
		if (_moveJoints) {
			Q robotQ = _dev->getQ(state);
			
			robotQ(3) += _r; _r = 0.0;
			robotQ(4) += _p; _p = 0.0;
			robotQ(5) += _y; _y = 0.0;
			
			_dev->setQ(robotQ, state);
			_target = Kinematics::worldTframe(_tcpFrame, state);
			_rpy = RPY<>(_target.R());
			
			_moveJoints = false;
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
	_gripperController = _dwc->findController<PDController>("GraspController");
	_pointerFrame = _dwc->getWorkcell()->findFrame<MovableFrame>("Pointer");
	_bodies = _dwc->getBodies();
	_dev->setQ(Q(6, 0.1, -1.5, -1.5, -1.5, 1.5, 0), state);
	WorkCellScene::Ptr scene = getRobWorkStudio()->getWorkCellScene();
	scene->setFrameAxisVisible(true, _pointerFrame.get());
	getRobWorkStudio()->updateAndRepaint();
	
	FKTable fktable(state);
	Transform3D<> T = fktable.get(_tcpFrame);
	Transform3D<> worldTTCP = Kinematics::worldTframe(_tcpFrame, state);
	_target = worldTTCP;
	_rpy = RPY<>(worldTTCP.R());
	// /TRY
    
    RW_WARN("");
    if(_sim == NULL) {
        _engine = ownedPtr(new ODESimulator(_dwc));
        RW_WARN("");
    	_engine->setContactLoggingEnabled(true);
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
    _modeLabel->setText("GLOBAL MODE");
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
	/* print out contact points */
	typedef boost::unordered_map<std::pair<std::string,std::string>, bool> CMap;
	CMap cbodies = _engine->getContactingBodies();
	std::vector<SimulationTrajectory::SimulationStep::ObjectPair> cnames;
	
	if (cbodies.size() > 0) {
		cout << "Contact points: ";
		BOOST_FOREACH (CMap::value_type& c, cbodies) {
			cnames.push_back(std::make_pair(c.first.first, c.first.second));
			cout << c.first.first << "-" << c.first.second << " ";
		}
		cout << endl;
	}

	/*typedef std::vector<SimulationTrajectory::SimulationStep::ObjectPair>::const_iterator J;
	for (J it = cnames.begin(); it != cnames.end(); ++it) {
		cout  << ":" << it->first << "-" << it->second << "$$";
	}*/
	
	/* record simulation state */
	FKTable fk(state);
	if (_trajectory) {
		//FKTable fk(state);
		vector<SimulationTrajectory::SimulationStep::ObjectPose> objPoses;
		
		BOOST_FOREACH (Body::Ptr body, _bodies) {
			objPoses.push_back(make_pair(body->getName(), fk.get(body->getBodyFrame())));
		}
		
		SimulationTrajectory::SimulationStep sstep(sim->getTime(), _dev->getQ(state), Q(), objPoses, cnames);
		cout << sstep<< " ";
		_trajectory->addStep(sstep);
	}
	
	/* error handling */
	if (sim->isInError()) {
	    std::cout << "Simulator in error... what to do!" << std::endl;
		sim->reset(state);
		return;
	}
    _robotController->stop();
	Transform3D<> baseToTCP = _dev->baseTframe(_tcpFrame, state);
	Transform3D<> worldTTCP = Kinematics::worldTframe(_tcpFrame, state);
    Transform3D<> target = inverse(_dev->worldTbase(state)) * _target;
    
    /* move the robot */
    // only move if we are far enough from the target pose
    Metric<Transform3D<> >::Ptr t3d_metric = MetricFactory::makeTransform3DMetric<double>(1.0, 1.0);
    if (t3d_metric->distance(baseToTCP, target) > 0.001) {
        _robotController->movePTP_T(target, 70, 0.1);
    }
    
    /*if (_mode = RotationMode) {
		Q targetQ = _dev->getQ(state);
		targetQ(5) = _y;
		_robotController->movePTP(targetQ, 70);
		
		target = worldTTCP;
		//_pointerFrame->setTransform(worldTTCP, state);
	}*/
    
    /* close the gripper */
    if (_grasping) {
		//_gripperController->setTargetPos(Q(1, 0.0));
		_gripperController->setTargetPos( Q(10, 66,55,66,0, 66,55,66, 66,55,66)*Deg2Rad );
	} else {
		//_gripperController->setTargetPos(Q(1, 0.035));
		_gripperController->setTargetPos( Q(10,0,0,0,0,0,0,0,0,0,0)*Deg2Rad );
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
