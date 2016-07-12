#include "RWSimPlugin.hpp"

#ifdef __WIN32
#include <windows.h>
#endif

#include <sstream>

#include <boost/foreach.hpp>

#include <RobWorkStudio.hpp>

#include <rw/models/JointDevice.hpp> 

#include <rwlibs/opengl/TactileArrayRender.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwlibs/control/JointController.hpp>
#include <rwlibs/opengl/Drawable.hpp>

#include <rwsim/control/BodyController.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/sensor/TactileArraySensor.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>

#include <rwsimlibs/gui/BodyControllerWidget.hpp>
#include <rwsimlibs/gui/CreateEngineDialog.hpp>
#include <rwsimlibs/gui/GraspRestingPoseDialog.hpp>
#include <rwsimlibs/gui/GraspSelectionDialog.hpp>
#include <rwsimlibs/gui/JointControlDialog.hpp>
#include <rwsimlibs/gui/SimCfgDialog.hpp>
#include <rwsimlibs/gui/RestingPoseDialog.hpp>
#include <rwsimlibs/gui/SupportPoseAnalyserDialog.hpp>
#include <rwsimlibs/gui/TactileSensorDialog.hpp>

#include <QFileDialog>
#include <QMenuBar>
#include <QMessageBox>
#include <QTimer>

using rw::models::JointDevice;
using rw::trajectory::TimedState;
using rw::math::Math;
using rw::kinematics::State;
using rw::graphics::Render;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;
using rwlibs::control::JointController;

using namespace rwsim::dynamics;
using rwsim::loaders::DynamicWorkCellLoader;
using rwsim::sensor::TactileArraySensor;
using namespace rwsim::simulator;
using namespace rws;

RWSimPlugin::RWSimPlugin():
	RobWorkStudioPlugin("RWSimPlugin", QIcon(":/rwsimplugin/SimulationIcon.png")),
	_dwc(NULL),
	_sim(NULL),
	_startTime(0),
	_debugDrawable(NULL),
	_debugRender(NULL),
	_openCalled(false),
	_tactileSensorDialog(NULL),
	_timerShot(NULL),
	_openAction(NULL),
	_planarPoseDistAction(NULL),
    _poseDistAction(NULL),
    _graspSelectionAction(NULL),
    _graspRestPoseAction(NULL),
    _restPoseAction(NULL),
    _poseAnalyserAction(NULL)
{
    setupUi(this);

#ifdef RWSIM_HAVE_LUA
    _luastate = 0;
#endif


    connect(_openDwcBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_openLastDwcBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_editDwcBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_createSimulatorBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_destroySimulatorBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_simConfigBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_stepBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_startBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_resetBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_stopBtn     ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_saveStatePathBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_tactileSensorBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_openDeviceCtrlBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_timeStepSpin  ,SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );
    connect(_timeScaleSpin  ,SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );
    connect(_debugLevelSpin ,SIGNAL(valueChanged(int)), this, SLOT(changedEvent()) );
    connect(_updateIntervalSpin ,SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );
    connect(_debugLevelSpin , SIGNAL(valueChanged(int)), this, SLOT(changedEvent()));

    _controlGroupBox->setEnabled(false);
    _deviceGroupBox->setEnabled(false);
    _loggingGroupBox->setEnabled(false);

    // seed generat
    Math::seed( clock() );

    _timer = new QTimer( this );
    _timer->setInterval( (int)(_updateIntervalSpin->value()*1000) );
    connect( _timer, SIGNAL(timeout()), this, SLOT(changedEvent()) );
}

RWSimPlugin::~RWSimPlugin(){
	if(_timerShot!=NULL)
        _timerShot->stop();
    _timer->stop();
};

namespace {
}


void RWSimPlugin::setupMenu(QMenu* pluginmenu){

    QMenuBar *menu = getRobWorkStudio()->menuBar();

    _openAction = new QAction(QIcon(":/images/open.png"), tr("&Open DynamicWorkCell..."), this); // owned
    connect(_openAction, SIGNAL(triggered()), this, SLOT(btnPressed()));

    _planarPoseDistAction = new QAction(tr("&Planar Pose Distributions"), this); // owned
    connect(_planarPoseDistAction, SIGNAL(triggered()), this, SLOT(btnPressed()));

    _poseDistAction = new QAction(tr("&Pose Distributions"), this); // owned
    connect(_poseDistAction, SIGNAL(triggered()), this, SLOT(btnPressed()));

    _graspSelectionAction = new QAction(tr("&Grasp selection"), this); // owned
    connect(_graspSelectionAction, SIGNAL(triggered()), this, SLOT(btnPressed()));

    _graspRestPoseAction = new QAction(tr("&Grasp Simulation"), this); // owned
    connect(_graspRestPoseAction, SIGNAL(triggered()), this, SLOT(btnPressed()));

    _restPoseAction = new QAction(tr("&Object Drop Simulation"), this); // owned
    connect(_restPoseAction, SIGNAL(triggered()), this, SLOT(btnPressed()));

    _poseAnalyserAction = new QAction(tr("&Pose Analyser"), this); // owned
    connect(_poseAnalyserAction, SIGNAL(triggered()), this, SLOT(btnPressed()));

    // insert the "open dynamics" into the file menu
    boost::tuple<QMenu*, QAction*, int> action = getAction(menu, "&File", "&Close");
    if(action.get<1>()!=NULL)
        action.get<0>()->insertAction(action.get<1>(), _openAction);

    QMenu *dynMenu = new QMenu(tr("Simulation"), this);
    dynMenu->addAction(_openAction);
    dynMenu->addSeparator();
    dynMenu->addAction( _planarPoseDistAction );
    dynMenu->addAction( _poseDistAction );
    dynMenu->addAction( _graspSelectionAction );
    dynMenu->addAction( _graspRestPoseAction );
    dynMenu->addAction( _restPoseAction );
    dynMenu->addAction( _poseAnalyserAction );

    boost::tuple<QWidget*, QAction*, int> action2 = getAction(menu, "Help");
    if(action.get<1>()!=NULL)
        menu->insertMenu(action2.get<1>(), dynMenu);

    //dynMenu->addAction("Create Simulator", _createSimulatorBtn)
    //dynMenu->addAction("Settings", _createSimulatorBtn)

}

rw::common::PropertyMap& RWSimPlugin::settings(){
    return getRobWorkStudio()->getPropertyMap().get<rw::common::PropertyMap>("RobWorkStudioSettings");
}

void RWSimPlugin::btnPressed(){
    QObject *obj = sender();
    if( obj == _openDwcBtn || obj==_openAction ){
    	openDwc("");
    	if(_dwc==NULL) return;
    	_closeDwcBtn->setDisabled(false);
    	_openDwcBtn->setDisabled(true);
    	_openLastDwcBtn->setDisabled(true);
    } else if( obj == _timerShot ) {
        //std::cout << "timer shot" << std::endl;
        if(!_openCalled)
            return;
        _timerShot->stop();
        std::string str = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("DWC");

        openDwc(str);
        if( _dwc==NULL ) return;

        _closeDwcBtn->setDisabled(false);
        _openDwcBtn->setDisabled(true);
        _openLastDwcBtn->setDisabled(true);
    } else if( obj == _openLastDwcBtn ) {
        std::string dwc = settings().get<std::string>("RWSimLastOpennedDWC");
        openDwc(dwc);

        if( _dwc==NULL ) return;
    	_closeDwcBtn->setDisabled(false);
    	_openDwcBtn->setDisabled(true);
    	_openLastDwcBtn->setDisabled(true);
    } else if( obj == _closeDwcBtn ) {
    	_closeDwcBtn->setDisabled(true);
    	_openDwcBtn->setDisabled(false);
    	_openLastDwcBtn->setDisabled(false);
    	getRobWorkStudio()->setWorkcell( NULL );
    	_dwc = NULL;
    } else if( obj == _editDwcBtn ) {

    } else if( obj == _createSimulatorBtn ) {
        // Open a create simulator dialog
    	CreateEngineDialog eDialog(_dwc,this);
    	eDialog.exec();
    	DynamicSimulator::Ptr sim = eDialog.getSimulator();

    	if(sim==NULL)
    		return;

    	_createSimulatorBtn->setDisabled(true);
    	_destroySimulatorBtn->setDisabled(false);
    	_simConfigBtn->setDisabled(false);
    	State state = getRobWorkStudio()->getState();
    	sim->init(state);
    	_sim = ownedPtr(new ThreadSimulator(sim, getRobWorkStudio()->getState()));

#ifdef RWSIM_HAVE_LUA
    	rwsim::swig::addSimulatorInstance(_sim, "rwsimplugin");
#endif

    	ThreadSimulator::StepCallback cb( boost::bind(&RWSimPlugin::stepCallBack, this, _2) );

        _deviceControlBox->addItem(sim->getBodyController()->getControllerName().c_str());
    	_controlGroupBox->setEnabled(true);
        _deviceGroupBox->setEnabled(true);
        _loggingGroupBox->setEnabled(true);

        _debugLevelSpin->setValue( settings().get<int>("RWSimDebugLevelMask", 0) );
        _timeStepSpin->setValue( settings().get<double>("RWSimTimeStep", 0.01) );
        _timeScaleSpin->setValue( settings().get<double>("RWSimTimeScale", 0) );
        _updateIntervalSpin->setValue( settings().get<double>("RWSimUpdateInterval", 0.1) );



    	_sim->setStepCallBack(cb);
    	_timer->start();


    } else if( obj == _destroySimulatorBtn ) {
        _controlGroupBox->setEnabled(false);
        _deviceGroupBox->setEnabled(false);
        _loggingGroupBox->setEnabled(false);

        if(_sim==NULL){
            log().error() << "Simulator not created yet!\n" ;
            return;
        }
    	_sim->stop();
    	_sim->getSimulator()->exitPhysics();
    	_sim = NULL;
    	_destroySimulatorBtn->setDisabled(true);
    	_createSimulatorBtn->setDisabled(false);
    	_simConfigBtn->setDisabled(true);
    	_timer->stop();
    } else if( obj == _simConfigBtn ) {
    	if(_sim==NULL){
    	    log().error() << "Simulator not created yet!\n" ;
    		return;
    	}
    	SimCfgDialog eDialog(_sim->getSimulator(),this);
    	eDialog.exec();

    } else if( obj == _stepBtn ) {
        if(_sim==NULL){
            log().error() << "Simulator not created yet!\n" ;
            return;
        }

        if( _sim->isRunning() )
    		_sim->stop();

    	_sim->step();

    	getRobWorkStudio()->setState(_sim->getState());
    } else if( obj == _startBtn ) {
        if(_sim==NULL){
            log().error() << "Simulator not created yet!\n" ;
            return;
        }

    	_startBtn->setDisabled(true);
    	_stopBtn->setDisabled(false);
    	_sim->start();
    } else if( obj == _stopBtn ) {
        if(_sim==NULL){
            log().error() << "Simulator not created yet!\n" ;
            return;
        }

    	_sim->stop();
    	_startBtn->setDisabled(false);
    	_stopBtn->setDisabled(true);
    } else if( obj == _resetBtn ) {
        if(_sim==NULL){
            log().error() << "Simulator not created yet!\n" ;
            return;
        }

    	_sim->reset( getRobWorkStudio()->getState() );
    } else if( obj == _saveStatePathBtn )  {
        getRobWorkStudio()->setTimedStatePath(_path);
    } else if( obj == _openDeviceCtrlBtn ){
        if(_sim==NULL){
            log().error() << "Simulator not created yet!\n" ;
            return;
        }
        std::string ctrlname = _deviceControlBox->currentText().toStdString();
        State state = getRobWorkStudio()->getState();

        DynamicDevice *ddev = _dwc->findDevice(ctrlname).get();
        if( ddev !=NULL){
            // use a device control interface

            return;
        }

        SimulatedController::Ptr ctrl = _dwc->findController(ctrlname);
        if(!ctrl){
            if(ctrlname==_sim->getSimulator()->getBodyController()->getControllerName()){
                rwsim::control::BodyController::Ptr bcont = _sim->getSimulator()->getBodyController();
                BodyControlDialog *dialog = new BodyControlDialog(/*_dwc,*/ bcont, this);
                dialog->show();
                dialog->raise();
                dialog->activateWindow();
                return;
            }
        }
        if(ctrl){
            if( ctrl->getControllerHandle(_sim->getSimulator())!=NULL ){
                if( JointController* jctrl = dynamic_cast<JointController*>(ctrl->getControllerHandle(_sim->getSimulator()).get()) ){
                    JointControlDialog *dialog = new JointControlDialog(jctrl,this);
                    dialog->show();
                    dialog->raise();
                    dialog->activateWindow();

                } else {
                    RW_WARN("No support for this controller type!");
                }
            } else if( rwsim::control::BodyController::Ptr bcont = ctrl.cast<rwsim::control::BodyController>() ){
                BodyControlDialog *dialog = new BodyControlDialog(/*_dwc,*/ bcont, this);
                dialog->show();
                dialog->raise();
                dialog->activateWindow();
            }
            return;
        }

        RW_THROW("Device or controller by name \"" << ctrlname << "\" was not found!");

        // create a joint controller for the device and add it too the simulator

    } else if( obj==_tactileSensorBtn ){
        if( _tactileSensorDialog==NULL )
            _tactileSensorDialog = new TactileSensorDialog(_dwc.get(), this);

        //connect(this,SIGNAL(updateDialog()),_tactileSensorDialog,SLOT(updateState()) );
        connect(this,SIGNAL(updateDialog(const rw::kinematics::State&)),_tactileSensorDialog,SLOT(setState(const rw::kinematics::State&)) );

        _tactileSensorDialog->show();
        _tactileSensorDialog->raise();
        _tactileSensorDialog->activateWindow();
    } else if( obj== _planarPoseDistAction) {

    } else if( obj== _poseDistAction) {

    } else if( obj== _graspSelectionAction) {
        if(_dwc==NULL){
            QMessageBox::information( NULL,"Error","This requires an openned Dynamic WorkCell!", QMessageBox::Ok);
            return;
        }
        State state = getRobWorkStudio()->getState();
        rw::proximity::CollisionDetector::Ptr colDect = getRobWorkStudio()->getCollisionDetector();
        GraspSelectionDialog *graspSelectionDialog = new GraspSelectionDialog(state, _dwc.get(), colDect.get(),  this);
        connect(graspSelectionDialog,SIGNAL(stateChanged(const rw::kinematics::State&)),this,SLOT(setRobWorkStudioState(const rw::kinematics::State&)) );
        graspSelectionDialog->show();
        graspSelectionDialog->raise();
        graspSelectionDialog->activateWindow();

    } else if( obj== _graspRestPoseAction) {
        if(_dwc==NULL){
            QMessageBox::information( NULL,"Error","This requires an openned Dynamic WorkCell!", QMessageBox::Ok);
            return;
        }
        State state = getRobWorkStudio()->getState();
        rw::proximity::CollisionDetector::Ptr colDect = getRobWorkStudio()->getCollisionDetector();
        GraspRestingPoseDialog *graspRestPoseDialog = new GraspRestingPoseDialog(state, _dwc.get(), colDect.get(),  this);

        connect(graspRestPoseDialog,SIGNAL(stateChanged(const rw::kinematics::State&)),this,SLOT(setRobWorkStudioState(const rw::kinematics::State&)) );
        //connect(graspRestPoseDialog,SIGNAL(restingPoseEvent(const RestingConfig&)), this,SLOT(restConfigEvent(const RestingConfig&)) );
        // TODO: this should use the genericAnyEvent
        graspRestPoseDialog->show();
        graspRestPoseDialog->raise();
        graspRestPoseDialog->activateWindow();

    } else if( obj== _restPoseAction) {
        if(_dwc==NULL){
            QMessageBox::information( NULL,"Error","This requires an openned Dynamic WorkCell!", QMessageBox::Ok);
            return;
        }

        State state = getRobWorkStudio()->getState();
        rw::proximity::CollisionDetector::Ptr colDect = getRobWorkStudio()->getCollisionDetector();
        RestingPoseDialog *restPoseDialog = new RestingPoseDialog(state, _dwc.get(), colDect.get(),  this);
        connect(restPoseDialog,SIGNAL(stateChanged(const rw::kinematics::State&)),this,SLOT(setRobWorkStudioState(const rw::kinematics::State&)) );
        connect(restPoseDialog,SIGNAL(restingPoseEvent(const rw::kinematics::State&)),this,SLOT(setRobWorkStudioState(const rw::kinematics::State&)) );
        restPoseDialog->show();
        restPoseDialog->raise();
        restPoseDialog->activateWindow();
    } else if( obj== _poseAnalyserAction) {
        if(_dwc==NULL){
            QMessageBox::information( NULL,"Error","This requires an openned Dynamic WorkCell!", QMessageBox::Ok);
            return;
        }

        State state = getRobWorkStudio()->getState();
        rw::proximity::CollisionDetector::Ptr colDect = getRobWorkStudio()->getCollisionDetector();
        SupportPoseAnalyserDialog *poseAnalyserDialog = new SupportPoseAnalyserDialog(state, _dwc.get(), colDect.get(),  getRobWorkStudio(), this);
        connect(poseAnalyserDialog,SIGNAL(stateChanged(const rw::kinematics::State&)),this,SLOT(setRobWorkStudioState(const rw::kinematics::State&)) );
        poseAnalyserDialog->show();
        poseAnalyserDialog->raise();
        poseAnalyserDialog->activateWindow();
    }

}

void RWSimPlugin::setRobWorkStudioState(const rw::kinematics::State& state){
    getRobWorkStudio()->setState(state);
}

namespace {

	std::vector<Eigen::MatrixXf> getTactileData(const std::vector<SimulatedSensor::Ptr>& sensors, const State& state){
		std::vector<Eigen::MatrixXf> datas;
		BOOST_FOREACH(const SimulatedSensor::Ptr& sensor, sensors){
        	if( TactileArraySensor *tsensor = dynamic_cast<TactileArraySensor*>( sensor.get() ) ) {
                datas.push_back( tsensor->getTexelData(state) );
            }
        }
		return datas;
	}
}

void RWSimPlugin::stepCallBack(const rw::kinematics::State& state){
    // here we log stuff if enabled
    _state = state;
	if( _recordStatePathBox->isChecked()){
	    _path.push_back( TimedState( _sim->getTime() , state) );
	}
}


void RWSimPlugin::changedEvent(){
    QObject *obj = sender();
    if( obj == _timer ){
        // update stuff
        updateStatus();
    } else if( obj == _timeStepSpin ){
        settings().set<double>("RWSimTimeStep",_timeStepSpin->value());
    	_sim->setTimeStep( _timeStepSpin->value() );
    } else if( obj == _updateIntervalSpin ) {
        settings().set<double>("RWSimUpdateInterval",_updateIntervalSpin->value());
    	_timer->setInterval( (int)(_updateIntervalSpin->value()*1000) );
    	//_timer->setInterval( settings().get<double>("RWSimUpdateInterval",0.01)*1000 );
    } else if( obj == _timeScaleSpin ){
        settings().set<double>("RWSimTimeScale",_timeScaleSpin->value());
        std::cout << "Setting time scale: " << _timeScaleSpin->value() << std::endl;
        _sim->setRealTimeScale( _timeScaleSpin->value() );
    } else if( obj == _debugLevelSpin ){
        settings().set<int>("RWSimDebugLevelMask",_debugLevelSpin->value());

    	//std::cout << "Debug level spin!!" << std::endl;
    	if(_debugRender==NULL){
    		if( _debugLevelSpin->value()==0 )
    			return;

    		_debugRender = _sim->getSimulator()->createDebugRender();
    		if( _debugRender == NULL ){
    			Log::errorLog() << "The current simulator does not support debug rendering!" << std::endl;
    			return;
    		}
    		_debugRender->setDrawMask( _debugLevelSpin->value() );
    		_debugDrawable = new Drawable( _debugRender, "DebugRender" );
            getRobWorkStudio()->getWorkCellScene()->addDrawable(_debugDrawable, _dwc->getWorkcell()->getWorldFrame());
    	} else {
    	    _debugRender->setDrawMask( _debugLevelSpin->value() );
    	    if(_debugLevelSpin->value()==0){
    	        _debugDrawable->setVisible(false);
    	    } else {
    	        _debugDrawable->setVisible(true);
    	    }
    	}

	}
}

void RWSimPlugin::updateStatus(){
	if(_sim==NULL)
		return;

	if( _forceSceneUpdate->isChecked() && _sim->isRunning() ){
		getRobWorkStudio()->setState( _state );
	}

	double time = _sim->getTime();
	std::stringstream str;
	str << time << " s";
	_timeLbl->setText(str.str().c_str());
}

void RWSimPlugin::open(rw::models::WorkCell* workcell){

#ifdef RWSIM_HAVE_LUA
	struct RWSimLuaLibrary: public rwlibs::swig::LuaState::LuaLibrary {
		const std::string getId(){ return "RWSimLuaLibrary"; }
		bool initLibrary(rwlibs::swig::LuaState& lstate){
			//std::cout << "CALLING INIT" << std::endl;
			return rwsim::swig::openLuaLibRWSim(lstate.get())==0;}
	};
    // check if the lua state is there and if RobWorkSim libraries has been added
	rwlibs::swig::LuaState::Ptr lstate = getRobWorkStudio()->getPropertyMap().get<rwlibs::swig::LuaState::Ptr>("LuaState", NULL);
    if(lstate!=NULL && lstate!=_luastate){
        _luastate = lstate;
        rwsim::swig::openLuaLibRWSim(_luastate->get() );
        _luastate->addLibrary( rw::common::Ptr<rwlibs::swig::LuaState::LuaLibrary>(new RWSimLuaLibrary()) );

        //_luastate->removeLibrary( "LuaLibRWSim" );
    }
    if(_luastate!=NULL && _dwc!=NULL){
        rwsim::swig::setDynamicWorkCell( _dwc.get() );
    }
#endif

    if( workcell==NULL || _dwc==NULL ){
	    _openCalled = true;
		return;
	}

	_state = getRobWorkStudio()->getState();

	// add sensor drawables to the workcell drawer
    BOOST_FOREACH(SimulatedSensor::Ptr sensor,  _dwc->getSensors()){
        if( TactileArraySensor::Ptr tsensor = sensor.cast<TactileArraySensor>() ){
            //std::cout << "ADDING TACTILE SENSOR DRAWER..." << std::endl;
        	log().debug() << "Adding tactile sensor render to \"" << sensor->getSensorModel()->getName() << "\"!" << std::endl;
            TactileArrayRender *render = new TactileArrayRender( tsensor->getTactileArrayModel() );
            Drawable *drawable = new Drawable(ownedPtr<Render>(render), sensor->getSensorModel()->getName() );
            getRobWorkStudio()->getWorkCellScene()->addDrawable(drawable, tsensor->getFrame() );
        }
    }

    _deviceControlBox->clear();
    BOOST_FOREACH(DynamicDevice::Ptr device, _dwc->getDynamicDevices()){
        rw::models::Device *dev = &device->getModel();
        if( dynamic_cast<JointDevice*>(dev) == NULL )
            continue;
        RW_ASSERT(dev);
        //std::cout << "Dev name: " << std::endl;
        //std::cout << dev->getName() << std::endl;
        _deviceControlBox->addItem(dev->getName().c_str());
    }

    BOOST_FOREACH(SimulatedController::Ptr ctrl, _dwc->getControllers()){
        RW_ASSERT(ctrl!=NULL);
        _deviceControlBox->addItem(ctrl->getControllerName().c_str());
    }

}

void RWSimPlugin::openDwc(const std::string& file){
	std::string dwcFile;

	_context._previousOpenDirectory =  settings().get<std::string>("RWSimLastOpennedDIR",_context._previousOpenDirectory);
	if( file.empty() ){
	    QString selectedFilter;
	    const QString dir(_context._previousOpenDirectory.c_str());

	    QString filename = QFileDialog::getOpenFileName(
	        this,
	        "Open Drawable", // Title
	        dir, // Directory
	        "DWC XML files ( *.dwc.xml )"
	        "\nAll supported ( *.xml )"
	        "\n All ( *.* )",
	        &selectedFilter);

	    dwcFile = filename.toStdString();
	} else {
		dwcFile = file;
	}
	if (dwcFile.empty())
		return;

    _context._previousOpenDirectory =
    	rw::common::StringUtil::getDirectoryName(dwcFile);

    settings().set<std::string>("RWSimLastOpennedDIR",_context._previousOpenDirectory);
    settings().set<std::string>("RWSimLastOpennedDWC",dwcFile);

    Ptr<DynamicWorkCell> dwc(NULL);
    try {
        log().info() << "Openning Dynamic WorkCell: " << dwcFile << std::endl;
        dwc = DynamicWorkCellLoader::load(dwcFile);
    } catch (const Exception& exp) {
        QMessageBox::information(
            NULL,
            "Exception",
            exp.getMessage().getText().c_str(),
            QMessageBox::Ok);
        return;
    }
    if( dwc==NULL )
    	RW_THROW("Dynamic workcell is null");

    _dwc = dwc;
    // adding the DynamicWorkcell to the propertymap such that others can use it

    getRobWorkStudio()->getPropertyMap().add<DynamicWorkCell::Ptr>(
            "DynamicWorkcell",
            "A workcell with dynamic description",
            _dwc );

    // signal to other plugins that a DynamicWorkCell has been loadet
    getRobWorkStudio()->genericEvent().fire("DynamicWorkCellLoaded");

    // if we add to propertymap before openning workcell then other plugins can test for it
    getRobWorkStudio()->setWorkcell( dwc->getWorkcell() );
}

void RWSimPlugin::close(){

}

void RWSimPlugin::initialize(){
    getRobWorkStudio()->stateChangedEvent().add(
    		boost::bind(&RWSimPlugin::stateChangedListener, this, _1), this);
    Log::setLog( _log );
    _timerShot = NULL;

    if( getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").has("DWC") ){
        _timerShot = new QTimer( this );
        _timerShot->setSingleShot(false);
        _timerShot->setInterval( 500 );
        _timerShot->start();
        connect( _timerShot, SIGNAL(timeout()), this, SLOT(btnPressed()) );

        //getRobWorkStudio()->getView()->setZoomScale(5.0);
        //getRobWorkStudio()->getView()->setCheckForCollision(false);
    } else {
        //std::cout << "NOOOOO ARG" << std::endl;
    }
}

void RWSimPlugin::stateChangedListener(const State& state){
    updateDialog(state);
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(RWSimPlugin);
#endif
