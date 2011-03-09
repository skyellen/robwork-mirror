#include "SimTaskPlugin.hpp"

#include <QPushButton>

#include <RobWorkStudio.hpp>

#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwsim/drawable/SimulatorDebugRender.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rw/graspplanning/Grasp3D.hpp>
#include <fstream>
#include <iostream>

#include <sandbox/WrenchMeasure3D.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

USE_ROBWORKSIM_NAMESPACE
using namespace robworksim;

using namespace rws;

using namespace rwlibs::simulation;

SimTaskPlugin::SimTaskPlugin():
    RobWorkStudioPlugin("SimTaskPluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // now connect stuff from the ui component
    connect(_loadTaskBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_saveResultBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_loadConfigBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_saveConfigBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_startBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_stopBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_updateConfigBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_showTaskSpinBox    ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

    _timer = new QTimer( NULL );
    _timer->setInterval( 200 );
    connect( _timer, SIGNAL(timeout()), this, SLOT(btnPressed()) );

    _propertyView = new PropertyViewEditor(this);
    _propertyView->setPropertyMap(&_config);

    QVBoxLayout *vbox = new QVBoxLayout;
    vbox->addWidget(_propertyView);
    _configGroupBox->setLayout(vbox);

    _loadConfigBtn->setEnabled(false);
    _saveConfigBtn->setEnabled(false);
    _startBtn->setEnabled(false);
    _stopBtn->setEnabled(false);
    _loadTaskBtn->setEnabled(false);
    _saveResultBtn->setEnabled(false);


}

SimTaskPlugin::~SimTaskPlugin()
{
}

void SimTaskPlugin::initialize() {
    std::cout << "SimTaskPlugin::initialize" << std::endl;
    getRobWorkStudio()->stateChangedEvent().add(
            boost::bind(&SimTaskPlugin::stateChangedListener, this, _1), this);

    getRobWorkStudio()->genericEvent().add(
          boost::bind(&SimTaskPlugin::genericEventListener, this, _1), this);

    Log::setLog( _log );
}

void SimTaskPlugin::startSimulation() {

    if(_tasks==NULL){
        QMessageBox::information(this, "SimTaskPlugin", "Cannot start simulation, no tasks have been loaded!");
        return;
    }

    // create simulator and stuff here
    makeSimulator();

    if(_hand==NULL){
        log().error() << "Device is invalid! ";
        return;
    }

    if(_dhand==NULL){
        log().error() << "Device is invalid! No such DynamicDevice.";
        return;
    }

    if(_mbase==NULL){
        log().error() << "Movable base frame does not exist.";
        return;
    }

    if(_object==NULL){
        log().error() << "Object does not exist.";
        return;
    }

    if(_tcp==NULL){
        log().error() << "End frame  does not exist.";
        return;
    }

    log().info() << "Starting simulation!";
    _nextTaskIndex = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get("StartIdx", 0);
    _lastSaveTaskIndex = _nextTaskIndex;
    _stopped = false;
    SimulatedController* scontroller = _dwc->findController("GraspController").get();
    if(scontroller==NULL)
        RW_THROW("No controller!");
    _controller = dynamic_cast<rwlibs::control::JointController*>( scontroller->getController() );
    _progressBar->setRange( 0 , _targets->size());
    _progressBar->setValue( 0 );
    _tsim->setPeriodMs(-1);
    _currentState = NEW_GRASP;
    _timer->start();
    _tsim->start();

    log().info() << "Simulation Started\n";
}

void SimTaskPlugin::open(WorkCell* workcell)
{
    if(workcell==NULL || _dwc==NULL)
        return;
    _wc = workcell;

    _loadConfigBtn->setEnabled(true);
    _saveConfigBtn->setEnabled(true);

    loadConfig(true);

    _propertyView->update();

    if(getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").has("Auto")){
        if(!_configured){
            log().error() << "Plugin not configured, please load a valid config file!\n";
            return;
        }
        // load tasks from file
        loadTasks(true);

        //start simulation
        startSimulation();
    }
}

void SimTaskPlugin::close() {

}

void SimTaskPlugin::btnPressed() {
    QObject *obj = sender();
    if(obj==_loadTaskBtn){
        // load tasks from a filename
        loadTasks(false);
    } else if(obj==_saveResultBtn){
        saveTasks(false);
    } else if(obj==_saveConfigBtn){
        saveConfig();
    } else if(obj==_loadConfigBtn){
        if(_dwc==NULL)
            return;
        loadConfig(false);
    } else if(obj==_startBtn){
        startSimulation();
    }else if(obj==_updateConfigBtn){
        _propertyView->update();
        updateConfig();
    } else if(obj==_stopBtn){
        _tsim->stop();
        _timer->stop();
    } else if(obj==_showTaskSpinBox){

        if(_targets==NULL){
            log().error() << "No targets\n";
            return;
        }
        if(!_configured){
            log().error() << "Not configured\n";
            return;
        }
        State state = getRobWorkStudio()->getState();
        int idxTmp = _showTaskSpinBox->value();

        if(idxTmp<0 || idxTmp >= _targets->size()){
            log().error() << "Choosen task is out of bounds\n";
            return;
        }

        Transform3D<> wTp = Kinematics::worldTframe(_mbase->getParent(state), state);
        Transform3D<> start = inverse(wTp) * _wTe_n * (*_targets)[idxTmp]->get() * inverse(_bTe);
        _mbase->setTransform(start, state);
        // and calculate the home lifting position
        _home = _wTe_home * (*_targets)[idxTmp]->get() * inverse(_bTe);
        _hand->setQ(_openQ, state);
        _object->getMovableFrame()->setTransform(_objHome, state);

        std::cout << "eTb: " << inverse(_bTe) << std::endl;
        std::cout << "TARGET: " << (*_targets)[idxTmp]->get() << std::endl;
        std::cout << "HOME: " << _home << std::endl;

        getRobWorkStudio()->setState(state);
    } else if(obj==_timer){
        // update the RobWorkStudio state
        State state = _tsim->getState();
        getRobWorkStudio()->setState(state);

        if(_stopped){
            _tsim->stop();
            _timer->stop();
            if(getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").has("Auto")){
                saveTasks(true);
            }
        }

        // update progress
        _progressBar->setValue( _nextTaskIndex );

    }
}


void SimTaskPlugin::loadConfig(bool automatic){
    std::string prevDir = settings().get<std::string>("RWSimLastOpennedDIR","");

    std::string filename = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("SimTaskConfig", "");
    std::string simTaskConfigFile = filename;
    if(!automatic){

        QString selectedFilter;
        if(filename=="")
            filename = prevDir;
        const QString dir(filename.c_str());

        QString filename = QFileDialog::getOpenFileName(
            this,
            "Open Property file", // Title
            dir, // Directory
            "All supported ( *.xml )"
            " \nRW TASK files ( *.prop.xml )"
            " \n All ( *.* )",
            &selectedFilter);

        simTaskConfigFile = filename.toStdString();
    }

    if(simTaskConfigFile!=""){
        std::cout << "Loading tasks: ";
        std::cout << "\t-Filename: " << simTaskConfigFile;
        try {
            std::cout << "3" << std::endl;
            _config = XMLPropertyLoader::load( simTaskConfigFile );
        } catch(...) {
            QMessageBox::information(this, "SimTaskPlugin", "SimTaskConfig could not be loadet!");
        }
    }
    updateConfig();

    // for some reason this crashes sometimes...
    //_propertyView->setPropertyMap( &_config );
}

void SimTaskPlugin::updateConfig(){
    // START
    State state = getRobWorkStudio()->getState();

    std::string devName;
    if( !_config.has("DeviceName") || _config.get<std::string>("DeviceName")==""){
        if(_dwc->getDynamicDevices().size()>0){
            devName = _dwc->getDynamicDevices()[0]->getModel().getName();
        }
        _config.add<std::string>("DeviceName","Name of the hand used for grasping!", devName);
    }
    devName = _config.get<std::string>("DeviceName");
    _hand =_wc->findDevice<JointDevice>(devName).get();
    _dhand = _dwc->findDevice(devName);

    std::string baseName;
    if( !_config.has("MovableBase") || _config.get<std::string>("MovableBase")==""){
        // check all frames from device base to world
        if(_hand != NULL ){
            std::vector<Frame*> frames = Kinematics::childToParentChain(_hand->getBase(), _wc->getWorldFrame(), state);
            BOOST_FOREACH(Frame *tmpKinFrame, frames){
                if( KinematicBody *kbody = _dwc->findBody<KinematicBody>(tmpKinFrame->getName()) ){
                    baseName = kbody->getName();
                }
            }
        }
        _config.add<std::string>("MovableBase","Name of the body that the hand is attached to",baseName);
    }
    baseName = _config.get<std::string>("MovableBase");
    _mbase = _wc->findFrame<MovableFrame>(baseName);

    if( !_config.has("TCP") || _config.get<std::string>("TCP")=="" ){
        _config.add<std::string>("TCP", "Name of the Tool Center Point of the hand", baseName);
    }
    std::string tcpName = _config.get<std::string>("TCP");
    _tcp = _wc->findFrame(tcpName);

    std::string objName;
    if( !_config.has("ObjectName") || _config.get<std::string>("ObjectName")=="" ){
        // find the first rigid body
        std::vector<RigidBody*> rbodies = _dwc->findBodies<RigidBody>();
        if(rbodies.size()>0)
            objName = rbodies[0]->getName();
        _config.add<std::string>("ObjectName","Name of the object that is to be grasped", objName);
    }
    objName = _config.get<std::string>("ObjectName");
    _object = _dwc->findBody<RigidBody>(objName);


    if( !_config.has("CalculateWrenchQuality") ){
        _config.add<bool>("CalculateWrenchQuality","Set true if the quality of the grasp should be calculated", true);
    }
    _calcWrenchQuality = _config.get<bool>("CalculateWrenchQuality");

    if( !_config.has("MaxObjectGripperDistance") ){
        _config.add<double>("MaxObjectGripperDistance","The maximum allowed distance between gripper and object", 50.0);
    }
    _maxObjectGripperDistance = _config.get<double>("MaxObjectGripperDistance");


    if(_hand!=NULL ){
        _openQ = _config.get<Q>("DefOpenQ", _hand->getQ(state));
        _closeQ = _config.get<Q>("DefCloseQ", _hand->getQ(state));
    }



    if(_mbase!=NULL && _tcp!=NULL)
        _bTe = Kinematics::frameTframe(_mbase, _tcp, state);

    if(_object!=NULL)
        _objHome = _object->getMovableFrame()->getTransform(state);

    _config.add<bool>("ShowDebug","If enabled, all contacts and simulation geometry is visualized", false);

    // TODO: body sensor, wrench space analysis, if choosen

    if(_hand!=NULL && _tcp!=NULL && _object!=NULL && _mbase!=NULL && _dhand!=NULL){
        _loadTaskBtn->setEnabled(true);
        _saveResultBtn->setEnabled(true);

        _configured = true;
    } else {
        _loadTaskBtn->setEnabled(false);
        _saveResultBtn->setEnabled(false);
        _startBtn->setEnabled(false);
        _stopBtn->setEnabled(false);

        _configured = false;
    }
}

void SimTaskPlugin::saveConfig(){

    std::string simConfigFile("simConfigFile.prop.xml");
    QString selectedFilter;
    const QString file(simConfigFile.c_str());

    QString filename = QFileDialog::getSaveFileName(
        this,
        "Save config file", // Title
        file, // Directory/file
        "All supported ( *.xml )"
        " \nRW TASK files ( *.task.xml )"
        " \n All ( *.* )",
        &selectedFilter);

    simConfigFile = filename.toStdString();

    if(simConfigFile=="")
        return;

    log().info() << "Saving configuration: \n";
    log().info() << "\t-Filename: " << simConfigFile << "\n";

    try {
        XMLPropertySaver::save(_config, simConfigFile);
    } catch(...) {
        QMessageBox::information(this, "SimTaskPlugin", "SimTaskConfig could not be loadet!");
    }
}

rw::common::PropertyMap& SimTaskPlugin::settings(){
    return getRobWorkStudio()->getPropertyMap().get<rw::common::PropertyMap>("RobWorkStudioSettings");
}


void SimTaskPlugin::loadTasks(bool automatic){
    if(!_configured){
        log().error() << "Cannot load tasks before configuration!\n";
        return;
    }
    std::string prevDir = settings().get<std::string>("RWSimLastOpennedDIR","");
    std::string filename = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("TaskTestFile", "");
    std::string taskFile = filename;

    if(!automatic){

        QString selectedFilter;
        if(filename=="")
            filename = prevDir;
        const QString dir(filename.c_str());

        QString filename = QFileDialog::getOpenFileName(
            this,
            "Open Task file", // Title
            dir, // Directory
            "All supported ( *.xml )"
            " \nRW TASK files ( *.task.xml )"
            " \n All ( *.* )",
            &selectedFilter);

        taskFile = filename.toStdString();
    }

    if(taskFile=="")
        return;
    log().info() << "Loading tasks: ";
    log().info() << "\t-Filename: " << taskFile;

    try {
        XMLTaskLoader loader;
        loader.load( taskFile );
        _tasks = loader.getCartesianTask();
    } catch (const Exception& exp) {
        QMessageBox::information(this, "SimTaskPlugin", "Unable to load tasks from file");
        return;
    }
    _targets = &_tasks->getTargets();

    _showTaskSpinBox->setRange(0, _tasks->getTargets().size() );
    _nrTaskSpinBox->setRange( 0, _tasks->getTargets().size() );
    _nrTaskSpinBox->setValue( _tasks->getTargets().size() );
    log().info() << "LOAD TASKS DONE, nr of tasks: " << _tasks->getTargets().size();


    _wTe_n = _tasks->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
    _wTe_home = _tasks->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
    _openQ = _tasks->getPropertyMap().get<Q>("OpenQ", _openQ);
    _closeQ = _tasks->getPropertyMap().get<Q>("CloseQ", _closeQ);
    log().info() << "openQ" << _openQ ;
    log().info() << "closeQ" << _closeQ ;

    _startBtn->setEnabled(true);
    _stopBtn->setEnabled(true);

}

void SimTaskPlugin::stateChangedListener(const State& state) {

}

std::vector<rw::sensor::Contact3D> SimTaskPlugin::getObjectContacts(const rw::kinematics::State& state){
    const std::vector<rw::sensor::Contact3D>& contacts = _bsensor->getContacts();
    const std::vector<Body*>& bodies = _bsensor->getBodies();
    RW_ASSERT(bodies.size() == contacts.size() );

    std::vector<rw::sensor::Contact3D> contactres;
    std::map<std::string, Frame*> frameTree = Kinematics::buildFrameMap( *_hand->getBase(),  state);
    for(int i=0; i<bodies.size(); i++){
        if( bodies[i]!=NULL ){
            // test that the body frame is part of the gripper

            if( frameTree.find(bodies[i]->getBodyFrame()->getName() ) != frameTree.end() ){
                contactres.push_back(contacts[i]);
            }

        }
    }
    return contactres;
}

/**

-   Move the gripper in the start transform
-   If collision in start pose
    o   Set TestStatus = CollisionInitially
    o   Continue;
-   Simulate the gripper closing
-   If distance between jaws=0
    o   Set TestStatus = ObjectMissed
    o   Continue
-   Simulate moving to home pose
-   If object dropped
    o   Set TestStatus = ObjectDropped
    o   Continue
-   Set TestStatus = Success
-   Store Gripper Configuration



 */

void SimTaskPlugin::step(const rw::kinematics::State& state){
    std::cout <<_sim->getTime() << "   ";
    if( _stopped ){
        return;
    }
    if(_sim->getTime()>5.0 ){
        (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _graspedQ[0]);
        (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", TimeOut);
        _currentState = NEW_GRASP;
    }

    Transform3D<> cT3d = Kinematics::worldTframe(_object->getBodyFrame(), state);

    if(_currentState!=NEW_GRASP ){
        if( _tsim->isInError() ) {
            std::cout << "SIMULATION FAILURE1: " << std::endl;
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _graspedQ[0]);
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", SimulationFailure);
            _restObjTransform = Kinematics::frameTframe(_mbase, _object->getBodyFrame(), state);
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<Transform3D<> >("GripperTObject", _restObjTransform);
            _currentState = NEW_GRASP;
        } else if( MetricUtil::dist2(_objHome.P(),cT3d.P()) > _maxObjectGripperDistance ){
            std::cout <<_sim->getTime() << " : ";
            std::cout << "TASK FAILURE1: " << MetricUtil::dist2(_objHome.P(),cT3d.P()) << ">" << 0.5 << std::endl;
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _graspedQ[0]);
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", SimulationFailure);
            _restObjTransform = Kinematics::frameTframe(_mbase, _object->getBodyFrame(), state);
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<Transform3D<> >("GripperTObject", _restObjTransform);
            _currentState = NEW_GRASP;
        } /*else if( _objHome.P()[2] > (cT3d.P()[2]+0.04) ){
            std::cout <<_sim->getTime() << " : ";
            std::cout << "TASK FAILURE2: " << _objHome.P()[2] << ">" << (cT3d.P()[2]+0.04) << std::endl;
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _graspedQ[0]);
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", SimulationFailure);
            _restObjTransform = Kinematics::frameTframe(_mbase, _object->getBodyFrame(), state);
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<Transform3D<> >("GripperTObject", _restObjTransform);

            _currentState = NEW_GRASP;
        }*/
    }

    //std::cout << "step callback" << std::endl;
    if(_currentState==GRASPING){
        //std::cout << "grasping" << std::endl;
        _graspedQ = _hand->getQ(state);
        if(_sim->getTime()>1.2){
            // test if the grasp is in rest
            bool isResting = DynamicUtil::isResting(_dwc, state);
            //std::cout << isResting << "&&" << _sim->getTime() << "-" << _restingTime << ">0.08" << std::endl;
            // if it is in rest then lift object
            if( (isResting && ( (_sim->getTime()-_restingTime)>0.08)) || _sim->getTime()>3 ){
                // remember to check the transform of object relative to gripper
                _restObjTransform = Kinematics::frameTframe(_mbase, _object->getBodyFrame(), state);
                _graspTime = _sim->getTime();


                // now instruct the RigidBodyController to move the object to the home configuration
                State nstate = state;
                //_mbase->setTransform(_home, nstate);

                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _graspedQ[0]);
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<Transform3D<> >("GripperTObject", _restObjTransform);
                /*
                if(_graspedQ[0]<0.001){
                    (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", ObjectMissed);
                    _currentState = NEW_GRASP;
                } else
                */
                {
                    _sim->setTarget(_dhand->getBase(), _home, nstate);
                    _tsim->reset(nstate);
                    _currentState = LIFTING;
                }
            }
            if( !isResting ){
                _restingTime = _sim->getTime();
            }
            /*
            if(_graspedQ[0]<0.001){
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _graspedQ[0]);
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", ObjectMissed);
                _currentState = NEW_GRASP;
            }
            */

        } else {
            _restingTime = _sim->getTime();
        }
    }

    if(_currentState==LIFTING){
        // test if object has been lifted
        bool isLifted = true;
        Transform3D<> ct3d = Kinematics::worldTframe(_dhand->getBase()->getBodyFrame(), state);
        isLifted &= MetricUtil::dist2( ct3d.P(), _home.P() )<0.001;
        //isLifted &= ct3d.R().equal(_home.R(),0.01);

        // if its lifted then verify the object gripper transform
        if(isLifted){
            Transform3D<> t3d = Kinematics::frameTframe(_mbase, _object->getBodyFrame(), state);
            _graspedQ = _hand->getQ(state);
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<Transform3D<> >("GripperTObject", t3d);
            /*
            if(_graspedQ[0]<0.001){
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", ObjectDropped);
            } else
            */
            if((MetricUtil::dist2(_restObjTransform.P(),t3d.P())>0.02 && _graspedQ[0]<0.002)){
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", ObjectDropped);
                std::cout <<_sim->getTime() << " : " << "ObjectDropped" << std::endl;
            } else {
                // we are relatively successfull, so calculate the quality of the grasp
                std::vector<Contact3D> contacts = getObjectContacts(state);

                // calculate grasp quality
                rw::math::Q qualities( Q::zero(2) );
                Grasp3D g3d( contacts );

                std::cout << "***** NR OF CONTACTS IN GRASP: " << g3d.contacts.size() << std::endl;
                Vector3D<> cm = _object->getInfo().masscenter;
                double r = GeometryUtil::calcMaxDist( _object->getGeometry(), cm);
                std::cout << "Wrench calc" << std::endl;
                rw::graspplanning::sandbox::WrenchMeasure3D wmeasure( 20 );
                wmeasure.setObjectCenter(cm);
                wmeasure.setLambda(1/r);
                wmeasure.quality(g3d);
                std::cout << "Wrench calc done!" << std::endl;
                qualities(0) = wmeasure.getMinWrench();
                CMDistCCPMeasure3D CMCPP( cm, 0.3);
                qualities(1) = CMCPP.quality( g3d );
                std::cout << "Quality: " << qualities << std::endl;
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<Q>("Quality", qualities);

                if( _restObjTransform.R().equal(t3d.R(), 0.01 ) && (MetricUtil::dist2(_restObjTransform.P(),t3d.P())<0.006 ) ) {
                    (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", Success);
                    std::cout <<_sim->getTime() << " : " << "Success" << std::endl;
                } else {
                    (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", ObjectSlipped);
                    std::cout <<_sim->getTime() << " : " << "ObjectSlipped" << std::endl;
                }
            }
            _currentState = NEW_GRASP;
        }
        _graspedQ = _hand->getQ(state);

        /*
        if(_graspedQ[0]<0.001){
            (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", ObjectDropped);
            _currentState = NEW_GRASP;
        }
        */
    }

    if(_currentState==NEW_GRASP){
        State nstate = getRobWorkStudio()->getState();
        // pop new task from queue
        // if all tasks

        bool colFreeSetup = false;
        do{
            if(_nextTaskIndex>=(int)_targets->size()){
                // end we are done
                _stopped = true;
                std::cout << "STOP" << std::endl;
                // save the result
                return;
            }

            Transform3D<> wTp = Kinematics::worldTframe(_mbase->getParent(state), state);
            Transform3D<> start = inverse(wTp) * _wTe_n * (*_targets)[_nextTaskIndex]->get() * inverse(_bTe);

            _mbase->setTransform(start, nstate);
            // and calculate the home lifting position
            _home = _wTe_home * (*_targets)[_nextTaskIndex]->get() * inverse(_bTe);
            //std::cout << "START: " << start << std::endl;
            //std::cout << "HOME : " << _home << std::endl;

            _hand->setQ(_openQ, nstate);
            _object->getMovableFrame()->setTransform(_objHome, nstate);

            colFreeSetup = !getRobWorkStudio()->getCollisionDetector()->inCollision(nstate, NULL, true);
            _nextTaskIndex++;

            std::cout << "Current index: " << (_nextTaskIndex-1) << std::endl;
            if( !colFreeSetup ){
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<int>("TestStatus", CollisionInitially);
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<double>("GripperConfiguration", _openQ[0]);
                Transform3D<> bTobj = Kinematics::frameTframe(_mbase, _object->getMovableFrame(), nstate);
                (*_targets)[_nextTaskIndex-1]->getPropertyMap().set<Transform3D<> >("GripperTObject", bTobj);
                std::cout << "0.0 : InCollision " << std::endl;
            }
        } while( !colFreeSetup );
        if( _nextTaskIndex > _lastSaveTaskIndex+40 ){
            saveTasks(true);
            _lastSaveTaskIndex = _nextTaskIndex;
        }
        // reset simulation
        _dhand->getBase()->reset(nstate);
        _tsim->reset(nstate);
        _sim->disableBodyControl();
        _controller->setTargetPos(_closeQ);
        std::cout << "CLOSEQ: " << _closeQ << "  " << _openQ << std::endl;
        _currentState = GRASPING;
        _restingTime = 0;
    }
}

void SimTaskPlugin::makeSimulator(){
    if(_sim != NULL)
        return;

    // we have a DWC create the simulator
    State state = getRobWorkStudio()->getState();
    log().debug() << "Making physics engine";
    ODESimulator::Ptr _engine = ownedPtr( new ODESimulator(_dwc));
    log().debug() << "Making simulator";
    _sim = ownedPtr( new DynamicSimulator(_dwc, _engine ));
    std::cout << "Initializing simulator";
    try {
        _sim->init(state);
    } catch(...){
        log().error() << "could not initialize simulator!\n";
    }
    log().debug() << "Creating Thread simulator";

    _bsensor = ownedPtr(new BodyContactSensor("SimTaskObjectSensor", _object->getBodyFrame()));
    _sim->addSensor( _bsensor );

    _tsim = ownedPtr( new ThreadSimulator(_sim, state) );
    ThreadSimulator::StepCallback cb( boost::bind(&SimTaskPlugin::step, this, _1) );
    _tsim->setStepCallBack( cb );
    _tsim->setPeriodMs(-1);
    _tsim->setTimeStep(0.001);

    rwsim::drawable::SimulatorDebugRender::Ptr debugRender = _sim->createDebugRender();
    if( debugRender == NULL ){
        Log::errorLog() << "The current simulator does not support debug rendering!" << std::endl;
        return;
    }

    debugRender->setDrawMask( 1 );
    rwlibs::opengl::Drawable *debugDrawable = new rwlibs::opengl::Drawable( debugRender, "DebugRender" );

    getRobWorkStudio()->getWorkCellScene()->addDrawable(debugDrawable, _dwc->getWorkcell()->getWorldFrame());
}

/*
void SimTaskPlugin::loadTasks(){
    std::string filename = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("TaskTestFile", "");
    log().info() << "Loading tasks: ";
    log().info() << "\t-Filename: " << filename;

    try {
        XMLTaskLoader loader;
        loader.load(filename);
        _tasks = loader.getCartesianTask();
    } catch (const Exception& exp) {
        QMessageBox::information(this, "SimTaskPlugin", "Unable to load tasks from file");
        return;
    }
    log().info() << "LOAD TASKS DONE, nr of tasks: " << _tasks->getTargets().size();
}
*/
void SimTaskPlugin::exportMathematica(const std::string& filename) {

       log().info() << "Saving tasks in mathematica format: ";
       log().info() << "\t-Filename: " << filename;

       std::ofstream outfile(filename.c_str());

       std::vector<CartesianTarget::Ptr> targets = _tasks->getTargets();
       BOOST_FOREACH(CartesianTarget::Ptr target, targets) {
                  const Vector3D<>& pos = target->get().P();
                  const RPY<> rpy(target->get().R());
                  int status = target->getPropertyMap().get<int>("TestStatus", UnInitialized);
                  double distance = target->getPropertyMap().get<double>("GripperConfiguration", -1);
                  Transform3D<> t3d = target->getPropertyMap().get<Transform3D<> >("GripperTObject", Transform3D<>::identity());
                  RPY<> rpyObj(t3d.R());
                  outfile<<"{"<<pos(0)<<","<<pos(1)<<","<<pos(2)<<","<<rpy(0)<<","<<rpy(1)<<","<<rpy(2)<<","<<status<<","<<distance << ","
                          << t3d.P()[0] << "," << t3d.P()[1] << "," <<t3d.P()[2] << ","
                          << rpyObj(0) << "," << rpyObj(1) << "," <<rpyObj(2) << "}"<<std::endl;

       }
       outfile.close();
}


void SimTaskPlugin::saveTasks(bool automatic){

    std::string filename = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("TaskTestOutFile", "test.task.xml");
    std::string taskFile = filename;
    if(!automatic){

        QString selectedFilter;
        const QString file(filename.c_str());

        QString filename = QFileDialog::getSaveFileName(
            this,
            "Open Task file", // Title
            file, // Directory/file
            "All supported ( *.xml )"
            " \nRW TASK files ( *.task.xml )"
            " \n All ( *.* )",
            &selectedFilter);

        taskFile = filename.toStdString();
    }

    log().info() << "Saving tasks: \n";
    log().info() << "\t-Filename: " << taskFile << "\n";

    try {
        XMLTaskSaver saver;
        saver.save(_tasks, taskFile );

    } catch (const Exception& exp) {
        QMessageBox::information(this, "Task Execution Widget", "Unable to save tasks");
    }
    std::stringstream sstr;
    sstr << taskFile << ".m.txt";
    exportMathematica(sstr.str());
}


void SimTaskPlugin::genericEventListener(const std::string& event){
    if( event=="DynamicWorkcellLoadet" ){
        // get the dynamic workcell from the propertymap
        RW_DEBUG("Getting dynamic workcell from propertymap!");

        DynamicWorkCell::Ptr dwc =
            getRobWorkStudio()->getPropertyMap().get<DynamicWorkCell::Ptr>("DynamicWorkcell",NULL);

        if( dwc==NULL){
            std::cout << "Could not load dynamic workcell from propertymap!!" << std::endl;
            return;
        }
        //std::cout << "dwc" << dwc->getWorkCell()->getName() << std::endl;
        _dwc = dwc;
    } else if( event=="ExecuteSimulationTask" ){
        _tasks = getRobWorkStudio()->getPropertyMap().get<rwlibs::task::CartesianTask::Ptr>("SimulationTask", NULL);
        if(_tasks!=NULL)
            startSimulation();
    }
}

Q_EXPORT_PLUGIN(SimTaskPlugin);
