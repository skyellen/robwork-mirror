#include "SimTaskPlugin.hpp"

#include <rw/graspplanning/CMDistCCPMeasure3D.hpp>
#include <rw/graspplanning/Grasp3D.hpp>
#include <rw/graspplanning/GWSMeasure3D.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rwlibs/control/JointController.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>
#include <rwlibs/task/loader/XMLTaskSaver.hpp>

#include <rws/RobWorkStudio.hpp>
#include <rws/propertyview/PropertyViewEditor.hpp>

#include <rwsim/drawable/SimulatorDebugRender.hpp>
#include <rwsim/dynamics/DynamicDevice.hpp>
#include <rwsim/dynamics/DynamicUtil.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>

#include <QFileDialog>
#include <QPushButton>
#include <QMessageBox>
#include <QTimer>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <stack>

using namespace rw::common;
using rw::geometry::GeometryUtil;
using namespace rw::graspplanning;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using rw::sensor::Contact3D;
using rwlibs::proximitystrategies::ProximityStrategyFactory;
using rwlibs::simulation::SimulatedController;
using namespace rwlibs::task;
using rws::RobWorkStudioPlugin;
using namespace rwsim::dynamics;
using rwsim::sensor::BodyContactSensor;
using namespace rwsim::simulator;

const int NR_OF_QUALITY_MEASURES = 3;

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

    _timer->setInterval( 100 );

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
    getRobWorkStudio()->stateChangedEvent().add(
            boost::bind(&SimTaskPlugin::stateChangedListener, this, _1), this);

    getRobWorkStudio()->genericEvent().add(
          boost::bind(&SimTaskPlugin::genericEventListener, this, _1), this);

    Log::setLog( _log );
}

void SimTaskPlugin::startSimulation() {

    if(_taskQueue.size()==0){
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

    //if(_object==NULL){
    //    log().error() << "Object does not exist.";
    //    return;
    //}

    if(_tcp==NULL){
        log().error() << "End frame  does not exist.";
        return;
    }

    log().info() << "Starting simulation!";
    setTask( getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get("StartIdx", 0) );
    _nrOfExperiments = 0;
    _lastSaveTaskIndex = 0;
    _failed=0;
    _success=0;
    _slipped=0;
    _collision=0;
    _simfailed=0;
    _timeout=0;
    //_lastSaveTaskIndex = _nextTaskIndex;
    _stopped = false;
    SimulatedController* scontroller = _dwc->findController("GraspController").get();
    if(scontroller==NULL)
        RW_THROW("No controller!");
    _controller = scontroller->getControllerHandle(_sim).cast<rwlibs::control::JointController>();
    _progressBar->setRange( 0 , _totalNrOfExperiments);
    _progressBar->setValue( 0 );
    _currentState = NEW_GRASP;
    _wallTimer.resetAndResume();
    _wallTotalTimer.resetAndResume();


    _timer->start();
    //_tsim->setInError(true);
    _tsim->start();
    log().info() << "Simulation Started\n";
}

void SimTaskPlugin::open(WorkCell* workcell)
{
    if(workcell==NULL || _dwc==NULL)
        return;

    _nrOfExperiments = 0;
    _lastSaveTaskIndex = 0;
    _failed=0;
    _success=0;
    _slipped=0;
    _collision=0;
    _simfailed=0;
    _timeout=0;
    _skipped=0;

    _wc = workcell;
    _simState = workcell->getDefaultState();
    _collisionDetector = ownedPtr(
        new CollisionDetector( workcell, ProximityStrategyFactory::makeDefaultCollisionStrategy()));

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
    // destroy simulation and stuff
    _wc = NULL;
    //_dwc = NULL;
    _taskQueue.clear();
    _currenttask = NULL;
    _tsim = NULL;
    _sim = NULL;
    _hand = NULL;
    _dhand = NULL;
    _mbase = NULL;
    _tcp = NULL;
    _objects.clear();
    _bsensors.clear();
    //_object = NULL;
    //_object2 = NULL;
    _controller = NULL;
    //_bsensor = NULL;
    //_bsensor2 = NULL;

    _loadConfigBtn->setEnabled(false);
    _saveConfigBtn->setEnabled(false);
    _loadTaskBtn->setEnabled(false);
    _saveResultBtn->setEnabled(false);
    _startBtn->setEnabled(false);
    _stopBtn->setEnabled(false);

    _configured = false;

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

        if(idxTmp<0 || idxTmp >= (int)_alltargets.size()){
            log().error() << "Choosen task is out of bounds\n";
            return;
        }
        CartesianTask::Ptr task = _alltargets[idxTmp].first;
        CartesianTarget::Ptr target = _alltargets[idxTmp].second;

        Transform3D<> wTe_n = task->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
        //Transform3D<> wTe_home = task->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
        //Vector3D<> approach = _currenttask->getPropertyMap().get<Vector3D<> >("Approach", Vector3D<>(0,0,0));
        //Transform3D<> approachDef = Transform3D<>( approach, Rotation3D<>::identity());
        Q openQ = task->getPropertyMap().get<Q>("OpenQ", _openQ);
        //Q closeQ = _currenttask->getPropertyMap().get<Q>("CloseQ", _closeQ);


        Transform3D<> wTp = Kinematics::worldTframe(_mbase->getParent(state), state);
        Transform3D<> start = inverse(wTp) * wTe_n * target->get() * inverse(_bTe);
        _mbase->setTransform(start, state);

        // and calculate the home lifting position
        //_home = wTe_home * target->get() * inverse(_bTe);
        _hand->setQ(openQ, state);
        BOOST_FOREACH(RigidBody::Ptr object, _objects){
            Transform3D<> objHome = object->getMovableFrame()->getTransform( _homeState );
            object->getMovableFrame()->setTransform(objHome, state );
        }
        //std::cout << "eTb: " << inverse(_bTe) << std::endl;
        //std::cout << "TARGET: " << (*_targets)[idxTmp]->get() << std::endl;
        //std::cout << "HOME: " << _home << std::endl;

        getRobWorkStudio()->setState(state);
    } else if(obj==_timer){
        // update the RobWorkStudio state
        //State state = _tsim->getState();
        getRobWorkStudio()->setState(_simState);

        _wallTimeLbl->setText( _wallTotalTimer.toString("hh:mm:ss").c_str() );
        _simTimeLbl->setText( Timer(_simTime*1000).toString("hh:mm:ss:zzz").c_str() );
        int nrgrasps = _failed + _success + _slipped + _collision + _timeout + _simfailed;
        double avgGraspTime = (_wallTotalTimer.getTime()/(_failed + _success + _slipped + _collision + _timeout + _simfailed))*1000;
        _timePerGraspLbl->setText( Timer((int)avgGraspTime).toString("ss:zzz").c_str() );
        _timeToFinishLbl->setText( Timer((int)(avgGraspTime* (_totalNrOfExperiments-nrgrasps))).toString("hh:mm").c_str() );
        if(_stopped){
            _tsim->stop();
            _timer->stop();
            if(getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").has("Auto")){
                std::cout << "AUTO CLOSE ACTIVATED, exiting RobWorkStudio" << std::endl;
                //saveTasks(true);
                getRobWorkStudio()->postExit();
            } else {
                std::cout << "AUTO CLOSE DEACTIVATED" << std::endl;
            }
        } else {
            // update progress
            _progressBar->setValue( _nrOfExperiments );
        }
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
        log().info() << "Loading tasks: ";
        log().info() << "\t-Filename: " << simTaskConfigFile;
        try {
            _config = XMLPropertyLoader::load( simTaskConfigFile );
        } catch(...) {
            QMessageBox::information(this, "SimTaskPlugin", "SimTaskConfig could not be loaded!");
        }
    }
    updateConfig();

    // for some reason this crashes sometimes...
    _propertyView->setPropertyMap( &_config );
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
    _hand =_wc->findDevice<Device>(devName).get();
    _dhand = _dwc->findDevice(devName);
    _rhand = _dhand.cast<RigidDevice>();

    std::string baseName;
    if( !_config.has("MovableBase") || _config.get<std::string>("MovableBase")==""){
        // check all frames from device base to world
        if(_hand != NULL ){
            std::vector<Frame*> frames = Kinematics::childToParentChain(_hand->getBase(), _wc->getWorldFrame(), state);
            BOOST_FOREACH(Frame *tmpKinFrame, frames){
                if( KinematicBody::Ptr kbody = _dwc->findBody<KinematicBody>(tmpKinFrame->getName()) ){
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

    /*
    std::string objName;
    if( !_config.has("ObjectName") || _config.get<std::string>("ObjectName")=="" ){
        // find the first rigid body if no ObjectName specified in the config file
        std::vector<RigidBody*> rbodies = _dwc->findBodies<RigidBody>();
        if(rbodies.size()>0)
            objName = rbodies[0]->getName();
        _config.add<std::string>("ObjectName","Name of the object that is to be grasped", objName);
    }
    objName = _config.get<std::string>("ObjectName");
    RigidBody *object = _dwc->findBody<RigidBody>(objName);
    if(object!=NULL){

        _objects.push_back(object);
    }

    // TWOOBJ: check if ObjectName2 exists
    std::string objName2;
    if( _config.has("ObjectName2") && _config.get<std::string>("ObjectName2")!="" ){
      objName2 = _config.get<std::string>("ObjectName2");
      object = _dwc->findBody<RigidBody>(objName2);
      if(object!=NULL)
          _objects.push_back(object);
    }
*/
    _objects.clear();
    std::vector<RigidBody::Ptr> rbodies = _dwc->findBodies<RigidBody>();
    BOOST_FOREACH(RigidBody::Ptr object, rbodies){
    	if(object->getBodyFrame()->getName()=="EndFrame")
    		continue;
    	std::map<std::string, Frame*> fmap = Kinematics::buildFrameMap(_hand->getBase(), state);
        if( fmap.find(object->getBodyFrame()->getName()) ==fmap.end())
        	_objects.push_back(object);
    }
    if( !_config.has("CalculateWrenchQuality") ){
        _config.add<bool>("CalculateWrenchQuality","Set true if the quality of the grasp should be calculated", true);
    }
    //_calcWrenchQuality = _config.get<bool>("CalculateWrenchQuality");

    if( !_config.has("MaxObjectGripperDistance") ){
        _config.add<double>("MaxObjectGripperDistance","The maximum allowed distance between gripper and object", 50.0);
    }
    _maxObjectGripperDistance = _config.get<double>("MaxObjectGripperDistance", 50);


    if(_hand!=NULL ){
        _openQ = _config.get<Q>("DefOpenQ", _hand->getQ(state));
        _closeQ = _config.get<Q>("DefCloseQ", _hand->getQ(state));
    }



    if(_mbase!=NULL && _tcp!=NULL)
        _bTe = Kinematics::frameTframe(_mbase, _tcp, state);

    _homeState = state;
    _restObjState = state;

    _config.add<bool>("ShowDebug","If enabled, all contacts and simulation geometry is visualized", false);

    log().info()
            << (_hand!=NULL) <<"&&"
            << (_tcp!=NULL) <<"&&"
            << (_objects.size()>0) <<"&&"
            << (_mbase!=NULL) <<"&&"
            << (_dhand!=NULL) <<"&&"<< "\n";
    // TODO: body sensor, wrench space analysis, if choosen
    if(_hand!=NULL && _tcp!=NULL && _objects.size()>0 && _mbase!=NULL && _dhand!=NULL){
        _loadTaskBtn->setEnabled(true);
        _saveResultBtn->setEnabled(true);
        _configured = true;
    } else {
        //std::cout << (_hand!=NULL) <<" && " << (_tcp!=NULL) <<" && " << (_objects.size()>0) <<" && " << (_mbase!=NULL) <<" && " << (_dhand!=NULL) << std::endl;
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
    _taskQueue.clear();
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
    rwlibs::task::CartesianTask::Ptr task;
    try {
        XMLTaskLoader loader;
        loader.load( taskFile );
        task = loader.getCartesianTask();
    } catch (const Exception& exp) {
        QMessageBox::information(this, "SimTaskPlugin", "Unable to load tasks from file");
        return;
    }
    // iterate over all tasks and add them to the taskQueue
    _roottask = task;
    int nrOfTargets = 0;
    std::stack<rwlibs::task::CartesianTask::Ptr> tmpStack;
    tmpStack.push(task);
    while(!tmpStack.empty()){
        rwlibs::task::CartesianTask::Ptr tmpTask = tmpStack.top();
        tmpStack.pop();

        _taskQueue.push_back(tmpTask);
        nrOfTargets += tmpTask->getTargets().size();

        BOOST_FOREACH(CartesianTarget::Ptr target, tmpTask->getTargets()){
            _alltargets.push_back( std::make_pair(tmpTask,target));
        }

        BOOST_FOREACH(rwlibs::task::CartesianTask::Ptr subtask, tmpTask->getTasks()){
            tmpStack.push(subtask);
        }
    }

    _totalNrOfExperiments = nrOfTargets;
    _showTaskSpinBox->setRange(0, _alltargets.size() );
    _nrTaskSpinBox->setRange( 0, _alltargets.size() );
    _nrTaskSpinBox->setValue( _alltargets.size()-1 );
    log().info() << "LOAD TASKS DONE, nr of tasks: " << nrOfTargets;
    setTask(0);

    _startBtn->setEnabled(true);
    _stopBtn->setEnabled(true);

    //_showTaskSpinBox->setValue(1);
    //_showTaskSpinBox->setValue(0);
}

void SimTaskPlugin::stateChangedListener(const State& state) {

}

std::vector<rw::sensor::Contact3D> SimTaskPlugin::getObjectContacts(const rw::kinematics::State& state, RigidBody::Ptr object, BodyContactSensor::Ptr sensor, std::vector<Body::Ptr>& gripperbodies ){
    const std::vector<rw::sensor::Contact3D>& contacts = sensor->getContacts(state);
    const std::vector<Body::Ptr>& bodies = sensor->getBodies(state);

    RW_ASSERT(bodies.size() == contacts.size() );
    //std::cout << "nr contacts: " << contacts.size() << " body: " << object->getName() << std::endl;
    std::vector<rw::sensor::Contact3D> contactres;
    std::map<std::string, Frame*> frameTree = Kinematics::buildFrameMap( _hand->getBase(),  state);
    frameTree[_hand->getBase()->getName()] = _hand->getBase();
    for(size_t i=0; i<bodies.size(); i++){
        if( bodies[i]!=NULL ){
            // test that the body frame is part of the gripper
            //std::cout << "Body: " << bodies[i]->getBodyFrame()->getName() << std::endl;
            if( frameTree.find(bodies[i]->getBodyFrame()->getName() ) != frameTree.end() ){
                if(contacts[i].normalForce>0.0001){
                    contactres.push_back(contacts[i]);
                    contactres.back().mu = _dwc->getMaterialData().getFrictionData(object->getMaterialID(),bodies[i]->getMaterialID()).parameters[0].second(0);
                    // allso save the body of the gripper that is in contact
                    if(std::find(gripperbodies.begin(), gripperbodies.end(), bodies[i]) == gripperbodies.end() )
                        gripperbodies.push_back(bodies[i]);
                }
            }
        } else {
            //std::cout << "Body: NULL" << std::endl;
        }
    }
    //std::cout << "Get CONTACTS " << contacts.size() << " --> " << contactres.size() << std::endl;
    return contactres;
}

SimTaskPlugin::GraspedObject SimTaskPlugin::getObjectContacts(const rw::kinematics::State& state)
{
    std::vector<GraspedObject> result;
    for(size_t i=0; i<_objects.size();i++){
        GraspedObject obj;
        obj.object = _objects[i];
        obj.contacts = getObjectContacts(state, _objects[i], _bsensors[i], obj.bodies);
        if(obj.contacts.size()>0)
            result.push_back(obj);
    }
    if(result.size()==0)
        return GraspedObject();
    int bestIdx = 0;
    for(size_t i=1;i<result.size();i++){
        if( result[i].contacts.size() > result[bestIdx].contacts.size() )
            bestIdx = i;
    }
    return result[bestIdx];
}

rw::math::Q SimTaskPlugin::calcGraspQuality(const State& state){
    GraspedObject gobj = getObjectContacts(state);
    std::vector<Contact3D> contacts = gobj.contacts;
    RigidBody::Ptr object = gobj.object;
    // calculate grasp quality
    rw::math::Q qualities( Q::zero(NR_OF_QUALITY_MEASURES) );
    if(gobj.object==NULL || gobj.contacts.size()==0)
        return qualities;
    Grasp3D g3d( contacts );

    if(g3d.contacts.size()<4){
        std::vector<Contact3D > cons = g3d.contacts;
        BOOST_FOREACH(Contact3D& c, cons){
            // add a small random value to normal and position
            c.n += Vector3D<>(Math::ran(-0.1,0.1), Math::ran(-0.1,0.1),Math::ran(-0.1,0.1));
            c.n = normalize(c.n);
            c.p += Vector3D<>(Math::ran(-0.002,0.002), Math::ran(-0.002,0.002),Math::ran(-0.002,0.002));
            g3d.contacts.push_back(c);
        }
    }


    std::cout << "***** NR OF CONTACTS IN GRASP: " << g3d.contacts.size() << std::endl;
    /*
    BOOST_FOREACH(Contact3D con, g3d.contacts){
        std::cout << "--- contact";
        std::cout << "\n-- nf: " << con.normalForce;
        std::cout << "\n-- mu: " << con.mu;
        std::cout << "\n-- n : " << con.n;
        std::cout << "\n-- p: " << con.p << std::endl;
    }
    */

    Vector3D<> cm = object->getInfo().masscenter;
    double r = GeometryUtil::calcMaxDist( object->getGeometry(), cm, object->getBodyFrame(), state);
    //std::cout << "cm    : " << cm << std::endl;
    //std::cout << "Radius: " << r<< std::endl;

    std::cout << "w2 "<<std::endl;
    rw::graspplanning::GWSMeasure3D wmeasure2( 10 , false);
    wmeasure2.setObjectCenter(cm);
    wmeasure2.setLambda(1/r);
    wmeasure2.quality(g3d);

    std::cout << "w3 "<<std::endl;
    //

    rw::graspplanning::GWSMeasure3D wmeasure3( 10, true );
    wmeasure3.setObjectCenter(cm);
    wmeasure3.setLambda(1/r);
    wmeasure3.quality(g3d);

    //std::cout << "getvals " << r<< std::endl;
    //std::cout << "Wrench calc done!" << std::endl;
    qualities(0) = wmeasure2.getMinWrench();
    qualities(1) = wmeasure3.getMinWrench();

    std::cout << "CMCPP " << r<< std::endl;
    CMDistCCPMeasure3D CMCPP( cm, r*2);
    qualities(2) = CMCPP.quality( g3d );
    std::cout << "Quality: " << qualities << std::endl;
    return qualities;
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


bool SimTaskPlugin::hasNextTarget(){
    if(_targets == NULL || _nextTargetIndex >= (int)_targets->size()){
        if(_currentTaskIndex+1 >= (int)_taskQueue.size()){
            return false;
        }
    }
    return true;
}

rwlibs::task::CartesianTask::Ptr SimTaskPlugin::getTask(){
    return _currenttask;
}

void SimTaskPlugin::setTask(int i){
    if(i<0 || i>= (int)_taskQueue.size())
        return;

    _currentTaskIndex = i;
    _currenttask = _taskQueue[i];
    _targets = &_currenttask->getTargets();
    _nextTargetIndex = 0;
/*
    log().info() << "Setting task:" << _currenttask->getId()
                 << "\n-- task nr:"<< i
                 << " success:"<< _success
                 << " slipped:" << _slipped
                 << " failed:" << _failed
                 << " collisions:" << _collision
                 << " timeouts:" << _timeout
                 << " simfailures:" << _simfailed << "\n";
*/

    _wTe_n = _currenttask->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
    _wTe_home = _currenttask->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
    Vector3D<> approach = _currenttask->getPropertyMap().get<Vector3D<> >("Approach", Vector3D<>(0,0,0));
    _approachDef = Transform3D<>( approach, Rotation3D<>::identity());
    _openQ = _currenttask->getPropertyMap().get<Q>("OpenQ", _openQ);
    _closeQ = _currenttask->getPropertyMap().get<Q>("CloseQ", _closeQ);

    //log().info() << "openQ" << _openQ << "\n";
    //log().info() << "closeQ" << _closeQ << "\n";
}

rwlibs::task::CartesianTarget::Ptr SimTaskPlugin::getNextTarget(){
    // were we iterate over all tasks and their targets

    if(_targets == NULL || _nextTargetIndex>= (int)_targets->size()){
        // get the next task and reinitialize _targets and _currentTaskIndex
        if(_currentTaskIndex+1 >= (int)_taskQueue.size()){
            return NULL; // there is no more tasks
        }
        setTask(_currentTaskIndex+1);
    }
    _currentTargetIndex = _nextTargetIndex;
    log().info() << "-- target nr: "<< std::setw(5) << _currentTaskIndex
                 << " success:"<< std::setw(5) << _success
                 << " slipped:" << std::setw(5) << _slipped
                 << " failed:" << std::setw(5) << _failed
                 << " collisions:" << std::setw(5) << _collision
                 << " timeouts:" << std::setw(5) << _timeout
                 << " skipped:" << std::setw(5) << _skipped
                 << " simfailures:" << std::setw(5) <<_simfailed << "\n";

    _nextTargetIndex++;
    return (*_targets)[ _currentTargetIndex ];
}

rwlibs::task::CartesianTarget::Ptr SimTaskPlugin::getTarget(){
    return (*_targets)[ _currentTargetIndex ];
}

namespace {
    double getMaxObjectDistance(std::vector<RigidBody::Ptr> objects, const State& s1, const State& s2){
        double max = 0;
        BOOST_FOREACH(RigidBody::Ptr object, objects){
            Transform3D<> t1 = object->getTransformW(s1);
            Transform3D<> t2 = object->getTransformW(s2);
            if(MetricUtil::dist2(t1.P(),t2.P())>max)
                max = MetricUtil::dist2(t1.P(),t2.P());
        }
        return max;
    }
}

void SimTaskPlugin::step(ThreadSimulator* sim, const rw::kinematics::State& state){
    //std::cout <<_sim->getTime() << "    " << std::endl;
    int delay = _delaySpin->value();
    _simTime = _sim->getTime();
    if( delay!= 0 )
        TimerUtil::sleepMs(delay);
    if( _stopped ){
        return;
    }
    _simState = state;

    if(_wallTimer.getTime()>60){ //seconds
        _timeout++;
        getTarget()->getPropertyMap().set<Q>("GripperConfiguration", _graspedQ);
        getTarget()->getPropertyMap().set<int>("TestStatus", TimeOut);
        _currentState = NEW_GRASP;
    }

    if(_sim->getTime()>20.0 && _currentState != NEW_GRASP){
        _timeout++;
        getTarget()->getPropertyMap().set<Q>("GripperConfiguration", _graspedQ);
        getTarget()->getPropertyMap().set<int>("TestStatus", TimeOut);
        _currentState = NEW_GRASP;
    }

    //Transform3D<> cT3d = Kinematics::worldTframe(_object->getBodyFrame(), state);
    if( _tsim->isInError() ) {
        // the simulator is in error, reinitialize or fix the error
        _simfailed++;
        std::cout << "SIMULATION FAILURE0: " << std::endl;
        getTarget()->getPropertyMap().set<Q>("GripperConfiguration", _graspedQ);
        getTarget()->getPropertyMap().set<int>("TestStatus", SimulationFailure);
        _restObjState = state;
        for(size_t i=0; i<_objects.size(); i++){
            Transform3D<> restTransform = Kinematics::frameTframe(_mbase, _objects[i]->getBodyFrame(), state);
            getTarget()->getPropertyMap().set<Transform3D<> >("GripperTObject"+boost::lexical_cast<std::string>(i), restTransform);
        }
        _currentState = NEW_GRASP;
    }

    if(_currentState!=NEW_GRASP ){
        if( getMaxObjectDistance( _objects, _homeState, state) > _maxObjectGripperDistance ){
            double mdist=getMaxObjectDistance( _objects, _homeState, state);
            _simfailed++;
            std::cout <<_sim->getTime() << " : ";
            std::cout << "TASK FAILURE1: " << mdist << ">" << 0.5 << std::endl;
            getTarget()->getPropertyMap().set<Q>("GripperConfiguration", _graspedQ);
            getTarget()->getPropertyMap().set<int>("TestStatus", SimulationFailure);
            _restObjState = state;
            for(size_t i=0; i<_objects.size(); i++){
                Transform3D<> restTransform = Kinematics::frameTframe(_mbase, _objects[i]->getBodyFrame(), state);
                getTarget()->getPropertyMap().set<Transform3D<> >("GripperTObject"+boost::lexical_cast<std::string>(i), restTransform);
            }
            _currentState = NEW_GRASP;
        }
    }

    if(_currentState ==APPROACH){

        Transform3D<> ct3d = Kinematics::worldTframe(_dhand->getBase()->getBodyFrame(), state);
        bool isLifted = MetricUtil::dist2( ct3d.P(), _approach.P() )<0.002;
        //std::cout << MetricUtil::dist2( ct3d.P(), _approach.P() ) << " < " << 0.002 << std::endl;
        //if(_sim->getTime()>1.2){

        //std::cout << "APPROACH: " << std::endl;
        if(isLifted){
            std::cout << "GRASPING" << std::endl;
            _controller->setTargetPos(_closeQ);
            _currentState=GRASPING;
            _approachedTime = _sim->getTime();
            _restingTime = _approachedTime;

            Transform3D<> t3d  = Kinematics::frameTframe(_tcp, _objects[0]->getBodyFrame(), state);
            getTarget()->getPropertyMap().set<Transform3D<> > ("ObjectTtcpApproach", inverse(t3d) );

        }
    }


    //std::cout << "step callback" << std::endl;
    if(_currentState==GRASPING){
        //std::cout << "grasping" << std::endl;
        _graspedQ = _hand->getQ(state);
        if(_sim->getTime()> _approachedTime+0.2){
            // test if the grasp is in rest
            bool isResting = DynamicUtil::isResting(_dhand, state, 0.02);
            //std::cout << isResting << "&&" << _sim->getTime() << "-" << _restingTime << ">0.08" << std::endl;
            // if it is in rest then lift object
            if( (isResting && ( (_sim->getTime()-_restingTime)>0.08)) || _sim->getTime()>10 ){
                // remember to check the transform of object relative to gripper
                //_restObjTransform = Kinematics::frameTframe(_mbase, _object->getBodyFrame(), state);
                _restObjState = state;
                _graspTime = _sim->getTime();

                _postLiftObjState = state;
                //_objectBeginLift = _object->getBodyFrame()->getTransform(state);
                // now instruct the RigidBodyController to move the object to the home configuration
                State nstate = state;
                //_mbase->setTransform(_home, nstate);
                getTarget()->getPropertyMap().set<Q>("GripperConfiguration", _graspedQ);
                for(size_t i=0;i<_objects.size();i++){
                    getTarget()->getPropertyMap().set<Transform3D<> >("GripperTObject"+boost::lexical_cast<std::string>(i),
                                                                      _objects[i]->getTransformW(_restObjState));

                }
                GraspedObject gobj = getObjectContacts(state);
                if( gobj.object == NULL ){
                    _failed++;
                    std::cout << "NEW_GRASP" << std::endl;
                    std::cout << "ObjectMissed" << std::endl;
                    getTarget()->getPropertyMap().set<int>("TestStatus", ObjectMissed);
                    getTarget()->getPropertyMap().set<Q>("QualityBeforeLifting", Q::zero(2));
                    _currentState = NEW_GRASP;
                } else {
                    std::cout << "LIFTING" << std::endl;
                    Q qualities = calcGraspQuality(state);
                    getTarget()->getPropertyMap().set<Q>("QualityBeforeLifting", qualities);
                    _sim->setTarget(_dhand->getBase(), _home, nstate);
                    _tsim->reset(nstate);
                    _currentState = LIFTING;
                }
            }
            if( !isResting ){
                _restingTime = _sim->getTime();
            }

        } else {
            _restingTime = _sim->getTime();
        }
    }

    if(_currentState==LIFTING){
        // test if object has been lifted
        bool isLifted = true;
        Transform3D<> ct3d = Kinematics::worldTframe(_dhand->getBase()->getBodyFrame(), state);
        isLifted &= MetricUtil::dist2( ct3d.P(), _home.P() )<0.005;
        //isLifted &= ct3d.R().equal(_home.R(),0.01);
        //std::cout << MetricUtil::dist2( ct3d.P(), _home.P() ) << "<" << 0.001 << std::endl;
        // if its lifted then verify the object gripper transform
        if (isLifted) {
            GraspedObject gobj = getObjectContacts(state);
            _graspedQ = _hand->getQ(state);
            //getTarget()->getPropertyMap().set<Transform3D<> > ("GripperTObject", t3d);
            if( gobj.object == NULL ){
                std::cout << "No contacts!" << std::endl;
                _failed++;
                getTarget()->getPropertyMap().set<int> ("TestStatus", ObjectDropped);
                getTarget()->getPropertyMap().set<int> ("LiftStatus", ObjectDropped);
                getTarget()->getPropertyMap().set<Q>("QualityAfterLifting", Q::zero(2));
            } else {
                getTarget()->getPropertyMap().set<int> ("LiftStatus", Success);
                Q qualities = calcGraspQuality(state);
                getTarget()->getPropertyMap().set<Q>("QualityAfterLifting", qualities);

                //Transform3D<> t3d = Kinematics::frameTframe(_mbase, gobj.object->getBodyFrame(), state);

                // Test the success of lifting the object.
                // We need to look at the objects that are actually touching
                Body::Ptr object = gobj.object;
                Body::Ptr gripperBody = gobj.bodies[0];
                /*
                Transform3D<> wTp = Kinematics::worldTframe(_mbase->getParent(state), state);

                Transform3D<> objectBefore = gobj.object->getTransformW(_postLiftObjState);
                Transform3D<> objectNow = gobj.object->getTransformW(state);
                Vector3D<> objectMoveVector = objectNow.P() - objectBefore.P();

                Transform3D<> baseBefore = _dhand->getBase()->getTransformW(_postLiftObjState);
                Transform3D<> baseAfter = _dhand->getBase()->getTransformW(state);

                Vector3D<> liftVector = baseAfter.P() - baseBefore.P();
                */

                Transform3D<> tcpTo_before = Kinematics::frameTframe(_tcp, object->getBodyFrame(), _postLiftObjState);
                Transform3D<> tcpTo_after  = Kinematics::frameTframe(_tcp, object->getBodyFrame(), state);
                getTarget()->getPropertyMap().set<Transform3D<> > ("ObjectTtcpGrasp", inverse(tcpTo_before) );
                getTarget()->getPropertyMap().set<Transform3D<> > ("ObjectTtcpLift", inverse(tcpTo_after) );


                Transform3D<> oTg_before = Kinematics::frameTframe(object->getBodyFrame(), gripperBody->getBodyFrame(), _postLiftObjState);
                Transform3D<> oTg_after  = Kinematics::frameTframe(object->getBodyFrame(), gripperBody->getBodyFrame(), state);
                Vector3D<> slipVector = oTg_after.P() - oTg_before.P();
                // allow op to 2 cm slip else its a fault

                double slippage = slipVector.norm2();

                double liftResult;
                if(slippage <= 0.02)
                    liftResult = (0.02 - slippage)*50;
                else
                    liftResult = 0.0;
                std::cout << "Slippage: " << slippage <<" " << object->getName()<<" " << gripperBody->getName() << std::endl;
                std::cout << "LIFT RESULTS: " << liftResult << std::endl;
                getTarget()->getPropertyMap().set<double> ("LiftResult", liftResult);

                if (liftResult == 0.0) {
                    _failed++;
                    getTarget()->getPropertyMap().set<int> ("TestStatus", ObjectDropped);
                    std::cout << _sim->getTime() << " : " << "ObjectDropped" << std::endl;
                } else if (liftResult > 0.50) { // At most 1cm difference with hand lift
                    _success++;
                    getTarget()->getPropertyMap().set<int> ("TestStatus", Success);
                    std::cout << _sim->getTime() << " : " << "Success" << std::endl;
                } else {
                    _slipped++;
                    getTarget()->getPropertyMap().set<int> ("TestStatus", ObjectSlipped);
                    std::cout << _sim->getTime() << " : " << "ObjectSlipped" << std::endl;
                }

            }
            getTarget()->getPropertyMap().set<Q>("GripperConfigurationPost", _graspedQ);
            for(size_t i=0;i<_objects.size();i++){
                getTarget()->getPropertyMap().set<Transform3D<> >("GripperTObjectLift"+boost::lexical_cast<std::string>(i),
                                                                  _objects[i]->getTransformW(_restObjState));
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

            if( !hasNextTarget() ){
            //if(_nextTaskIndex>=(int)_targets->size()){
                // end we are done

                saveTasks(true);
                // save the result
                getRobWorkStudio()->postState(nstate);
                std::cout << "STOP" << std::endl;
                _stopped = true;

                return;
            }

            rwlibs::task::CartesianTarget::Ptr target = getNextTarget();

            if( target->getPropertyMap().get<int>("TestStatus",-1)>=0 ){
                // if test status is set then we allready processed this task.
                _skipped++;
                std::cout << "SKIPPING TARGET - allready processed!\n";
                continue;
            }

            nstate = _homeState;
            Transform3D<> wTp = Kinematics::worldTframe(_mbase->getParent(nstate), nstate);
            Transform3D<> start = inverse(wTp) * _wTe_n * target->get() * inverse(_bTe);

            _mbase->setTransform(start, nstate);
            // and calculate the home lifting position
            _home = _wTe_home * target->get() * inverse(_bTe);
            _approach = _wTe_n * target->get()* _approachDef * inverse(_bTe);
            //std::cout << "START: " << start << std::endl;
            //std::cout << "HOME : " << _home << std::endl;

            _hand->setQ(_openQ, nstate);
            for(size_t i=0;i<_objects.size();i++){
                Transform3D<> tobj = _objects[i]->getMovableFrame()->getTransform(_homeState);
                _objects[i]->getMovableFrame()->setTransform(tobj, nstate);
            }
            // set max force
            if(!_rhand.isNull()){
                Q forceLim = getTask()->getPropertyMap().get<Q>("TauMax",Q());
                if(forceLim.size()>0)
                    _rhand->setMotorForceLimits(forceLim);
            }

            colFreeSetup = !_collisionDetector->inCollision(nstate, NULL, true);

            //std::cout << "Current index: " << (_nextTaskIndex-1) << std::endl;
            if( !colFreeSetup ){
                target->getPropertyMap().set<int>("TestStatus", CollisionInitially);
                target->getPropertyMap().set<Q>("GripperConfiguration", _openQ);

                for(size_t i=0;i<_objects.size();i++){
                    getTarget()->getPropertyMap().set<Transform3D<> >("GripperTObject"+boost::lexical_cast<std::string>(i),
                                                                      _objects[i]->getTransformW(state));
                }

                //std::cout << "0.0 : InCollision " << std::endl;
                _collision++;
            }
            //std::cout << "1:" << _collision << _nrOfExperiments<< std::endl;

            _nrOfExperiments++;
        } while( !colFreeSetup );

        if( _nrOfExperiments > _lastSaveTaskIndex+40 ){
            saveTasks(true);
            _lastSaveTaskIndex = _nrOfExperiments;
        }
        // reset simulation
        _dhand->getBase()->reset(nstate);
        _tsim->reset(nstate);
        _sim->disableBodyControl();
        _sim->setTarget(_dhand->getBase(), _approach, nstate);
        _controller->setTargetPos(_openQ);
        _wallTimer.resetAndResume();
        _currentState = APPROACH;
        Transform3D<> t3d  = Kinematics::frameTframe(_tcp, _objects[0]->getBodyFrame(), nstate);
        getTarget()->getPropertyMap().set<Transform3D<> > ("ObjectTtcpTarget", inverse(t3d) );

        _restingTime = 0;
    }
}

void SimTaskPlugin::makeSimulator(){
    if(_sim != NULL)
        return;

    // we have a DWC create the simulator
    State state = getRobWorkStudio()->getState();
    _mbase->setTransform(Transform3D<>(Vector3D<>(100,100,100)), state);
    getRobWorkStudio()->setState(state);
    log().debug() << "Making physics engine";
    ODESimulator::Ptr engine = ownedPtr( new ODESimulator(_dwc));
    log().debug() << "Making simulator";
    _sim = ownedPtr( new DynamicSimulator(_dwc, engine ));
    log().debug() << "Initializing simulator";
    try {
        _sim->init(state);
    } catch(...){
        log().error() << "could not initialize simulator!\n";
    }
    log().debug() << "Creating Thread simulator";


    for(size_t i=0;i<_objects.size();i++){
        _bsensors.push_back( ownedPtr(new BodyContactSensor("SimTaskObjectSensor", _objects[i]->getBodyFrame())) );
        _sim->addSensor( _bsensors.back() , state);
    }

    _tsim = ownedPtr( new ThreadSimulator(_sim, state) );
    ThreadSimulator::StepCallback cb( boost::bind(&SimTaskPlugin::step, this, _1, _2) );

    _tsim->setStepCallBack( cb );
    _tsim->setTimeStep(0.01);


    rwsim::drawable::SimulatorDebugRender::Ptr debugRender = _sim->createDebugRender();
    if( debugRender == NULL ){
        Log::errorLog() << "The current simulator does not support debug rendering!" << std::endl;
        return;
    }

    debugRender->setDrawMask( 7 );
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
       outfile << "// Description: {target.pos(3), target.rpy(3), TestStatus(1), GripperConfiguration("<<_openQ.size()<<"), "
               "GripperTObject.pos, GripperTObject.rpy, ObjectTtcpBefore.pos, ObjectTtcpBefore.rpy, ObjectTtcpAfter.pos, ObjectTtcpAfter.rpy}\n";
       outfile << "// TestStatus enum { UnInitialized=0, Success=1, CollisionInitially=2, ObjectMissed=3, ObjectDropped=4, ObjectSlipped=5, TimeOut=6, SimulationFailure=7}\n";
       BOOST_FOREACH(CartesianTask::Ptr task, _taskQueue){
           std::vector<CartesianTarget::Ptr> targets = task->getTargets();
           outfile<<"{" << task->getId() << "}\n";
           BOOST_FOREACH(CartesianTarget::Ptr target, targets) {
              const Vector3D<>& pos = target->get().P();
              const RPY<> rpy(target->get().R());
              int status = target->getPropertyMap().get<int>("TestStatus", UnInitialized);
              outfile<<"{"<<pos(0)<<","<<pos(1)<<","<<pos(2)<<","<<rpy(0)<<","<<rpy(1)<<","<<rpy(2)<<","<<status<<",";

              Q distance = target->getPropertyMap().get<Q>("GripperConfiguration", Q::zero(_openQ.size()));
              for(size_t i=0;i<distance.size();i++)
                  outfile << distance[i] << ",";

              Transform3D<> t3d = target->getPropertyMap().get<Transform3D<> >("GripperTObject", Transform3D<>::identity());
              RPY<> rpyObj(t3d.R());
              outfile << t3d.P()[0] << "," << t3d.P()[1] << "," <<t3d.P()[2] << ","
                  << rpyObj(0) << "," << rpyObj(1) << "," <<rpyObj(2) << ",";

              t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpTarget", Transform3D<>::identity() );
              rpyObj = RPY<>(t3d.R());
              outfile << t3d.P()[0] << "," << t3d.P()[1] << "," <<t3d.P()[2] << ","
                  << rpyObj(0) << "," << rpyObj(1) << "," <<rpyObj(2)<< ",";

              t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpApproach", Transform3D<>::identity() );
              rpyObj = RPY<>(t3d.R());
              outfile << t3d.P()[0] << "," << t3d.P()[1] << "," <<t3d.P()[2] << ","
                  << rpyObj(0) << "," << rpyObj(1) << "," <<rpyObj(2)<< ",";

              t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpGrasp", Transform3D<>::identity() );
              rpyObj = RPY<>(t3d.R());
              outfile << t3d.P()[0] << "," << t3d.P()[1] << "," <<t3d.P()[2] << ","
                  << rpyObj(0) << "," << rpyObj(1) << "," <<rpyObj(2) << ",";

              t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpLift", Transform3D<>::identity() );
              rpyObj = RPY<>(t3d.R());
              outfile << t3d.P()[0] << "," << t3d.P()[1] << "," <<t3d.P()[2] << ","
                  << rpyObj(0) << "," << rpyObj(1) << "," <<rpyObj(2) << "}"<< "\n";

           }
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
        saver.save(_roottask, taskFile );

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
            log().error() << "Could not load dynamic workcell from propertymap!!" << std::endl;
            return;
        }
        //std::cout << "dwc" << dwc->getWorkCell()->getName() << std::endl;
        _dwc = dwc;
    } else if( event=="ExecuteSimulationTask" ){

        //_roottask = getRobWorkStudio()->getPropertyMap().get<rwlibs::task::CartesianTask::Ptr>("SimulationTask", NULL);

        //if(_tasks!=NULL)
        //    startSimulation();
    }
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(SimTaskPlugin);
#endif
