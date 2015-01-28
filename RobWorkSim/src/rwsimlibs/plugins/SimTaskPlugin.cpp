#include "SimTaskPlugin.hpp"

#include <rwsim/simulator/GraspTaskSimulator.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rws/RobWorkStudio.hpp>

#include <rw/graspplanning/GWSMeasure3D.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwsim/util/SurfacePoseSampler.hpp>

#include <rwlibs/task/GraspTask.hpp>

#include <boost/lexical_cast.hpp>
#include <QPushButton>
#include <QInputDialog>

#include <fstream>
#include <iostream>

USE_ROBWORK_NAMESPACE
using namespace robwork;

USE_ROBWORKSIM_NAMESPACE
using namespace robworksim;

using namespace rws;
using namespace rwlibs::proximitystrategies;
using namespace rwlibs::simulation;
using namespace rwlibs::task;

SimTaskPlugin::SimTaskPlugin():
    RobWorkStudioPlugin("SimTaskPluginUI", QIcon(":/simtaskplugin/pa_icon.png")),
    _nrOfTargetsToGen(1000),
    _imgRecordPostfix(0)
{
    setupUi(this);

    // now connect stuff from the ui component
    connect(_loadTaskBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_saveResultBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_startBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_stopBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_showTaskSpinBox    ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );
    connect(_delaySpin    ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );
    connect(_genTasksBox    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    _timer = new QTimer( this );
    _timer->setInterval( 100 );

    connect( _timer, SIGNAL(timeout()), this, SLOT(btnPressed()) );

    _typeComboBox->addItem("SCUP");
    _typeComboBox->addItem("PG70");
    _typeComboBox->addItem("PG70_SMALL");
    _typeComboBox->addItem("SDH_BALL");
    _typeComboBox->addItem("SDH_PAR");
    _typeComboBox->addItem("SDH_PAR1");
    _typeComboBox->addItem("SDH_PAR2");
    _typeComboBox->addItem("SDH_PAR1_TABLE");
    _typeComboBox->addItem("SDH_PAR2_TABLE");

    _typeComboBox->addItem("SDH_CYL");
    _typeComboBox->addItem("GS20");
    _typeComboBox->addItem("GS20_WIDE");

    _outputFormatBox->addItem("RWTask");
    _outputFormatBox->addItem("UIBK");
    _outputFormatBox->addItem("TXT");


    _startBtn->setEnabled(false);
    _stopBtn->setEnabled(false);
    _loadTaskBtn->setEnabled(true);
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
    //_log->info() << "Current log..." << std::endl;
    //_log = getRobWorkInstance()->getLogPtr();
    //_log->info() << "NEW log..." << std::endl;
    //Log::setLog( _log );
}

void SimTaskPlugin::startSimulation() {
    _imgRecordPostfix = 0;
    if(_graspSim==NULL){
        QMessageBox::information(this, "SimTaskPlugin", "Grasp simulator has not been created yet!");
        return;
    }
    try {
        _graspSim->startSimulation( _initState );
    } catch(...){
        return;
    }


    if( _debugRender == NULL ){
        _debugRender = _graspSim->getSimulator()->getSimulator()->createDebugRender();
        if(_debugRender==NULL)
            Log::errorLog() << "The current simulator does not support debug rendering!" << std::endl;
    }


    _debugRender->setDrawMask( SimulatorDebugRender::DRAW_CONTACT_NORMAL | SimulatorDebugRender::DRAW_FRICTION_CONE | SimulatorDebugRender::DRAW_BODY_FORCES );
    rwlibs::opengl::Drawable *debugDrawable = new rwlibs::opengl::Drawable( _debugRender, "DebugRender" );
    getRobWorkStudio()->getWorkCellScene()->addDrawable(debugDrawable, _dwc->getWorkcell()->getWorldFrame());
    _timer->start();
}

void SimTaskPlugin::open(WorkCell* workcell)
{
    if(workcell==NULL || _dwc==NULL)
        return;

    Math::seed( TimerUtil::currentTimeUs() );

    _wc = workcell;
    _graspSim = ownedPtr( new GraspTaskSimulator(_dwc) );

    _objectComboBox->clear();
    std::vector<RigidBody::Ptr> bodies = _dwc->findBodies<RigidBody>();
    BOOST_FOREACH(RigidBody::Ptr body, bodies){
        _objectComboBox->addItem(body->getName().c_str());
    }

    bool successOnly = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").has("SuccessOnly");
    _onlySuccessBox->setChecked(successOnly);
    std::string taskFormat = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("OutputFormat","RWTask");
    int idx = _outputFormatBox->findText(taskFormat.c_str());
    if(idx>=0)
        _outputFormatBox->setCurrentIndex(idx);

    _loadTaskBtn->setEnabled(true);

    std::string gripper = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("Gripper","GS20");
    idx = _typeComboBox->findText(gripper.c_str());
    if(idx<0){
        RW_WARN("The gripper \""<< gripper << "\" does not exist! Setting default gripper");
        idx = 0;
    }
    _typeComboBox->setCurrentIndex(idx);

    _initState = getRobWorkStudio()->getState();
    //_genTasksBox enabled and nr of task
    //std::string objectName = _objectComboBox->currentText().toStdString();
    _nrOfTargetsToGen = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<int>("GenerateTasksSize",1000);

    //std::string seedTargets = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<string>("Seed","");
    //if(boost::filesystem::exists(seedTargets)){
    //    _seedTargets = GraspTask::load( seedTargets );
    //}
    if(getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").has("Auto")){
        if(getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").has("GenerateTasks") ){
            _genTasksBox->setChecked(true);
            setCurrentTask( generateTasks(_nrOfTargetsToGen) );
        } else {
            // load tasks from file
            _genTasksBox->setChecked(false);
            loadTasks(true);
        }
        //start simulation
        startSimulation();
    }
}

void SimTaskPlugin::close() {
    // destroy simulation and stuff
    _timer->stop();
    if(_graspSim!=NULL){
        _graspSim->pauseSimulation();
    }
    _graspSim = NULL;
    _wc = NULL;

    _loadTaskBtn->setEnabled(false);
    _saveResultBtn->setEnabled(false);
    _startBtn->setEnabled(false);
    _stopBtn->setEnabled(false);

}

void SimTaskPlugin::btnPressed() {
    QObject *obj = sender();
    if(obj==_loadTaskBtn){
        // load tasks from a filename
        loadTasks(false);
    } else if (obj == _genTasksBox){
        if( !_genTasksBox->isChecked() ){
            _loadTaskBtn->setEnabled(false);
            _startBtn->setEnabled(true);
            _stopBtn->setEnabled(true);
        } else {
            _loadTaskBtn->setEnabled(true);
            _startBtn->setEnabled(false);
            _stopBtn->setEnabled(false);
        }
    } else if(obj==_saveResultBtn){
        saveTasks(false);
    } else if(obj==_startBtn){
        if( !_graspSim->isRunning() ){
            _mergedResult = ownedPtr( new GraspTask() );
            if( _genTasksBox->isChecked() ){
                // generate an initial task list and set it
                GraspTask::Ptr tasks = generateTasks(_nrOfTargetsToGen);
                setCurrentTask(tasks);
            }
            startSimulation();
        } else {
            _graspSim->resumeSimulation();
            _saveResultBtn->setEnabled(false);
        }
    } else if(obj==_stopBtn){
        _saveResultBtn->setEnabled(true);
        _graspSim->pauseSimulation();

    } else if( obj == _delaySpin ){
        int delay = _delaySpin->value();
        if(_graspSim->getSimulator()!=NULL)
            _graspSim->getSimulator()->setRealTimeScale(delay);
    } else if(obj==_showTaskSpinBox){
/*
        std::vector<std::pair<GraspSubTask*,GraspTarget*> > _targets = _graspSim->getTasks()->getAllTargets();
        if(_targets==NULL){
            log().error() << "No targets\n";
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
        Transform3D<> wTe_home = task->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
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
        BOOST_FOREACH(RigidBody *object, _objects){
            Transform3D<> objHome = object->getMovableFrame()->getTransform( _homeState );
            object->getMovableFrame()->setTransform(objHome, state );
        }
        //std::cout << "eTb: " << inverse(_bTe) << std::endl;
        //std::cout << "TARGET: " << (*_targets)[idxTmp]->get() << std::endl;
        //std::cout << "HOME: " << _home << std::endl;

        getRobWorkStudio()->setState(state);
*/
    } else if(obj==_timer){
        if(_graspSim->getSimulator()==NULL)
            return;

        if(_recordBox->isChecked()){
            std::stringstream sstr;

            sstr << "record_" << _wc->getName()<< "_" << _imgRecordPostfix << ".png";
            _imgRecordPostfix++;
            getRobWorkStudio()->saveViewGL(sstr.str().c_str());
        }

        State state = _graspSim->getSimulator()->getState();
        // update the RobWorkStudio state
        //State state = _tsim->getState();
        getRobWorkStudio()->setState(state);
        _wallTimeLbl->setText( _wallTotalTimer.toString("hh:mm:ss").c_str() );
        _simTimeLbl->setText( Timer(_simTime*1000).toString("hh:mm:ss:zzz").c_str() );
        std::vector<int> stat = _graspSim->getStat();
        //int nrgrasps = _failed + _success + _slipped + _collision + _timeout + _simfailed;
        int nrgrasps = _graspSim->getNrTargetsDone();
        double avgGraspTime = (_wallTotalTimer.getTime()/(nrgrasps))*1000;
        _timePerGraspLbl->setText( Timer((int)avgGraspTime).toString("ss:zzz").c_str() );
        _timeToFinishLbl->setText( Timer((int)(avgGraspTime* (_graspSim->getNrTargetsDone()-nrgrasps))).toString("hh:mm").c_str() );

        std::vector<int> stats = _graspSim->getStat();


        log().info() << stats[0] << ":" << stats[1] << ":" << stats[2] << ":" << stats[3] << ":" << stats[4] << ":" << stats[5] << ":" << stats[6]
                << stats[7] << ":" << stats[8] << ":" << stats[9] << ":" << stats[10] << ":" << stats[11] << ":" << stats[12] << ":" << stats[13] << std::endl;

        if( _graspSim->isFinished() && !_genTasksBox->isChecked()){

            _saveResultBtn->setEnabled(true);
            //
            _timer->stop();
            if(getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").has("Auto")){
                _timer->stop();
                saveTasks(true);
                getRobWorkStudio()->postExit();
                return;
            } else {

            }
        } else if(_graspSim->isFinished() && _genTasksBox->isChecked()){

            if(getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").has("Auto")){
                _timer->stop();
                saveTasks(true);
                getRobWorkStudio()->postExit();
                return;
            } else {

            }

            // read back result and add it to the merged result
            //log().info() << "mergedSize: " << _mergedResult->getTargets().size() << std::endl;

            if( _continuesBox->isChecked() ){
                // copy grasp results into merged
                GraspTask::Ptr task = _graspSim->getResult();

                if(_onlySuccessBox->isChecked()){
                    std::vector<GraspResult::TestStatus> includefilter;
                    includefilter.push_back( GraspResult::Success );
                    includefilter.push_back( GraspResult::ObjectSlipped );
                    task->filterTasks( includefilter );
                }

                if(_mergedResult->getSubTasks().size()==0){
                    _mergedResult = task;
                } else {
                    BOOST_FOREACH(GraspSubTask &stask, task->getSubTasks()){
                        _mergedResult->getSubTasks().push_back(stask);
                    }
                }
            }
            // save tasks
            log().info() << "SAVE MERGED TASKS\n";
            //std::cout << "SAVE MERGED TASKS\n";
            GraspTask::saveRWTask(_mergedResult, "mergedResultTmp.task.xml");

            //std::cout << "mergedSize: " << _mergedResult->getTargets().size() << std::endl;
            //log().info() << "mergedSize: " << _mergedResult->getTargets().size() << std::endl;

            // generate new targets
            // generaAutote an initial task list and set it
            if( _continuesBox->isChecked() ){
                GraspTask::Ptr tasks = generateTasks(_nrOfTargetsToGen);
                setCurrentTask(tasks);
                _graspSim->startSimulation( _initState );
            } else {
                _timer->stop();
            }

        } else {
            // update progress
            _progressBar->setValue( _graspSim->getNrTargetsDone() );
        }
    }
}

void SimTaskPlugin::updateConfig(){
    // START

}

rw::common::PropertyMap& SimTaskPlugin::settings(){
    return getRobWorkStudio()->getPropertyMap().get<rw::common::PropertyMap>("RobWorkStudioSettings");
}


void SimTaskPlugin::loadTasks(bool automatic){
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
            "RW TASK files ( *.task.xml )"
            "\nAll supported ( *.xml )"
            "\n All ( *.* )",
            &selectedFilter);

        taskFile = filename.toStdString();
    }

    if(taskFile=="")
        return;
    log().info() << "Loading tasks: ";
    log().info() << "\t-Filename: " << taskFile;


    GraspTask::Ptr task;
    try {
        task = GraspTask::load( taskFile );
    } catch (const Exception&) {
        QMessageBox::information(this, "SimTaskPlugin", "Unable to load tasks from file");
        return;
    }

    setCurrentTask( task );
}



GraspTask::Ptr SimTaskPlugin::generateTasks(int nrTasks){

    _dwc->getBodies();
    int nrOfCollisions = 0;
    std::string objectName = _objectComboBox->currentText().toStdString();
    std::string type = _typeComboBox->currentText().toStdString();
    std::string gripperName = type;
    std::string gripperTcpName;
    GraspTask::Ptr gtask = ownedPtr(new GraspTask());
    Body* body = _dwc->findBody(objectName).get();
    if(body==NULL){
        RW_THROW("OBJECT DOES NOT EXIST: " << objectName);
    }
    std::vector<Geometry::Ptr> geoms = body->getGeometry();
    SurfacePoseSampler ssurf( geoms );
    ssurf.setRandomRotationEnabled(false);

    // these should be the object transformation
    Vector3D<> pos(0, 0, 0);
    Rotation3D<> rot(1, 0, 0,
                     0, 1, 0,
                     0, 0, 1);

    // check if type is actually the gripper name, else query the user
    if( _wc->findDevice(type)==NULL ){
        QStringList items, tcpitems;
        BOOST_FOREACH(Device::Ptr dev, _wc->getDevices() ){
            std::string name = dev->getName();
            items << tr( name.c_str() );
        }
       bool ok;
       QString item = QInputDialog::getItem(this, tr("Please specify which gripper you use:"),
                                            tr("Gripper:"), items, 0, false, &ok);
       if ( !ok ){
           return NULL;
       }
       gripperName = item.toStdString();
       Device::Ptr dev = _wc->findDevice(gripperName);
       std::vector<Frame*> frames = Kinematics::findAllFrames( dev->getBase() );
       // now get the tcp to use
       BOOST_FOREACH(Frame *f, frames ){
           std::string name = f->getName();
           tcpitems << tr( name.c_str() );
       }

       item = QInputDialog::getItem(this, tr("Please specify which TCP frame you use:"),
                                            tr("TCP Frame:"), tcpitems, 0, false, &ok);

       if ( !ok ){
           return NULL;
       }

       gripperTcpName  = item.toStdString();


    } else {
        gripperName = type;
    }





    // first set up the configuration
    Vector3D<> d(0,0,-0.02);
    Transform3D<> wTe_n(pos, rot);
    Transform3D<> wTe_home(pos+inverse(rot)*d, rot);
    Q openQ(1,0.0);
    Q closeQ(1,1.0);
    if( type=="PG70" ){
        openQ  = Q(1, 0.034);
        closeQ = Q(1, 0.0);

        if(gripperTcpName.empty()){
            gtask->setTCPID("TCPPG70");
            gripperName = type;
        }
    } else if( type== "PG70_SMALL"){
        openQ  = Q(1, 0.01);
        closeQ = Q(1, 0.0);
        if(gripperTcpName.empty()){
            gripperName = type;
            gtask->setTCPID("TCPPG70");
        }
    } else if( type== "GS20"){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
        if(gripperTcpName.empty()){
            gripperName = type;
            gtask->setTCPID("TCPGS20");
        }
    } else if( type== "GS20_WIDE"){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
        if(gripperTcpName.empty()){
            gripperName = "GS20";
            gtask->setTCPID("TCPGS20");
        }
    } else if( type== "SDH_PAR"){
        openQ =  Q(7,-1.571,-1.571,1.571, -1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        gtask->setTCPID("SchunkHand.SDHTCP");
        gripperName = "SchunkHand";
    } else if( type== "SDH_PAR1"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        gtask->setTCPID("SchunkHand.SDHTCP1");
        gripperName = "SchunkHand";
    } else if( type== "SDH_PAR2"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.1, 0.1, -0.1, 0.1);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        gtask->setTCPID("SchunkHand.SDHTCP1");
        gripperName = "SchunkHand";

    } else if( type== "SDH_PAR1_TABLE"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        gtask->setTCPID("SchunkHand.SDHTCP1");
        gripperName = "SchunkHand";
    } else if( type== "SDH_PAR2_TABLE"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.1, 0.1, -0.1, 0.1);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        gtask->setTCPID("SchunkHand.SDHTCP1");
        gripperName = "SchunkHand";

    } else if( type== "SDH_BALL"){
        openQ = Q(7,-1.048, 0.174, 1.047 ,-1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 1.047,0.0, 0.349,0.0, 0.349);
        gtask->setTCPID("SchunkHand.SDHTCP");
        gripperName = "SchunkHand";
    } else if( type== "SDH_CYL"){
        openQ = Q(7, -1.048, 0.174, 0.0, -1.048, 0.174,-1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 0.0, 0.0, 0.349, 0.0, 0.349);
        gtask->setTCPID("SchunkHand.SDHTCP");
        gripperName = "SchunkHand";
    } else if( type== "SCUP"){
        openQ  = Q(1, 0.0);
        closeQ = Q(1, 1.0);
        gtask->setTCPID("EndFrame");
        _graspSim->setAlwaysResting(true);
        gripperName = type;
    } else {
        RW_THROW(" The gripper type is wrong! please specify a valid grippertype: (PG70, SCUP, SDH_PAR, SDH_CYL, SDH_BALL)");
    }
    //wTe_n = Transform3D<>::identity();
    //wTe_home = Transform3D<>::identity();
    gtask->setGripperID(gripperName);
    gtask->setGraspControllerID("GraspController");
    //rtask->getPropertyMap().set<std::string >("Object", objectName);

    //CartesianTask::Ptr tasks = ownedPtr( new CartesianTask());
    gtask->getSubTasks().resize(1);
    GraspSubTask &subtask = gtask->getSubTasks()[0];

    //rtask->addTask(tasks);

    subtask.offset = wTe_n;
    if( type== "SCUP"){
        subtask.approach = Transform3D<>(Vector3D<>(0,0,0.04));
        subtask.retract = Transform3D<>(Vector3D<>(0,0,0.0));
    } else if( gripperName=="GS20"){
        subtask.approach = Transform3D<>(Vector3D<>(0,0,0.04));
        subtask.retract = Transform3D<>(Vector3D<>(0,0,0.1));
    } else {
        subtask.approach = Transform3D<>(Vector3D<>(0,0,0.0));
        subtask.retract = Transform3D<>(Vector3D<>(0,0,0.10));
    }

    if( gripperName=="SchunkHand"){
        Q tau = Q(7, 2.0, 2.0, 10.0, 2.0, 2.0, 2.0, 2.0);
        // depending on the value of joint 2 adjust the forces
        double alpha = openQ(2);
        if(alpha<45*Deg2Rad){
            tau(3) = tau(0)/(2*cos(alpha));
            tau(5) = tau(0)/(2*cos(alpha));
        } else {
            tau(0) = std::max( 2*cos(alpha)*tau(3), 0.2);
        }
        subtask.tauMax = tau;
    }

    subtask.openQ = openQ;
    subtask.closeQ = closeQ;

    if( type=="GS20" || type=="GS20_WIDE"){
        ssurf.setBoundsD(-0.02,0.02);
    } else if( type=="SCUP" ){
        ssurf.setBoundsD(-0.02,0.005);
    } else {
        ssurf.setBoundsD(-0.04,0.04);
    }

    // now we choose a random number in the total area
    State state = _initState;
    Transform3D<> wTo = rw::kinematics::Kinematics::worldTframe(body->getBodyFrame(), state);
    if( (type== "SDH_PAR1_TABLE") || (type== "SDH_PAR2_TABLE")  ){
        ssurf.setZAxisDirectionEnabled(true);
        ssurf.setZAxisDirection(Vector3D<>(0,0,1));
        std::cout << "SDH_PAR1_TABLE" << std::endl;
        Device::Ptr dev = _wc->findDevice(gripperName);
        std::string tcp = gtask->getTCPID();
        Frame *tcpframe = _wc->findFrame(tcp);
        MovableFrame *base = dynamic_cast<MovableFrame*>( dev->getBase() );
        dev->setQ(closeQ,state);
        RW_ASSERT(dev);
        Transform3D<> baseTtcp = Kinematics::frameTframe(base,tcpframe, state);
        for(int i=0; i<nrTasks; i++){
            // we only sample
            bool incollision;
            Transform3D<> target;

            do {
                CollisionDetector::QueryResult result;
                target = wTo*ssurf.sample();
                Transform3D<> wTbase = target*inverse(baseTtcp);
                Transform3D<> wTparent = Kinematics::worldTframe(base->getParent(state), state);
                dev->setQ(openQ,state);
                base->setTransform(inverse(wTparent)*wTbase, state);
                incollision = getRobWorkStudio()->getCollisionDetector()->inCollision(state, &result);
                if(incollision)
                    nrOfCollisions++;
                /*
                dev->setQ(closeQ,state);
                if( getRobWorkStudio()->getCollisionDetector()->inCollision(state, &result) ){
                    incollision = false;
                    BOOST_FOREACH(kinematics::FramePair pair, result.collidingFrames){
                        //test if it collides with anything else than the object, if it does then its a failure
                        std::cout << pair.first->getName() << " -- "<< pair.second->getName() << std::endl;
                        if( (pair.first!=body->getBodyFrame()) && (pair.second!=body->getBodyFrame()) ){
                            incollision = true;
                            break;
                        }
                    }
                }
                */
            } while(incollision);
            std::cout << "Found: " << i+1 << "    \r";
            std::cout.flush();

            subtask.targets.push_back( GraspTarget( target ) );
        }
        log().info() << "Collisions: " << nrOfCollisions << "\n";
    } else {

        for(int i=0; i<nrTasks; i++){
            Transform3D<> target;

            target = wTo*ssurf.sample();
            subtask.targets.push_back( GraspTarget( target ) );
        }
    }
    return gtask;
}

void SimTaskPlugin::setCurrentTask(GraspTask::Ptr task){
    try {
        _graspSim->load(task);
    } catch(const Exception& exp){
        QMessageBox::information(this, "SimTaskPlugin", exp.what());
        return;
    }

    //_totalNrOfExperiments = _graspSim->getNrTargets();
    _showTaskSpinBox->setRange(0, (int)_graspSim->getNrTargets() );
    _nrTaskSpinBox->setRange( 0, (int)_graspSim->getNrTargets() );
    _nrTaskSpinBox->setValue( (int)_graspSim->getNrTargets()-1 );
    log().info() << "LOAD TASKS DONE, nr of tasks: " << _graspSim->getNrTargets();
    _progressBar->setMaximum( (int)_graspSim->getNrTargets());
    _startBtn->setEnabled(true);
    _stopBtn->setEnabled(true);

}

void SimTaskPlugin::stateChangedListener(const State& state) {

}

void SimTaskPlugin::exportMathematica(const std::string& filename) {
/*
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
      */
}


void SimTaskPlugin::saveTasks(bool automatic){

    std::string filename = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("TaskTestOutFile", "test.task.xml");
    std::string taskFile = filename;
    std::cout << "SAVING TASK: " << filename << std::endl;
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

    GraspTask::Ptr result;
    if( _mergeResultBox->isChecked() ){
        if(_mergedResult!=NULL ){
            result = _mergedResult;
        } else {
            result = _graspSim->getResult();
        }
    } else {
        result = _graspSim->getResult();
    }

    if(_onlySuccessBox->isChecked()){
        // remove all failing grasp targets
        std::vector<GraspResult::TestStatus> includefilter;
        includefilter.push_back( GraspResult::Success );
        includefilter.push_back( GraspResult::ObjectSlipped );
        result->filterTasks( includefilter );
    }

    try {
        if(_outputFormatBox->currentText()=="RWTask"){
            GraspTask::saveRWTask(result, taskFile);
        } else if(_outputFormatBox->currentText()=="UIBK"){
            GraspTask::saveUIBK(result, taskFile);
        } else if(_outputFormatBox->currentText()=="TXT"){
            GraspTask::saveText(result, taskFile);
        }
    } catch (const Exception&) {
        QMessageBox::information(this, "Task Execution Widget", "Unable to save tasks");
    }
    //std::stringstream sstr;
    //sstr << taskFile << ".m.txt";
    //exportMathematica(sstr.str());

}


void SimTaskPlugin::genericEventListener(const std::string& event){
    if( event=="DynamicWorkCellLoaded" ){
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
Q_EXPORT_PLUGIN(SimTaskPlugin);
#endif
