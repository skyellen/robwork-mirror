#include "SimTaskPlugin.hpp"

#include <rwsim/simulator/GraspTaskSimulator.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rws/RobWorkStudio.hpp>

#include <rw/graspplanning/GWSMeasure3D.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwsim/util/SurfacePoseSampler.hpp>

#include <rwsim/simulator/GraspTask.hpp>

#include <boost/lexical_cast.hpp>
#include <QPushButton>
#include <fstream>
#include <iostream>


const int NR_OF_QUALITY_MEASURES = 3;

USE_ROBWORK_NAMESPACE
using namespace robwork;

USE_ROBWORKSIM_NAMESPACE
using namespace robworksim;

using namespace rws;

using namespace rwlibs::simulation;

SimTaskPlugin::SimTaskPlugin():
    RobWorkStudioPlugin("SimTaskPluginUI", QIcon(":/simtaskplugin/pa_icon.png")),
    _nrOfTargetsToGen(1000)
{
    setupUi(this);

    // now connect stuff from the ui component
    connect(_loadTaskBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_saveResultBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_startBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_stopBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_updateConfigBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_showTaskSpinBox    ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );
    connect(_delaySpin    ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );
    connect(_genTasksBox    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    _timer = new QTimer( this );
    _timer->setInterval( 100 );

    connect( _timer, SIGNAL(timeout()), this, SLOT(btnPressed()) );

    _propertyView = new PropertyViewEditor(this);
    _propertyView->setPropertyMap(&_config);

    QVBoxLayout *vbox = new QVBoxLayout;
    vbox->addWidget(_propertyView);
    _configGroupBox->setLayout(vbox);

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

    _debugRender->setDrawMask( 15 );
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
    std::vector<RigidBody*> bodies = _dwc->findBodies<RigidBody>();
    BOOST_FOREACH(RigidBody* body, bodies){
        _objectComboBox->addItem(body->getName().c_str());
    }

    std::string taskFormat = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("OutputFormat","RWTask");
    int idx = _outputFormatBox->findText(taskFormat.c_str());
    if(idx>=0)
        _outputFormatBox->setCurrentIndex(idx);

    _loadTaskBtn->setEnabled(true);

    _propertyView->update();

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
            _mergedResult = ownedPtr( new CartesianTask() );
            if( _genTasksBox->isChecked() ){
                // generate an initial task list and set it
                rwlibs::task::CartesianTask::Ptr tasks = generateTasks(_nrOfTargetsToGen);
                setCurrentTask(tasks);
            }
            startSimulation();
        } else {
            _graspSim->resumeSimulation();
            _saveResultBtn->setEnabled(false);
        }
    } else if(obj==_updateConfigBtn){
        _propertyView->update();
        updateConfig();
    } else if(obj==_stopBtn){
        _saveResultBtn->setEnabled(true);
        _graspSim->pauseSimulation();

    } else if( obj == _delaySpin ){
        int delay = _delaySpin->value();
        if(_graspSim->getSimulator()!=NULL)
            _graspSim->getSimulator()->setPeriodMs(delay);
    } else if(obj==_showTaskSpinBox){

        /*
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
                CartesianTask::Ptr task = _graspSim->getResult();

                if(_onlySuccessBox->isChecked()){
                    std::vector<CartesianTarget::Ptr> stargets;
                    BOOST_FOREACH(rwlibs::task::CartesianTarget::Ptr target, task->getTargets()){
                        int status = target->getPropertyMap().get<int> ("TestStatus",-1);
                        if(status==GraspTaskSimulator::Success || status==GraspTaskSimulator::ObjectSlipped){
                            stargets.push_back(target);
                        }
                    }
                    if(stargets.size()>0){
                        task->getTargets() = stargets;
                    }
                }
                _mergedResult->addTask(task);
            }

            log().info() << "mergedSize: " << _mergedResult->getTargets().size() << std::endl;
            // generate new targets
            // generaAutote an initial task list and set it
            if( _continuesBox->isChecked() ){
                rwlibs::task::CartesianTask::Ptr tasks = generateTasks(_nrOfTargetsToGen);
                setCurrentTask(tasks);
                _graspSim->startSimulation( _initState );
            } else {
                _timer->stop();
            }
            // save tasks
            log().info() << "SAVE MERGED TASKS\n";
            GraspTaskSimulator::save("mergedResultTmp.task.xml", _mergedResult, GraspTaskSimulator::TaskFormat );

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
    rwlibs::task::CartesianTask::Ptr task;
    try {
        XMLTaskLoader loader;
        loader.load( taskFile );
        task = loader.getCartesianTask();
    } catch (const Exception& exp) {
        QMessageBox::information(this, "SimTaskPlugin", "Unable to load tasks from file");
        return;
    }

    setCurrentTask( task );
}



rwlibs::task::CartesianTask::Ptr SimTaskPlugin::generateTasks(int nrTasks){

    _dwc->getBodies();

    std::string objectName = _objectComboBox->currentText().toStdString();
    std::string type = _typeComboBox->currentText().toStdString();
    rwlibs::task::CartesianTask::Ptr tasks = ownedPtr(new rwlibs::task::CartesianTask());
    Body* body = _dwc->findBody(objectName);
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


    // first set up the configuration
    Vector3D<> d(0,0,-0.02);
    Transform3D<> wTe_n(pos, rot);
    Transform3D<> wTe_home(pos+inverse(rot)*d, rot);
    Q openQ(1,0.0);
    Q closeQ(1,1.0);
    if( type=="PG70" ){
        openQ  = Q(1, 0.034);
        closeQ = Q(1, 0.0);
        tasks->getPropertyMap().set<std::string>("TCP","TCPPG70");
    } else if( type== "PG70_SMALL"){
        openQ  = Q(1, 0.01);
        closeQ = Q(1, 0.0);
        tasks->getPropertyMap().set<std::string>("TCP","TCPPG70");
    } else if( type== "GS20"){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
        tasks->getPropertyMap().set<std::string>("TCP","TCPGS20");
    } else if( type== "GS20_WIDE"){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
        type = "GS20";
        tasks->getPropertyMap().set<std::string>("TCP","TCPGS20");
    } else if( type== "SDH_PAR"){
        openQ =  Q(7,-1.571,-1.571,1.571, -1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        tasks->getPropertyMap().set<std::string>("TCP","SchunkHand.SDHTCP");
        type = "SchunkHand";
    } else if( type== "SDH_PAR1"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        tasks->getPropertyMap().set<std::string>("TCP","SchunkHand.SDHTCP1");
        type = "SchunkHand";
    } else if( type== "SDH_PAR2"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.1, 0.1, -0.1, 0.1);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        tasks->getPropertyMap().set<std::string>("TCP","SchunkHand.SDHTCP1");
        type = "SchunkHand";

    } else if( type== "SDH_PAR1_TABLE"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        tasks->getPropertyMap().set<std::string>("TCP","SchunkHand.SDHTCP1");
        type = "SchunkHand";
    } else if( type== "SDH_PAR2_TABLE"){
        openQ =  Q(7,-1.571,-1.571,1.571, -0.1, 0.1, -0.1, 0.1);
        closeQ = Q(7,-1.571,-1.571,1.571,  0.0  , 0.419,  0.0,   0.419);
        tasks->getPropertyMap().set<std::string>("TCP","SchunkHand.SDHTCP1");
        type = "SchunkHand";

    } else if( type== "SDH_BALL"){
        openQ = Q(7,-1.048, 0.174, 1.047 ,-1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 1.047,0.0, 0.349,0.0, 0.349);
        tasks->getPropertyMap().set<std::string>("TCP","SchunkHand.SDHTCP");
        type = "SchunkHand";
    } else if( type== "SDH_CYL"){
        openQ = Q(7, -1.048, 0.174, 0.0, -1.048, 0.174,-1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 0.0, 0.0, 0.349, 0.0, 0.349);
        tasks->getPropertyMap().set<std::string>("TCP","SchunkHand.SDHTCP");
        type = "SchunkHand";
    } else if( type== "SCUP"){
        openQ  = Q(1, 0.0);
        closeQ = Q(1, 1.0);
        tasks->getPropertyMap().set<std::string>("TCP","EndFrame");
    } else {
        RW_THROW(" The gripper type is wrong! please specify a valid grippertype: (PG70, SCUP, SDH_PAR, SDH_CYL, SDH_BALL)");
    }
    //wTe_n = Transform3D<>::identity();
    //wTe_home = Transform3D<>::identity();
    tasks->getPropertyMap().set<std::string >("Gripper", type);
    tasks->getPropertyMap().set<std::string >("Object", objectName);
    tasks->getPropertyMap().set<Transform3D<> >("Offset", wTe_n);
    tasks->getPropertyMap().set<Transform3D<> >("Home", wTe_home);
    if( type== "SCUP"){
        tasks->getPropertyMap().set<Transform3D<> >("Approach", Transform3D<>(Vector3D<>(0,0,0.04)) );
        tasks->getPropertyMap().set<Transform3D<> >("Retract", Transform3D<>(Vector3D<>(0,0,-0.04)));
    } else {
        tasks->getPropertyMap().set<Transform3D<> >("Approach", Transform3D<>(Vector3D<>(0,0,0.0)) );
        tasks->getPropertyMap().set<Transform3D<> >("Retract", Transform3D<>(Vector3D<>(0,0,0.10)));
    }
    tasks->getPropertyMap().set<Q>("OpenQ", openQ);
    tasks->getPropertyMap().set<Q>("CloseQ", closeQ);

    if( type=="GS20" || type=="GS20_WIDE"){
        ssurf.setBoundsD(-0.03,0.03);
    } else if( type=="SCUP" ){
        ssurf.setBoundsD(-0.05,0.05);
    } else {
        ssurf.setBoundsD(-0.04,0.04);
    }

    // now we choose a random number in the total area
    State state = _initState;
    Transform3D<> wTo = rw::kinematics::Kinematics::worldTframe(body->getBodyFrame(), state);
    if( (type== "SDH_PAR1_TABLE") || (type== "SDH_PAR2_TABLE")  ){
        std::cout << "SDH_PAR1_TABLE" << std::endl;
        Device::Ptr dev = _wc->findDevice(type);
        std::string tcp = tasks->getPropertyMap().get<std::string>("TCP");
        Frame *tcpframe = _wc->findFrame(tcp);
        MovableFrame *base = dynamic_cast<MovableFrame*>( dev->getBase() );
        dev->setQ(closeQ,state);
        RW_ASSERT(dev);
        Transform3D<> baseTtcp = Kinematics::frameTframe(base,tcpframe, state);
        for(int i=0; i<nrTasks; i++){
            std::cout << "task" << std::endl;
            // we only sample
            bool incollision;
            Transform3D<> target;
            CollisionDetector::QueryResult result;
            do {
                target = wTo*ssurf.sample();
                Transform3D<> wTbase = target*inverse(baseTtcp);
                base->setTransform(wTbase, state);
                incollision = true;
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
            } while(incollision);


            CartesianTarget::Ptr ctarget = ownedPtr( new CartesianTarget(target) );
            tasks->addTarget( ctarget );
        }
    } else {

        for(int i=0; i<nrTasks; i++){
            Transform3D<> target = wTo*ssurf.sample();
            CartesianTarget::Ptr ctarget = ownedPtr( new CartesianTarget(target) );
            tasks->addTarget( ctarget );
        }
    }
    return tasks;
}

void SimTaskPlugin::setCurrentTask(rwlibs::task::CartesianTask::Ptr task){
    try {
        _graspSim->load(task);
    } catch(const Exception& exp){
        QMessageBox::information(this, "SimTaskPlugin", exp.what());
        return;
    }

    //_totalNrOfExperiments = _graspSim->getNrTargets();
    _showTaskSpinBox->setRange(0, _graspSim->getNrTargets() );
    _nrTaskSpinBox->setRange( 0, _graspSim->getNrTargets() );
    _nrTaskSpinBox->setValue( _graspSim->getNrTargets()-1 );
    log().info() << "LOAD TASKS DONE, nr of tasks: " << _graspSim->getNrTargets();
    _progressBar->setMaximum( _graspSim->getNrTargets()-1);
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

    rwlibs::task::CartesianTask::Ptr result;

    if(_mergedResult!=NULL){
        result = _mergedResult;
    } else {
        result = _graspSim->getResult();
    }

    if(_onlySuccessBox->isChecked()){
        // remove all failing grasp targets
        std::vector<rwlibs::task::CartesianTarget::Ptr> filteredtargets;
        BOOST_FOREACH(rwlibs::task::CartesianTarget::Ptr target, result->getTargets()){

            int status = target->getPropertyMap().get<int> ("TestStatus",-1);
            if(status==GraspTaskSimulator::Success || status==GraspTaskSimulator::ObjectSlipped){
                filteredtargets.push_back(target);
            }
        }
        result->getTargets() = filteredtargets;
    }

    rwlibs::task::CartesianTask::Ptr grasptask = ownedPtr( new rwlibs::task::CartesianTask() );
    grasptask->addTask(result);

    GraspTask gtask(grasptask);

    try {
        if(_outputFormatBox->currentText()=="RWTask"){
            GraspTask::saveRWTask(&gtask, taskFile);
        } else if(_outputFormatBox->currentText()=="UIBK"){
            GraspTask::saveUIBK(&gtask, taskFile);
        } else if(_outputFormatBox->currentText()=="TXT"){
            GraspTask::saveText(&gtask, taskFile);
        }
    } catch (const Exception& exp) {
        QMessageBox::information(this, "Task Execution Widget", "Unable to save tasks");
    }
    //std::stringstream sstr;
    //sstr << taskFile << ".m.txt";
    //exportMathematica(sstr.str());

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

Q_EXPORT_PLUGIN(SimTaskPlugin);
