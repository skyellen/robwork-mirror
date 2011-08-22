#include "SimTaskPlugin.hpp"

#include <rwsim/simulator/GraspTaskSimulator.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rws/RobWorkStudio.hpp>

#include <rw/graspplanning/GWSMeasure3D.hpp>
#include <rwlibs/opengl/Drawable.hpp>

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
    RobWorkStudioPlugin("SimTaskPluginUI", QIcon(":/pa_icon.png"))
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


    _timer = new QTimer( this );
    _timer->setInterval( 100 );

    connect( _timer, SIGNAL(timeout()), this, SLOT(btnPressed()) );

    _propertyView = new PropertyViewEditor(this);
    _propertyView->setPropertyMap(&_config);

    QVBoxLayout *vbox = new QVBoxLayout;
    vbox->addWidget(_propertyView);
    _configGroupBox->setLayout(vbox);

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

    Log::setLog( _log );
}

void SimTaskPlugin::startSimulation() {
    if(_graspSim==NULL){
        QMessageBox::information(this, "SimTaskPlugin", "Grasp simulator has not been created yet!");
        return;
    }

    _graspSim->startSimulation( getRobWorkStudio()->getState() );
    //RW_WARN("1");
    rwsim::drawable::SimulatorDebugRender::Ptr debugRender = _graspSim->getSimulator()->getSimulator()->createDebugRender();
    if( debugRender == NULL ){
        Log::errorLog() << "The current simulator does not support debug rendering!" << std::endl;
        return;
    }
    //RW_WARN("1");
    debugRender->setDrawMask( 7 );
    rwlibs::opengl::Drawable *debugDrawable = new rwlibs::opengl::Drawable( debugRender, "DebugRender" );
    getRobWorkStudio()->getWorkCellScene()->addDrawable(debugDrawable, _dwc->getWorkcell()->getWorldFrame());
    //RW_WARN("1");
    _timer->start();
    //RW_WARN("1");
}

void SimTaskPlugin::open(WorkCell* workcell)
{
    if(workcell==NULL || _dwc==NULL)
        return;

    _wc = workcell;
    _graspSim = ownedPtr( new GraspTaskSimulator(_dwc) );

    _loadTaskBtn->setEnabled(true);

    _propertyView->update();

    if(getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").has("Auto")){
        // load tasks from file
        loadTasks(true);

        //start simulation
        startSimulation();
    }
}

void SimTaskPlugin::close() {
    // destroy simulation and stuff
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
    } else if(obj==_saveResultBtn){
        saveTasks(false);
    } else if(obj==_startBtn){
        startSimulation();
    } else if(obj==_updateConfigBtn){
        _propertyView->update();
        updateConfig();
    } else if(obj==_stopBtn){
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
        //RW_WARN("1");
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
        //RW_WARN("1");
        if( _graspSim->isFinished() ){
            _saveResultBtn->setEnabled(true);
            //RW_WARN("1");
            _timer->stop();
            if(getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").has("Auto")){
                std::cout << "AUTO CLOSE ACTIVATED, exiting RobWorkStudio" << std::endl;
                //saveTasks(true);
                getRobWorkStudio()->postExit();
            } else {
                std::cout << "AUTO CLOSE DEACTIVATED" << std::endl;
            }
        } else {
            //RW_WARN("1");
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
    try {
        _graspSim->load(task);
    } catch(const Exception& exp){
        QMessageBox::information(this, "SimTaskPlugin", exp.what().c_str());
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

    try {
        XMLTaskSaver saver;
        saver.save(_graspSim->getResult(), taskFile );

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

Q_EXPORT_PLUGIN(SimTaskPlugin);
