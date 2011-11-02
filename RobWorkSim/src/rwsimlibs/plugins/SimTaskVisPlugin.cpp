#include "SimTaskVisPlugin.hpp"

#include <rwsim/rwsim.hpp>
//#include <rw/rw.hpp>
#include <QPushButton>
#include <RobWorkStudio.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwsim/drawable/SimulatorDebugRender.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rw/graspplanning/Grasp3D.hpp>
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/graspplanning/GWSMeasure3D.hpp>
#include <rwlibs/opengl/DrawableUtil.hpp>
#include <rw/common/macros.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>
#include <rwsim/simulator/GraspTask.hpp>

//#define PHOENIX_LIMIT 15

USE_ROBWORK_NAMESPACE
USE_ROBWORKSIM_NAMESPACE
using namespace robwork;
using namespace robworksim;
using namespace rws;
using namespace rwlibs::simulation;


namespace {

    class RenderTargets : public Render
    {

    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderTargets> Ptr;

        struct Target {
            GLfloat color[4];
            Transform3D<> trans;
            double scale;
            bool enabled;
        };

        RenderTargets():_size(-0.02), _zoffset(0.0){
            rw::geometry::Box box(_size/2,_size/8,_size/2);
            mesh = box.getTriMesh();
        };

        /* Functions inherited from Render */
        /**
         * @copydoc Render::draw
         */
        void draw(const DrawableNode::RenderInfo& info, DrawType type, double alpha) const {
            glPushMatrix();
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_LIGHTING);
            BOOST_FOREACH(Target target, _targets){
                //if(!target.enabled)
                //    continue;
                const Vector3D<> &zoffset = _zoffset*( target.trans.R()*Vector3D<>::z() );
                const Vector3D<> &p = target.trans.P()+zoffset;
                const Vector3D<> &pn = p+target.trans.R()*Vector3D<>::z()*_size + zoffset;

                glBegin(GL_LINES);
                glColor3fv(target.color);
                glVertex3f(p[0],p[1],p[2]);
                glVertex3f(pn[0],pn[1],pn[2]);
                glEnd();

                glBegin(GL_TRIANGLES);
                for(size_t i=0;i<mesh->size();i++){
                    const Triangle<> tri = mesh->getTriangle(i);
                    rwlibs::opengl::DrawableUtil::drawGLVertex(pn+target.trans.R()*tri[0]);
                    rwlibs::opengl::DrawableUtil::drawGLVertex(pn+target.trans.R()*tri[1]);
                    rwlibs::opengl::DrawableUtil::drawGLVertex(pn+target.trans.R()*tri[2]);
                }
                glEnd();
            }
            glEnable(GL_LIGHTING);
            glPopMatrix();
        }

        void setTargets(std::vector<Target>& targets){
            _targets = targets;
        }

        void setZOffset(float offset){
            _zoffset = offset;
        }
    private:
        float _size;
        float _zoffset;
        std::vector<Target> _targets;
        rw::geometry::TriMesh::Ptr mesh;

    };


    class FrameComboBox {
    public:
        FrameComboBox(QComboBox* box):_box(box){}


    private:
        QComboBox* _box;
    };

}

SimTaskVisPlugin::SimTaskVisPlugin():
    RobWorkStudioPlugin("SimTaskVisPlugin", QIcon(":/simtaskvisplugin/pa_icon.png"))
{
    setupUi(this);

    // now connect stuff from the ui component
    connect(_loadTaskBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_updateBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_zoffsetSpin    ,SIGNAL(valueChanged(double)), this, SLOT(btnPressed()) );
    _updateBtn->setEnabled(false);

    //_propertyView = new PropertyViewEditor(this);
    //_propertyView->setPropertyMap(&_config);

    //QVBoxLayout *vbox = new QVBoxLayout;
    //vbox->addWidget(_propertyView);

    QPalette plt;
    plt.setColor(QPalette::Text, Qt::red);
    plt.setColor(QPalette::WindowText, Qt::red);


    _collisionsBox->setPalette(plt);

}

SimTaskVisPlugin::~SimTaskVisPlugin()
{
}

void SimTaskVisPlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(
            boost::bind(&SimTaskVisPlugin::stateChangedListener, this, _1), this);

    getRobWorkStudio()->genericEvent().add(
          boost::bind(&SimTaskVisPlugin::genericEventListener, this, _1), this);

    Log::setLog( _log );
}

void SimTaskVisPlugin::open(WorkCell* workcell)
{
    // add drawable to the workcell
    if(workcell==NULL)
        return;

    _render = ownedPtr( new RenderTargets() );
    getRobWorkStudio()->getWorkCellScene()->addRender("pointRender", _render, workcell->getWorldFrame() );

    BOOST_FOREACH(MovableFrame* object, workcell->findFrames<MovableFrame>() ){
        _frameSelectBox->addItem(object->getName().c_str());
    }

}

void SimTaskVisPlugin::close() {

}

void SimTaskVisPlugin::btnPressed() {
    QObject *obj = sender();
    if(obj==_loadTaskBtn){
        // load tasks from a filename
        loadTasks(false);

        _ymtargets.clear();
        setTask(0);
        while(hasNextTarget()){
            CartesianTarget::Ptr target = getNextTarget();
            _ymtargets.push_back( std::make_pair( getTask(), target ) );
        }
        _updateBtn->setEnabled(true);
    } else if(obj==_zoffsetSpin){

        ((RenderTargets*)_render.get())->setZOffset( _zoffsetSpin->value() );

    } else if(obj==_updateBtn){
        // we need the frame that this should be drawn with respect too
        //std::cout << "update" << std::endl;
        std::string fname = _frameSelectBox->currentText().toStdString();
        MovableFrame* selframe = getRobWorkStudio()->getWorkcell()->findFrame<MovableFrame>(fname);
        if(selframe==NULL)
            return;
        double maxQual=-1000000, minQual=10000000;

        // for stats
        int droppedStat=0, missedStat=0, successStat=0, slippedStat=0, collisionStat=0, otherStat=0;

        Transform3D<> wTo = Kinematics::worldTframe(selframe, getRobWorkStudio()->getState() );
        int nrToShow = _nrOfTargetSpin->value();
        std::vector<RenderTargets::Target> rtargets;
        for(int i=0;i<nrToShow; i++){
        // generate target list and add it to the render

            int idx = i;
            if(nrToShow<_ymtargets.size()){
                idx = Math::ranI(0, _ymtargets.size());
            } else {
                if(idx>=_ymtargets.size())
                    break;
            }

            CartesianTask::Ptr task = _ymtargets[idx].first;
            CartesianTarget::Ptr target = _ymtargets[idx].second;

            Transform3D<> wTe_n = task->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
            Transform3D<> wTe_home = task->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());


            RenderTargets::Target rt;
            rt.color[0] = 0.0;
            rt.color[1] = 0.0;
            rt.color[2] = 0.0;
            rt.color[3] = 0.5;
            int testStatus = target->getPropertyMap().get<int>("TestStatus", -1);
            int qIdx = _qualitySpin->value();
            Q quality = target->getPropertyMap().get<Q>("QualityAfterLifting", Q(1, 0.0));
            if(quality.size()==0){
                //continue;
                quality = Q(1, 0.0);
            }

            if(qIdx>=(int)quality.size()){
                qIdx=quality.size()-1;
                continue;
            }

            if( maxQual<quality(qIdx) )
                maxQual = quality(qIdx);
            if( minQual>quality(qIdx) )
                minQual = quality(qIdx);
            if(_showQuality->isChecked()){
                if(quality(qIdx)<_fromThresSpin->value())
                    continue;
                if(quality(qIdx)>_toThresSpin->value())
                    continue;
            }

            if(testStatus==-1){
            	if(!_untestedBox->isChecked() )
            		continue;

            } else if(testStatus==ObjectDropped){

            	if(!_droppedBox->isChecked() )
            		continue;
            	droppedStat++;
            	rt.color[0] = 1.0;
            } else if(testStatus==ObjectMissed){
            	if(!_missedBox->isChecked() )
            		continue;
            	missedStat++;
                rt.color[0] = 0.5;
            } else if(testStatus==Success){
            	if(!_successBox->isChecked() )
            		continue;
            	successStat++;
                rt.color[1] = 1.0;
            } else if(testStatus==ObjectSlipped){
            	if(!_slippedBox->isChecked() )
            		continue;
            	slippedStat++;
                rt.color[0] = 0.0;
                rt.color[1] = 1.0;
                rt.color[2] = 1.0;
            } else if(testStatus==CollisionInitially){
            	if(!_collisionsBox->isChecked() )
            		continue;
            	collisionStat++;
                rt.color[0] = 1.0;
                rt.color[2] = 1.0;
            } else {
            	if(!_otherBox->isChecked() )
            		continue;
            	otherStat++;
                rt.color[0] = 0.5;
                rt.color[1] = 0.5;
                rt.color[2] = 0.5;
            }
            if( _showQuality->isChecked() ){

                rt.scale = quality(qIdx);
            } else {
                rt.scale = 1.0;
            }
            if(_showTargetBox->isChecked()){
                //std::cout << wTe_n << std::endl;
                bool has = target->getPropertyMap().has("ObjectTtcpApproach");
                if(has){
                    rt.trans = wTo * target->getPropertyMap().get<Transform3D<> > ("ObjectTtcpApproach");
                } else {
                    rt.trans = wTe_n*target->get();
                }
                rtargets.push_back(rt);
            }

            if(_showEndGraspTargetBox->isChecked()){
                // this should be relative to the object frame
                bool has = target->getPropertyMap().has("ObjectTtcpGrasp");
                //bool has = target->getPropertyMap().has("ObjectTtcpApproach");

                if(has){
                    rt.trans = wTo * target->getPropertyMap().get<Transform3D<> > ("ObjectTtcpGrasp");
                    rtargets.push_back(rt);
                }
            }

            if(_showEndLiftTargetBox->isChecked()){
                //bool has = target->getPropertyMap().has("ObjectTtcpLift");
                bool has = target->getPropertyMap().has("GripperTObjectLift0");

                if(has){
                    rt.trans = wTo * inverse( target->getPropertyMap().get<Transform3D<> > ("GripperTObjectLift0") );
                    rtargets.push_back(rt);
                }
            }

        }
        //std::cout << "NR TARGETS:: " << rtargets.size() << std::endl;

        // if quality should be shown then we start by calculating the offset and scale
        double offset = 0;
        double scale = 1;
        if(_showQuality->isChecked()){
            //offset = -minQual;
            //scale = 1.0/(maxQual-minQual);

            offset = -_fromThresSpin->value();
            scale = 1.0/(_toThresSpin->value()-_fromThresSpin->value());

            BOOST_FOREACH(RenderTargets::Target& t, rtargets){

                if( t.color[0]+t.color[1]+t.color[2]<0.00001 ){
                    t.color[0] = (1-(t.scale+offset)*scale);
                    t.color[1] = (1-(t.scale+offset)*scale);
                    t.color[2] = (1-(t.scale+offset)*scale);
                } else {
                    t.color[0] = t.color[0] * (t.scale+offset)*scale;
                    t.color[1] = t.color[1] * (t.scale+offset)*scale;
                    t.color[2] = t.color[2] * (t.scale+offset)*scale;
                }
                //std::cout << t.color[0] << " (" << t.scale << "+" << offset<<")*" << scale << std::endl;
            }
        }
        _fromThresSpin->setMinimum(minQual);
        _fromThresSpin->setMaximum(maxQual);
        _toThresSpin->setMinimum(minQual);
        _toThresSpin->setMaximum(maxQual);

        log().info() << "Total:" << successStat+slippedStat+otherStat+collisionStat+droppedStat+missedStat
                     << " Succes:"<<successStat
                     << " Slipped:"<<slippedStat
                     << " Dropped:"<<droppedStat
                     << " Missed:"<<missedStat
                     << " Collision:"<<collisionStat
                     << " Other:"<<otherStat << "\n";


        //std::cout << "setting : " << rtargets.size() << std::endl;
        ((RenderTargets*)_render.get())->setTargets(rtargets);
        getRobWorkStudio()->postUpdateAndRepaint();
    }

}


void SimTaskVisPlugin::loadConfig(bool automatic){
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
            QMessageBox::information(this, "SimTaskVisPlugin", "SimTaskConfig could not be loaded!");
        }
    }
    updateConfig();

    // for some reason this crashes sometimes...
    //_propertyView->setPropertyMap( &_config );
}

void SimTaskVisPlugin::updateConfig(){
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
        // find the first rigid body if no ObjectName specified in the config file
        std::vector<RigidBody*> rbodies = _dwc->findBodies<RigidBody>();
        if(rbodies.size()>0)
            objName = rbodies[0]->getName();
        _config.add<std::string>("ObjectName","Name of the object that is to be grasped", objName);
    }
    objName = _config.get<std::string>("ObjectName");
    RigidBody *object = _dwc->findBody<RigidBody>(objName);
    if(object!=NULL)
        _objects.push_back(object);

    // TWOOBJ: check if ObjectName2 exists
    std::string objName2;
    if( _config.has("ObjectName2") && _config.get<std::string>("ObjectName2")!="" ){
      objName2 = _config.get<std::string>("ObjectName2");
      object = _dwc->findBody<RigidBody>(objName2);
      if(object!=NULL)
          _objects.push_back(object);
    }

    if( !_config.has("CalculateWrenchQuality") ){
        _config.add<bool>("CalculateWrenchQuality","Set true if the quality of the grasp should be calculated", true);
    }
    _calcWrenchQuality = _config.get<bool>("CalculateWrenchQuality");

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
        _configured = true;
    } else {
        //std::cout << (_hand!=NULL) <<" && " << (_tcp!=NULL) <<" && " << (_objects.size()>0) <<" && " << (_mbase!=NULL) <<" && " << (_dhand!=NULL) << std::endl;
        _loadTaskBtn->setEnabled(false);
        _configured = false;
    }
}

void SimTaskVisPlugin::saveConfig(){

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
        QMessageBox::information(this, "SimTaskVisPlugin", "SimTaskConfig could not be loadet!");
    }
}

rw::common::PropertyMap& SimTaskVisPlugin::settings(){
    return getRobWorkStudio()->getPropertyMap().get<rw::common::PropertyMap>("RobWorkStudioSettings");
}





void SimTaskVisPlugin::loadTasks(bool automatic){
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

    std::string firstelem = IOUtil::getFirstXMLElement(taskFile);
    std::cout << "FIRST ELEMENT: " << firstelem << std::endl;

    log().info() << "Loading tasks: ";
    log().info() << "\t-Filename: " << taskFile;
    rwlibs::task::CartesianTask::Ptr task;

    try {
        GraspTask::Ptr gtask = GraspTask::load(taskFile);
        task = gtask->getRootTask();
    } catch (const Exception& exp) {
        QMessageBox::information(this, "SimTaskVisPlugin", "Unable to load tasks from file");
        return;
    }
    /*
    try {
        XMLTaskLoader loader;
        loader.load( taskFile );
        task = loader.getCartesianTask();
    } catch (const Exception& exp) {
        QMessageBox::information(this, "SimTaskVisPlugin", "Unable to load tasks from file");
        return;
    }
    */

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
        BOOST_FOREACH(rwlibs::task::CartesianTask::Ptr subtask, tmpTask->getTasks()){
            tmpStack.push(subtask);
        }
    }

    _totalNrOfExperiments = nrOfTargets;
    log().info() << "LOAD TASKS DONE, nr of tasks: " << nrOfTargets;
    setTask(0);

}

void SimTaskVisPlugin::stateChangedListener(const State& state) {

}



bool SimTaskVisPlugin::hasNextTarget(){
    if(_targets == NULL || _nextTargetIndex >= (int)_targets->size()){
        if(_currentTaskIndex+1 >= (int)_taskQueue.size()){
            return false;
        }
    }
    return true;
}

rwlibs::task::CartesianTask::Ptr SimTaskVisPlugin::getTask(){
    return _currenttask;
}

void SimTaskVisPlugin::setTask(int i){
    if(i<0 || i>= (int)_taskQueue.size())
        return;

    _currentTaskIndex = i;
    _currenttask = _taskQueue[i];
    _targets = &_currenttask->getTargets();
    _nextTargetIndex = 0;

    _wTe_n = _currenttask->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
    _wTe_home = _currenttask->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
    Vector3D<> approach = _currenttask->getPropertyMap().get<Vector3D<> >("Approach", Vector3D<>(0,0,0));
    _approachDef = Transform3D<>( approach, Rotation3D<>::identity());
    _openQ = _currenttask->getPropertyMap().get<Q>("OpenQ", _openQ);
    _closeQ = _currenttask->getPropertyMap().get<Q>("CloseQ", _closeQ);

    //log().info() << "openQ" << _openQ << "\n";
    //log().info() << "closeQ" << _closeQ << "\n";
}

rwlibs::task::CartesianTarget::Ptr SimTaskVisPlugin::getNextTarget(){
    // were we iterate over all tasks and their targets

    if(_targets == NULL || _nextTargetIndex>= (int)_targets->size()){
        // get the next task and reinitialize _targets and _currentTaskIndex
        if(_currentTaskIndex+1 >= (int)_taskQueue.size()){
            return NULL; // there is no more tasks
        }
        setTask(_currentTaskIndex+1);
    }
    _currentTargetIndex = _nextTargetIndex;
    _nextTargetIndex++;
    return (*_targets)[ _currentTargetIndex ];
}

rwlibs::task::CartesianTarget::Ptr SimTaskVisPlugin::getTarget(){
    return (*_targets)[ _currentTargetIndex ];
}


void SimTaskVisPlugin::genericEventListener(const std::string& event){
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

    }
}

Q_EXPORT_PLUGIN(SimTaskVisPlugin);
