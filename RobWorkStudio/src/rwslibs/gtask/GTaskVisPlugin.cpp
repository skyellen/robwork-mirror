#include "GTaskVisPlugin.hpp"

#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rw/geometry/Box.hpp>
#include <rw/models/WorkCell.hpp>
#include <rwlibs/opengl/DrawableUtil.hpp>
#include <rwlibs/task/GraspTask.hpp>

#include <rws/RobWorkStudio.hpp>

#include <QFileDialog>
#include <QPushButton>
#include <QMessageBox>
#include <QTimer>

#include <boost/bind.hpp>

#include <sstream>

//#define PHOENIX_LIMIT 15

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;
using namespace rwlibs::task;

namespace {

    class RenderTargets : public Render
    {

    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderTargets> Ptr;

        struct Target {
            Target():scale(1.0),enabled(true),ctask(NULL),endTarget(false){};
            GLfloat color[4];
            Transform3D<> trans;
            double scale;
            bool enabled;
            GraspSubTask *ctask;
            GraspTarget ctarget;
            bool endTarget; // true if trans is the final grasp target (grasp or lift)
        };

        RenderTargets():_size(-0.02f), _zoffset(0.0f){
            rw::geometry::Box box(_size/8,_size/2,_size/2);
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
                const Vector3D<> &p = target.trans.P();
                const Vector3D<> &pn = p+target.trans.R()*Vector3D<>::z()*_size + zoffset;
                const Vector3D<> &pc = p+(target.trans.R()*Vector3D<>::x()*_size/8);
                glBegin(GL_LINES);
                glColor3fv(target.color);
                glVertex3f(p[0],p[1],p[2]);
                glVertex3f(pn[0],pn[1],pn[2]);
                glColor3f(0.0f,0.0f,0.0f);
                glVertex3f(p[0],p[1],p[2]);
                glVertex3f(pc[0],pc[1],pc[2]);
                glEnd();

                glBegin(GL_TRIANGLES);
                glColor3fv(target.color);
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

        const std::vector<Target>& getTargets(){ return _targets; }

        void setZOffset(float offset){
            _zoffset = offset;
        }
    private:
        float _size;
        float _zoffset;
        std::vector<Target> _targets;
        rw::geometry::TriMesh::Ptr mesh;

    };
}

GTaskVisPlugin::GTaskVisPlugin():
    RobWorkStudioPlugin("GTaskVisPlugin", QIcon(":/simtaskvisplugin/pa_icon.png"))
{
    setupUi(this);
	

    // now connect stuff from the ui component
    connect(_loadTaskBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_updateBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_zoffsetSpin    ,SIGNAL(valueChanged(double)), this, SLOT(btnPressed()) );

    connect(_graspSelectSpin    ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );
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

GTaskVisPlugin::~GTaskVisPlugin()
{
}

void GTaskVisPlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(
            boost::bind(&GTaskVisPlugin::stateChangedListener, this, _1), this);

    getRobWorkStudio()->genericEvent().add(
          boost::bind(&GTaskVisPlugin::genericEventListener, this, _1), this);

    getRobWorkStudio()->genericAnyEvent().add(
          boost::bind(&GTaskVisPlugin::genericAnyEventListener, this, _1, _2), this);

    Log::setLog( _log );
}

void GTaskVisPlugin::open(WorkCell* workcell)
{
    // add drawable to the workcell
    if(workcell==NULL)
        return;
    _wc = workcell;
    _render = ownedPtr( new RenderTargets() );
    _targetDrawable = getRobWorkStudio()->getWorkCellScene()->addRender("pointRender", _render, workcell->getWorldFrame() );


    BOOST_FOREACH(MovableFrame* object, workcell->findFrames<MovableFrame>() ){
        _frameSelectBox->addItem(object->getName().c_str());
    }

    BOOST_FOREACH(Frame* object, workcell->findFrames<Frame>() ){
        _tcpSelectBox->addItem(object->getName().c_str());
    }

    BOOST_FOREACH(MovableFrame* object, workcell->findFrames<MovableFrame>() ){
        _baseSelectBox->addItem(object->getName().c_str());
    }

    BOOST_FOREACH(Device::Ptr dev, workcell->getDevices() ){
        _deviceSelectBox->addItem(dev->getName().c_str());
    }

}

void GTaskVisPlugin::close() {

}

void GTaskVisPlugin::btnPressed() {
    QObject *obj = sender();
    if(obj==_loadTaskBtn){
        // load tasks from a filename
        loadTasks(false);
    } else if(obj==_zoffsetSpin){
        ((RenderTargets*)_render.get())->setZOffset( _zoffsetSpin->value() );
    } else if(obj==_graspSelectSpin){
        int index = _graspSelectSpin->value();
        selectGrasp(index);
    } else if(obj==_updateBtn){
        updateVis();
    }
}

void GTaskVisPlugin::updateVis(){
    // we need the frame that this should be drawn with respect too
    //std::cout << "update" << std::endl;
    std::string fname = _frameSelectBox->currentText().toStdString();
    MovableFrame* selframe = getRobWorkStudio()->getWorkcell()->findFrame<MovableFrame>(fname);
    std::string tcpname = _tcpSelectBox->currentText().toStdString();

    Frame* tcpframe = getRobWorkStudio()->getWorkcell()->findFrame(tcpname);
    if(tcpframe==NULL)
        return;
    if(selframe==NULL)
        return;
    double maxQual=-1000000, minQual=10000000;

    // for stats
    int droppedStat=0, missedStat=0, successStat=0, slippedStat=0, collisionStat=0, collisionEnvStat=0, collisionObjStat=0, otherStat=0;

    bool showInGripperFrame = _invertBox->isChecked();

    Transform3D<> wTo = Kinematics::worldTframe(selframe, getRobWorkStudio()->getState() );
    // set the frame of the drawable
    getRobWorkStudio()->getWorkCellScene()->removeDrawable(_targetDrawable);
    _targetDrawable = getRobWorkStudio()->getWorkCellScene()->addRender("pointRender", _render, selframe );
    _targetDrawable->setTransform( inverse( wTo ) );


    Transform3D<> wTtcp = Kinematics::worldTframe(tcpframe, getRobWorkStudio()->getState() );

    int nrToShow = _nrOfTargetSpin->value();
    std::vector<RenderTargets::Target> rtargets;
    for(int i=0;i<nrToShow; i++){
    // generate target list and add it to the render

        int idx = i;
        if(nrToShow<(int)_ymtargets.size()){
            idx = Math::ranI(0, (int)_ymtargets.size());
        } else {
            if(idx>=(int)_ymtargets.size())
                break;
        }

        GraspSubTask *task = _ymtargets[idx].first;
        GraspTarget &target = *_ymtargets[idx].second;

        //Transform3D<> wTe_n = task->offset;

        if(target.result==NULL)
            continue;

        RenderTargets::Target rt;
        rt.ctask = task;
        rt.ctarget = target;
        rt.color[0] = 0.0;
        rt.color[1] = 0.0;
        rt.color[2] = 0.0;
        rt.color[3] = 0.5;
        int testStatus = target.result->testStatus;
        int qIdx = _qualitySpin->value();
        Q quality = target.result->qualityAfterLifting;
        if(quality.size()==0){
            //continue;
            quality = Q(1, 0.0);
        }

        if(qIdx>=(int)quality.size()){
            qIdx=(int)quality.size()-1;
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

        } else if(testStatus==GraspResult::ObjectDropped){
            if(!_droppedBox->isChecked() )
                continue;
            droppedStat++;
            rt.color[0] = 1.0;
        } else if(testStatus==GraspResult::ObjectMissed){
            if(!_missedBox->isChecked() )
                continue;
            missedStat++;
            rt.color[0] = 0.5;
        } else if(testStatus==GraspResult::Success){
            if(!_successBox->isChecked() )
                continue;
            successStat++;
            rt.color[1] = 1.0;
        } else if(testStatus==GraspResult::ObjectSlipped){
            if(!_slippedBox->isChecked() )
                continue;
            slippedStat++;
            rt.color[0] = 0.0;
            rt.color[1] = 1.0;
            rt.color[2] = 0.5;
        } else if(testStatus==GraspResult::CollisionInitially ||
                  testStatus==GraspResult::CollisionEnvironmentInitially ||
                  testStatus==GraspResult::CollisionObjectInitially){
            if(!_collisionsBox->isChecked() )
                continue;
            if(testStatus==GraspResult::CollisionEnvironmentInitially)
                collisionEnvStat++;
            else if(testStatus==GraspResult::CollisionObjectInitially)
                collisionObjStat++;
            else
                collisionStat++;
            rt.color[0] = 1.0;
            rt.color[1] = 0.0;
            rt.color[2] = 1.0;
        //} else if(testStatus==GraspTask::SimulationFailure){

        } else {
            if(!_otherBox->isChecked() )
                continue;
            otherStat++;
            if(testStatus==GraspResult::SimulationFailure){
                rt.color[0] = 1;
                rt.color[1] = 1;
                rt.color[2] = 1;
            } else {
                rt.color[0] = 0.5;
                rt.color[1] = 0.5;
                rt.color[2] = 0.5;
            }
        }
        if( _showQuality->isChecked() ){
            rt.scale = quality(qIdx);
        } else {
            rt.scale = 1.0;
        }
        if(_showTargetBox->isChecked()){
            //std::cout << wTe_n << std::endl;
            if(!showInGripperFrame){
                //rt.trans = wTo * target.result->objectTtcpTarget;
                //if(target.result->objectTtcpTarget.P().norm2()<0.000001)
                    rt.trans = wTo * target.pose;
            } else {
                rt.trans = wTtcp * inverse(target.result->objectTtcpTarget);
                if(target.result->objectTtcpTarget.P().norm2()<0.000001)
                    rt.trans = wTtcp * inverse( target.pose );
            }
            rtargets.push_back(rt);
        }

        if(_showEndGraspTargetBox->isChecked()){
            // this should be relative to the object frame
            if(!showInGripperFrame){
                rt.trans = wTo * target.result->objectTtcpGrasp;
            } else {
                rt.trans = wTtcp * inverse( target.result->objectTtcpGrasp );
            }
            rt.endTarget = true;
            rtargets.push_back(rt);
        }

        if(_showEndLiftTargetBox->isChecked()){
            if(!showInGripperFrame){
                rt.trans = wTo * target.result->objectTtcpLift;
            } else {
                rt.trans = wTtcp * inverse( target.result->objectTtcpLift );
            }
            rt.endTarget = true;
            rtargets.push_back(rt);
        }

    }
    //std::cout << "NR TARGETS:: " << rtargets.size() << std::endl;
    _graspSelectSpin->setMinimum(0);
    _graspSelectSpin->setMaximum((int)rtargets.size()-1);

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
                // a value between 0 and 1, with 1 being high quality and 0 being low
                double sval = (t.scale+offset)*scale;
                if( _lowIsHigh->isChecked() ){
                    sval = 1-sval;
                }
                Vector3D<float> red(1,0,0),yellow(1,1,0),green(0,1,0), color;
                if(sval<0.5){
                    // red to yellow
                    color = red+normalize(yellow-red)*sval*2;
                } else {
                    // sval is
                    // yellow to green
                    color = yellow+normalize(green-yellow)*(sval-0.5)*2;
                }
                t.color[0] = color[0];
                t.color[1] = color[1];
                t.color[2] = color[2];
            }
            //std::cout << t.color[0] << " (" << t.scale << "+" << offset<<")*" << scale << std::endl;
        }
    }

    _fromThresSpin->setMinimum(minQual - (maxQual-minQual));
    _fromThresSpin->setMaximum(maxQual + (maxQual-minQual));
    _toThresSpin->setMinimum(minQual - (maxQual-minQual));
    _toThresSpin->setMaximum(maxQual + (maxQual-minQual));

    if( !_costumThreshold->isChecked() ){
        _fromThresSpin->setValue(minQual);
        _toThresSpin->setValue(maxQual);
    }

    log().info() << "Total:" << successStat+slippedStat+otherStat+collisionStat+droppedStat+missedStat
                 << " Succes:"<<successStat
                 << " Slipped:"<<slippedStat
                 << " Dropped:"<<droppedStat
                 << " Missed:"<<missedStat
                 << " Collision:"<<collisionStat
                 << " CollisionObj:"<<collisionObjStat
                 << " CollisionEnv:"<<collisionEnvStat
                 << " Other:"<<otherStat << "\n";


    //std::cout << "setting : " << rtargets.size() << std::endl;
    ((RenderTargets*)_render.get())->setTargets(rtargets);
    getRobWorkStudio()->postUpdateAndRepaint();
}


rw::common::PropertyMap& GTaskVisPlugin::settings(){
    return getRobWorkStudio()->getPropertyMap().get<rw::common::PropertyMap>("RobWorkStudioSettings");
}

void GTaskVisPlugin::loadTasks(bool automatic){
    std::string prevDir = settings().get<std::string>("RWSimLastnnedDIR","");
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

    loadTasks( QString(taskFile.c_str()) );
}

void GTaskVisPlugin::loadTasks(QString taskFile_tmp){
    std::string taskFile = taskFile_tmp.toStdString();
    GraspTask::Ptr gtask;
    try {
        gtask = GraspTask::load(taskFile);
    } catch (const Exception&) {
        QMessageBox::information(this, "GTaskVisPlugin", "Unable to load tasks from file");
        return;
    }
    _graspTask = gtask;
    std::string tcpID = gtask->getTCPID();
    std::string gripperID = gtask->getGripperID();

    int tcpIdx = _tcpSelectBox->findText(tcpID.c_str());
    if(tcpIdx!=-1)
        _tcpSelectBox->setCurrentIndex(tcpIdx);

    int gripperIdx = _deviceSelectBox->findText(gripperID.c_str());
    if(gripperIdx!=-1){
        _deviceSelectBox->setCurrentIndex(gripperIdx);
        Device::Ptr dev = _wc->findDevice( gripperID );
        std::string basename = dev->getBase()->getName();
        int baseIdx = _baseSelectBox->findText(basename.c_str());
        _baseSelectBox->setCurrentIndex(baseIdx);
    }
    _ymtargets = gtask->getAllTargets();

    _totalNrOfExperiments = (int)_ymtargets.size();
    log().info() << "LOAD TASKS DONE, nr of tasks: " << _ymtargets.size() << "\n";
    _updateBtn->setEnabled(true);
}


void GTaskVisPlugin::stateChangedListener(const State& state) {

}

void GTaskVisPlugin::selectGrasp(int index){
    const std::vector<RenderTargets::Target>& targets = ((RenderTargets*)_render.get())->getTargets();
    if( (index<0) || (index>=(int)targets.size()) ){
         return;
    }

    // get the selected TCP frame
    std::string tcpName = _tcpSelectBox->currentText().toStdString();
    std::string baseName = _baseSelectBox->currentText().toStdString();
    std::string objectName = _frameSelectBox->currentText().toStdString();
    std::string devName = _deviceSelectBox->currentText().toStdString();

    Frame* tcp = _wc->findFrame(tcpName);

    MovableFrame* base = _wc->findFrame<MovableFrame>(baseName);
    MovableFrame* object = _wc->findFrame<MovableFrame>(objectName);
    Device::Ptr device = _wc->findDevice(devName);

    if(object==NULL)
        RW_THROW("selected object is not movable frame!");
    if(base==NULL)
        RW_THROW("selected base is not movable frame!");
    if(device==NULL)
        RW_THROW("selected device is not valid!");


    State state = getRobWorkStudio()->getState();
    Transform3D<> wTtcp = targets[index].trans;
    //wTtcp.R().normalize();
    if(tcp!=NULL && base!=NULL){
        Transform3D<> baseTtcp = Kinematics::frameTframe(base, tcp ,state);
        base->moveTo(wTtcp * inverse(baseTtcp), state);
    }


    if(device!=NULL){
        Q q;
        if(targets[index].endTarget){
            if(targets[index].ctarget.result != NULL){
                q = targets[index].ctarget.result->gripperConfigurationGrasp;
            } else {
                q = targets[index].ctask->closeQ;
            }
        } else {
            if(targets[index].ctarget.result != NULL && targets[index].ctarget.result->gripperConfigurationGrasp.size()>0){
                q = targets[index].ctarget.result->gripperConfigurationGrasp;
            } else if( targets[index].ctask->openQ.size()>0 ){
                q = targets[index].ctask->openQ;
            }
        }
        if(q.size()==device->getDOF())
            device->setQ(q, state);
    }
    if( targets[index].ctarget.result!=NULL )
        log().info() << "Quality of selected grasp: " << targets[index].ctarget.result->qualityAfterLifting << std::endl;

    getRobWorkStudio()->setState(state);

}

void GTaskVisPlugin::genericAnyEventListener(const std::string& event, boost::any data){
    // used as control from ither plugins or scripts
    try{
        if(event=="GVis::LoadFile"){
            std::string file = boost::any_cast<std::string>(data);
            QMetaObject::invokeMethod( this, "loadTasks", Qt::QueuedConnection, Q_ARG( QString, QString(file.c_str()) ) );
        } else if(event=="GVis::Update"){
            QMetaObject::invokeMethod( this, "updateVis", Qt::QueuedConnection);
        } else if(event=="GVis::SelectGrasp"){
            int grasp = (int)boost::any_cast<double>(data);
            QMetaObject::invokeMethod( this, "selectGrasp", Qt::QueuedConnection, Q_ARG( int, grasp ) );
        }


    } catch (...){
        Log::warningLog() << "GTaskVisPlugin: Event \"" << event << "\" did not have the correct datatype or an error occured!\n";
    }
}

void GTaskVisPlugin::genericEventListener(const std::string& event){

}


void GTaskVisPlugin::on_btnRecordVideo_clicked() {
	for (size_t i = 0; i<63; i++) {
		Transform3D<> Tcam = getRobWorkStudio()->getView()->getSceneViewer()->getTransform();	

		Transform3D<> preTransform(Vector3D<>(0,0,0), RPY<>(0,0,0.1));
		getRobWorkStudio()->getView()->getSceneViewer()->setTransform(preTransform*Tcam);
		getRobWorkStudio()->updateAndRepaint();
		QApplication::processEvents();
		std::stringstream ss;
		ss<<"d:\\temp\\Img"<<i<<".png";
		getRobWorkStudio()->saveViewGL(ss.str().c_str());
	}
	for (size_t i = 63; i<63*2; i++) {
		Transform3D<> Tcam = getRobWorkStudio()->getView()->getSceneViewer()->getTransform();	
	
		Transform3D<> preTransform(Vector3D<>(0,0,0), RPY<>(0.1,0,0));
		getRobWorkStudio()->getView()->getSceneViewer()->setTransform(preTransform*Tcam);
		getRobWorkStudio()->updateAndRepaint();
		QApplication::processEvents();
		std::stringstream ss;
		ss<<"d:\\temp\\Img"<<i<<".png";
		getRobWorkStudio()->saveViewGL(ss.str().c_str());
	}



}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(GTaskVisPlugin)
#endif
