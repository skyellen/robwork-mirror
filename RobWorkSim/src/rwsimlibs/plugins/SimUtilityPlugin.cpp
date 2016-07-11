/************************************************************************
 * RobWorkStudio Version 0.1
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * This Software is developed using the Qt Open Source Edition, and is
 * therefore only available under the GNU General Public License (GPL).
 *
 * RobWorkStudio can be used, modified and redistributed freely.
 * RobWorkStudio is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWorkStudio relies on RobWork, which has a different
 * license. For more information goto your RobWork directory and read license.txt.
 ************************************************************************/

#include "SimUtilityPlugin.hpp"

#include <vector>
#include <rw/common/TimerUtil.hpp>
//#include <btBulletDynamicsCommon.h>

#include <QLayout>
#include <QVariant>
#include <QTreeWidgetItem>
#include <QInputDialog>
#include <QPushButton>
#include <QLabel>
#include <QShortcut>
#include <QKeySequence>
#include <QGroupBox>

#include <boost/foreach.hpp>

#include <RobWorkStudio.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/models/Device.hpp>
#include <rwlibs/opengl/Drawable.hpp>

#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/control/VelRampController.hpp>
#include <rwsim/control/PDController.hpp>
#include <rwsim/control/SyncPDController.hpp>
#include <rwlibs/opengl/TactileArrayRender.hpp>

#include <rwsim/simulator/PhysicsEngineFactory.hpp>

#include <rw/common/Log.hpp>
#include <rw/common/Exception.hpp>
#include <rw/common/LogStreamWriter.hpp>

#include <rw/proximity/CollisionDetector.hpp>

#include <rwsimlibs/gui/RestingPoseDialog.hpp>
#include <rwsimlibs/gui/GraspRestingPoseDialog.hpp>
#include <rwsimlibs/gui/SupportPoseAnalyserDialog.hpp>

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

using namespace rw::models;
using namespace rw::math;
using namespace rw::sensor;
using namespace rw::common;
using namespace rw::trajectory;
using namespace rw::graphics;
using namespace rwlibs::opengl;
using namespace rw::kinematics;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;
using namespace rwsim::drawable;

using namespace rws;

//#define RW_DEBUG( str ) std::cout << str  << std::endl;

SimUtilityPlugin::SimUtilityPlugin():
    RobWorkStudioPlugin("SimUtilityPlugin", QIcon(":/SimulationIcon.png")),
    _restPoseDialog(NULL),
    _poseAnalyserDialog(NULL),
    _graspRestPoseDialog(NULL)
{
    // Construct layout and widget
    QWidget *widg = new QWidget(this);
    QVBoxLayout *toplay = new QVBoxLayout(widg);
    widg->setLayout(toplay);
    this->setWidget(widg);
    {
        int row = 0;
        QGroupBox* box = new QGroupBox("Utilities", this);
        toplay->addWidget(box);
        QGridLayout* lay = new QGridLayout(box);
        box->setLayout(lay);
        {
            QPushButton* button = new QPushButton("Resting pose");
            connect(button,SIGNAL(clicked()),this,SLOT(btnPressed()));
            lay->addWidget(button,row++,0);
            _restPoseBtn = button;
        }
        {
            QPushButton* button = new QPushButton("Support pose");
            connect(button,SIGNAL(clicked()),this,SLOT(btnPressed()));
            lay->addWidget(button,row++,0);
            _poseAnalyserBtn = button;
        }
        {
            QPushButton* button = new QPushButton("Tool eval");
            connect(button,SIGNAL(clicked()),this,SLOT(btnPressed()));
            lay->addWidget(button,row++,0);
            _toolEvalBtn = button;
        }
        {
            QPushButton* button = new QPushButton("Grasp selection");
            connect(button,SIGNAL(clicked()),this,SLOT(btnPressed()));
            lay->addWidget(button,row++,0);
            _graspSelectBtn = button;
        }

    }

    _restPoseBtn->setEnabled(false);
    _poseAnalyserBtn->setEnabled(false);
    _toolEvalBtn->setEnabled(false);
    _graspSelectBtn->setEnabled(false);

    toplay->addStretch(1);
}


SimUtilityPlugin::~SimUtilityPlugin() {

}

void SimUtilityPlugin::btnPressed(){
	QObject *obj = sender();
	//std::cout << "BtnPressed" << std::endl;
	if( obj == _poseAnalyserBtn ){
		if(_dwc==NULL)
			RW_THROW("No dynamic workcell loaded!");

	    if (!_poseAnalyserDialog ) {
	        State state = getRobWorkStudio()->getState();
	        rw::proximity::CollisionDetector::Ptr colDect = getRobWorkStudio()->getCollisionDetector();
	        _poseAnalyserDialog = new SupportPoseAnalyserDialog(state, _dwc.get(), colDect.get(),  getRobWorkStudio(), NULL);
	        connect(_poseAnalyserDialog,SIGNAL(stateChanged(const rw::kinematics::State&)),this,SLOT(stateChangedEvent(const rw::kinematics::State&)) );
	    }
	    _poseAnalyserDialog->show();
	    _poseAnalyserDialog->raise();
	    _poseAnalyserDialog->activateWindow();

	} else if( obj==_restPoseBtn ){
		if(_dwc==NULL)
			RW_THROW("No dynamic workcell loaded!");

	    if (!_restPoseDialog ) {
	        State state = getRobWorkStudio()->getState();
	        rw::proximity::CollisionDetector::Ptr colDect = getRobWorkStudio()->getCollisionDetector();
	        _restPoseDialog = new RestingPoseDialog(state, _dwc.get(), colDect.get(),  NULL);
	        connect(_restPoseDialog,SIGNAL(stateChanged(const rw::kinematics::State&)),this,SLOT(stateChangedEvent(const rw::kinematics::State&)) );
	    }

	    _restPoseDialog->show();
	    _restPoseDialog->raise();
	    _restPoseDialog->activateWindow();
	} else if( obj ==_toolEvalBtn ){
		if(_dwc==NULL)
			RW_THROW("No dynamic workcell loaded!");
		log().info() << "creating grasp rest pose \n";
	    if (!_graspRestPoseDialog ) {
	        State state = getRobWorkStudio()->getState();
	        rw::proximity::CollisionDetector::Ptr colDect = getRobWorkStudio()->getCollisionDetector();
	        _graspRestPoseDialog = new GraspRestingPoseDialog(state, _dwc.get(), colDect.get(),  NULL);
	        connect(_graspRestPoseDialog,SIGNAL(stateChanged(const rw::kinematics::State&)),this,SLOT(stateChangedEvent(const rw::kinematics::State&)) );
	        connect(_graspRestPoseDialog,SIGNAL(restingPoseEvent(const RestingConfig&)),
	                this,SLOT(restConfigEvent(const RestingConfig&)) );
	    }
	    log().info() << "Showing dialog!\n";
	    _graspRestPoseDialog->show();
	    _graspRestPoseDialog->raise();
	    _graspRestPoseDialog->activateWindow();
	} else if( obj == _timer ){

	} else if( obj==_graspSelectBtn ){
		if(_dwc==NULL)
			RW_THROW("No dynamic workcell loaded!");

	    if (!_graspSelectionDialog ) {
	        State state = getRobWorkStudio()->getState();
	        rw::proximity::CollisionDetector::Ptr colDect = getRobWorkStudio()->getCollisionDetector();
	        _graspSelectionDialog = new GraspSelectionDialog(state, _dwc.get(), colDect.get(),  NULL);
	        connect(_graspSelectionDialog,SIGNAL(stateChanged(const rw::kinematics::State&)),this,SLOT(stateChangedEvent(const rw::kinematics::State&)) );
	    }

	    _graspSelectionDialog->show();
	    _graspSelectionDialog->raise();
	    _graspSelectionDialog->activateWindow();
	}
}

void SimUtilityPlugin::restConfigEvent(const RestingConfig& restcfg){
    //std::cout << "rest config event\n";
    getRobWorkStudio()->setState(restcfg._state);
    getRobWorkStudio()->updateAndRepaint();

    //Frame *world = _dwc->getWorkcell()->getWorldFrame();
    //std::string gqual = world->getPropertyMap().get<std::string>("GraspQuality",std::string("NO"));
    //long time = TimerUtil::currentTimeMs()/1000;
    std::stringstream sstr;
    //std::string pathPre = world->getPropertyMap().get<std::string>("PathPre",std::string("c:/tmp"));
    //int s = time;
    //sstr << pathPre << "/RestImageSave_" << gqual << ".png";
    sstr << restcfg._desc << ".png";

    getRobWorkStudio()->getView()->saveBufferToFile(sstr.str().c_str());
    //std::cout << "rest config event saved" << std::endl;

}

void SimUtilityPlugin::stateChangedEvent(const rw::kinematics::State& state){
	getRobWorkStudio()->setState(state);
}

void SimUtilityPlugin::updateViewEvent(){
	if( !_restPoseDialog )
        return;
	//std::cout << "UpdateViewEvent" << std::endl;
    QObject *obj = sender();

    if( obj==_restPoseDialog ){
    	//std::cout << "UpdateViewEvent1" << std::endl;
        State state = _restPoseDialog->getState();
        getRobWorkStudio()->setState(state);
    }
}

void SimUtilityPlugin::open(rw::models::WorkCell* workcell)
{

}

void SimUtilityPlugin::genericEventListener(const std::string& event){
    //std::cout << "Generic event: " << event << std::endl;
    if( event=="DynamicWorkCellLoaded" ){
        // get the dynamic workcell from the propertymap
        RW_DEBUG("Getting dynamic workcell from propertymap!");

        Ptr<DynamicWorkCell> dwc =
			getRobWorkStudio()->getPropertyMap().get<Ptr<DynamicWorkCell> >("DynamicWorkcell",NULL);

        if( dwc==NULL){
        	std::cout << "Could not load dynamic workcell from propertymap!!" << std::endl;
        	return;
        }

        _dwc = dwc;

        _restPoseBtn->setEnabled(true);
        _poseAnalyserBtn->setEnabled(true);
        _toolEvalBtn->setEnabled(true);
        _graspSelectBtn->setEnabled(true);

        if( getRobWorkStudio()->getPropertyMap().has("Arg1") ){
            if(_dwc==NULL)
                RW_THROW("No dynamic workcell loaded!");
            log().info() << "creating grasp rest pose \n";
            if (!_graspRestPoseDialog ) {
                State state = getRobWorkStudio()->getState();
                rw::proximity::CollisionDetector::Ptr colDect = getRobWorkStudio()->getCollisionDetector();
                _graspRestPoseDialog = new GraspRestingPoseDialog(state, _dwc.get(), colDect.get(),  NULL);
                connect(_graspRestPoseDialog,SIGNAL(stateChanged(const rw::kinematics::State&)),this,SLOT(stateChangedEvent(const rw::kinematics::State&)) );
    	        connect(_graspRestPoseDialog,SIGNAL(restingPoseEvent(const RestingConfig&)),
    	                this,SLOT(restConfigEvent(const RestingConfig&)) );

            }
            log().info() << "Showing dialog!\n";
            _graspRestPoseDialog->show();
            _graspRestPoseDialog->raise();
            _graspRestPoseDialog->activateWindow();

            std::string saveDir = getRobWorkStudio()->getPropertyMap().get<std::string>("Arg2");
            _graspRestPoseDialog->setSaveDir(saveDir);

            std::string preshape = getRobWorkStudio()->getPropertyMap().get<std::string>("Arg3");
            _graspRestPoseDialog->setPreshapeStrategy(preshape);

            std::string id = getRobWorkStudio()->getPropertyMap().get<std::string>("Arg4",std::string("1"));
            _graspRestPoseDialog->setUniqueID(id);

            _graspRestPoseDialog->startAuto();
        }
    }
}

void SimUtilityPlugin::close()
{
    _restPoseBtn->setEnabled(false);
    _poseAnalyserBtn->setEnabled(false);
    _toolEvalBtn->setEnabled(false);
    _graspSelectBtn->setEnabled(false);
    _dwc = NULL;
    if(_restPoseDialog)
    	delete _restPoseDialog;
    if(_poseAnalyserDialog)
    	delete _poseAnalyserDialog;
}

void SimUtilityPlugin::stateChangedHandler(RobWorkStudioPlugin* sender)
{

}

void SimUtilityPlugin::initialize(){
    getRobWorkStudio()->genericEvent().add(
          boost::bind(&SimUtilityPlugin::genericEventListener, this, _1), this);



    Log::setLog( _log );

    _timer = new QTimer( NULL );
    _timer->setSingleShot(true);

    _timer->setInterval( 4000 );
    connect( _timer, SIGNAL(timeout()), this, SLOT(btnPressed()) );
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(SimUtilityPlugin);
#endif
