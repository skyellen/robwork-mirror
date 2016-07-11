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

#include "SupportPosePlugin.hpp"

#include <vector>

//#include <btBulletDynamicsCommon.h>

#include <QLayout>
#include <QVariant>
#include <QTreeWidgetItem>
#include <QInputDialog>
#include <QPushButton>
#include <QLabel>
#include <QShortcut>
#include <QKeySequence>

#include <boost/foreach.hpp>

#include <RobWorkStudio.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/models/Device.hpp>
#include <rwlibs/drawable/Drawable.hpp>

#include <rw/sensor/TactileArray.hpp>

//#include <simulator/bullet/BtSimulator.hpp>
//#include <simulator/bullet/BtDebugRender.hpp>

//#include <simulator/rwphysics/RWSimulator.hpp>
//#include <simulator/ode/ODESimulator.hpp>
#include <dynamics/KinematicDevice.hpp>
#include <dynamics/RigidDevice.hpp>
#include <loaders/DynamicWorkCellLoader.hpp>
#include <control/VelRampController.hpp>
#include <control/PDController.hpp>
#include <control/SyncPDController.hpp>
#include <sandbox/drawable/TactileArrayRender.hpp>

#include <simulator/PhysicsEngineFactory.hpp>

#include <rw/common/Log.hpp>
#include <rw/common/Exception.hpp>
#include <rw/common/LogStreamWriter.hpp>

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

using namespace rw::models;
using namespace rw::math;
using namespace rw::sensor;
using namespace rw::common;
using namespace rw::trajectory;
using namespace rwlibs::drawable;
using namespace rw::kinematics;
using namespace dynamics;
using namespace loaders;
using namespace drawable;

#define RW_DEBUG( str ) std::cout << str  << std::endl;

namespace
{
    QLabel* makeNumericQLabel(double val)
    {
        std::stringstream s; s << val;
        return new QLabel(s.str().c_str());

    }

    QDoubleSpinBox* makeDoubleSpinBox(double low, double high)
    {
    	QDoubleSpinBox* box = new QDoubleSpinBox();
        box->setDecimals(3);
        box->setRange(low, high);

        const double step = (high - low) / 1000;
        box->setSingleStep(step);

        return box;
    }

    void GLTransform(const Transform3D<>& transform)
    {
        GLfloat gltrans[16];
        for (int j = 0; j<3; j++) {
            for (int k = 0; k<3; k++)
                gltrans[j+4*k] = (float)transform(j,k);
            gltrans[12+j] = (float)transform(j, 3);
        }
        gltrans[3] = gltrans[7] = gltrans[11] = 0;
        gltrans[15] = 1;
        glMultMatrixf(gltrans);
    }


}


SupportPosePlugin::SupportPosePlugin():
    RobWorkStudioPlugin("SupportPosePlugin", QIcon(":/SimulationIcon.png")),
    _miscForces(40),
    _timeStep(0.03),
    _nextTime(0.0),
    _save(true),
    _simulator(NULL),
    _ghost(NULL),
    _simStartTime(0.0),
    _nrOfSims(0),
    _restPoseDialog(NULL)
{
    // Construct layout and widget
    QWidget *widg = new QWidget(this);
    QVBoxLayout *toplay = new QVBoxLayout(widg);
    widg->setLayout(toplay);
    this->setWidget(widg);
    {
        size_t row = 0;
        QGroupBox* box = new QGroupBox("Settings", this);
        toplay->addWidget(box);
        QGridLayout* lay = new QGridLayout(box);
        box->setLayout(lay);
        {
            QPushButton* button = new QPushButton("&Create simulator");
            lay->addWidget(button,row++,0);
            connect(button,SIGNAL(pressed()),this,SLOT(createSimulator()));
        }

        {
            QPushButton* button = new QPushButton("&Destroy simulator");
            lay->addWidget(button,row++,0);
            connect(button,SIGNAL(pressed()),this,SLOT(destroySimulator()));
        }

/*        _cmbDevices = new QComboBox();

        pLayout->addWidget(_cmbDevices, row++, 0);
        connect(_cmbDevices, SIGNAL(currentIndexChanged(int)), this, SLOT(cmbChanged ( int )));
*/

        _checkBox = new QCheckBox("Record state paths", this);
        lay->addWidget(_checkBox,row++,0);

        _forceUpdateBox = new QCheckBox("Force scene update",this);
        lay->addWidget(_forceUpdateBox,row++,0);

        {
            QLabel *label = new QLabel("dt: ");
            lay->addWidget(label, row, 0);

            _dtBox = makeDoubleSpinBox(0, 1);
            lay->addWidget(_dtBox, row++, 1); // own _box
            _dtBox->setValue(0.01);
        }

        {
            QLabel *label = new QLabel("Physics engine");
            lay->addWidget(label, row++, 0);
            std::vector<std::string> engineIDs =
                PhysicsEngineFactory::getEngineIDs();
            _engineBox = new QComboBox();
            BOOST_FOREACH(std::string engineID, engineIDs){
                _engineBox->addItem(engineID.c_str());
            }
            lay->addWidget(_engineBox);
        }
    }
    {
        size_t row = 0;
        QGroupBox* box = new QGroupBox("Control", this);

        toplay->addWidget(box);
        QGridLayout* lay = new QGridLayout(box);
        box->setLayout(lay);

        {
            QPushButton* button = new QPushButton("Resting Pose sim");
            connect(button,SIGNAL(clicked()),this,SLOT(startRestingPoseSim()));
            lay->addWidget(button,row++,0);
        }

        {
            QPushButton* button = new QPushButton("Step");
            connect(button,SIGNAL(clicked()),this,SLOT(update()));
            lay->addWidget(button,row++,0);
        }

        {
            QPushButton* button = new QPushButton("Start");
            connect(button,SIGNAL(clicked()),this,SLOT(startSimulation()));
            lay->addWidget(button,row,0);
        }

        {
            QPushButton* button = new QPushButton("Stop");
            connect(button,SIGNAL(clicked()),this,SLOT(stopSimulation()));
            lay->addWidget(button,row++,1);
        }

        {
            QPushButton* button = new QPushButton("Reset");
            connect(button,SIGNAL(clicked()),this,SLOT(resetSimulation()));
            lay->addWidget(button,row++,0);
        }

        {
            QPushButton* button = new QPushButton("Save state path");
            connect(button,SIGNAL(clicked()),this,SLOT(setSave()));
            lay->addWidget(button,row++,0);
        }
    }

    {
        size_t row = 0;
        QGroupBox* box = new QGroupBox("Status", this);

        toplay->addWidget(box);
        QGridLayout* lay = new QGridLayout(box);
        box->setLayout(lay);

/*      {
            QLabel* button = new QLabel("Step simulation");
            connect(button,SIGNAL(clicked()),this,SLOT(update()));
            lay->addWidget(button,row++,0);
        }

        {
            QPushButton* button = new QPushButton("Start simulation");
            connect(button,SIGNAL(clicked()),this,SLOT(startSimulation()));
            lay->addWidget(button,row,0);
        }

        {
            QPushButton* button = new QPushButton("Stop simulation");
            connect(button,SIGNAL(clicked()),this,SLOT(stopSimulation()));
            lay->addWidget(button,row++,1);
        }

        {
            QPushButton* button = new QPushButton("Reset simulation");
            connect(button,SIGNAL(clicked()),this,SLOT(resetSimulation()));
            lay->addWidget(button,row++,0);
        }

        {
            QPushButton* button = new QPushButton("Save state path");
            connect(button,SIGNAL(clicked()),this,SLOT(setSave()));
            lay->addWidget(button,row++,0);
        }
*/
        {
            QLabel *label = new QLabel("dt: ");
            lay->addWidget(label, row, 0);

            _timeLabel = makeNumericQLabel(0.0);
            lay->addWidget(_timeLabel,row++,1);
        }

        {
            QLabel *label = new QLabel("Nr of sims: ");
            lay->addWidget(label, row, 0);

            _nrOfSimsLabel = makeNumericQLabel(0.0);
            lay->addWidget(_nrOfSimsLabel,row++,1);
        }

    }


    std::ofstream *myfile = new std::ofstream();
    myfile->open ("debuglog.txt");
    rw::common::Log::setWriter("Debug", new rw::common::LogStreamWriter(myfile));
    _timer = new QTimer( NULL );
    connect( _timer, SIGNAL(timeout()), this, SLOT(saveState()) );
    toplay->addStretch(1);

}


SupportPosePlugin::~SupportPosePlugin() {

}

void SupportPosePlugin::updateCfgInfo(){


    // check if debug draw is checked
    if( _dBtWorld != NULL ){
        if( _debugDrawBox->isChecked() ) _dBtWorld->setEnabled( true );
        else _dBtWorld->setEnabled( false );
    }
    getRobWorkStudio()->updateAndRepaint();
}

void SupportPosePlugin::setSave(){
    getRobWorkStudio()->setTimedStatePath(_statePath);
    getRobWorkStudio()->updateAndRepaint();
}
#include <rw/proximity/CollisionDetector.hpp>
void SupportPosePlugin::startRestingPoseSim(){

    if (!_restPoseDialog ) {
        State state = getRobWorkStudio()->getState();
        rw::proximity::CollisionDetector *colDect = getRobWorkStudio()->getCollisionDetector();
        _restPoseDialog = new RestingPoseDialog(state, _dwc.get(), colDect,  this);
        connect(_restPoseDialog,SIGNAL(updateView()),this,SLOT(updateViewEvent()) );
    }

    _restPoseDialog->show();
    _restPoseDialog->raise();
    _restPoseDialog->activateWindow();

}

void SupportPosePlugin::updateViewEvent(){
    if( !_restPoseDialog )
        return;
    QObject *obj = sender();

    if( obj==_restPoseDialog ){
        State state = _restPoseDialog->getState();
        getRobWorkStudio()->setState(state);
    }
}

void SupportPosePlugin::createSimulator(){
    if( _dwc == NULL ){
        RW_DEBUG("dwc is null!");
        return;
    }

    // update the bodies that are to be thrown randomly
    _bodies.clear();
    BOOST_FOREACH(Body *body, _dwc->getBodies()){
        if( RigidBody *rbody=dynamic_cast<RigidBody*>(body) ){
            _bodies.push_back(rbody);
        }
    }

    std::cout << "Nr of rigid bodies: "<< _bodies.size() << std::endl;

    State state = getRobWorkStudio()->getState();

    std::string engineId = _engineBox->currentText().toStdString();
    RW_DEBUG("- Selected physics engine: " << engineId);
    Simulator *sim = PhysicsEngineFactory::newPhysicsEngine(engineId,_dwc.get());
    sim->initPhysics(state);

    _simulator = new ThreadSimulator(sim, state);

    _dState = state;
    _defState = state;
    _jointState = getRobWorkStudio()->getState();
}

void SupportPosePlugin::destroySimulator(){

    if(_simulator==NULL)
        return;

    _simulator->stop();
    Simulator *sim = _simulator->getSimulator();
    delete sim;
    delete _simulator;
    _simulator = NULL;
}

void SupportPosePlugin::open(rw::models::WorkCell* workcell)
{
    RW_DEBUG("SupportPosePlugin Open");
    std::cout << "SupportPosePlugin Open" << std::endl;

    if( workcell==NULL){
        RW_DEBUG("workcell is null!");
        return;
    }


}

void SupportPosePlugin::genericEventListener(const std::string& event){
    //std::cout << "Generic event: " << event << std::endl;
    if( event=="DynamicWorkCellLoaded" ){
        // get the dynamic workcell from the propertymap
        RW_DEBUG("Getting dynamic workcell from propertymap!");
        boost::shared_ptr<DynamicWorkCell> *dwc =
            getRobWorkStudio()->getPropertyMap().
                getPtr<boost::shared_ptr<DynamicWorkCell> >("DynamicWorkcell");
        if( dwc==NULL ){
            //// std::cout   << "No dynamic workcell in propertymap!!" << std::endl;
            return;
        }
        RW_DEBUG("Setting dynamic workcell!");
        _dwc = *dwc;

    }
}


void SupportPosePlugin::close()
{
    _engineBox->setDisabled(false);

    if(_simulator==NULL)
        return;

    _simulator->stop();
    Simulator *sim = _simulator->getSimulator();
    delete sim;
    delete _simulator;
}

void SupportPosePlugin::printContactGraph(){
    //_dworkcell->getContactGraph().writeToFile("contactgraph.dot");
}

void SupportPosePlugin::startSimulation(){
    if( _simulator==NULL ){
        std::cout << "Simulator null" << std::endl;
    }
    if( _simulator->isRunning() )
        return;

	bool saveToLog = false;
	if( _checkBox->isChecked() ){
		saveToLog = true;
	}
	_nrOfSims = 0;
	_simStartTime = 0.0;
	_maxNrOfSims = 100;

	_simulator->start();
    //_btnStart->setDisabled(true);
    //_btnUpdate->setDisabled(true);
    _checkBox->setDisabled(true);
    _nextTime += _timeStep;
	_timer->start(100);
	std::cout << "END start!" << std::endl;
}

void SupportPosePlugin::stopSimulation(){
    //_btnStart->setDisabled(false);
    //_btnUpdate->setDisabled(false);
    _checkBox->setDisabled(false);

    _simulator->stop();

	_timer->stop();
}

void SupportPosePlugin::resetSimulation(){

    if(_simulator==NULL)
		return;

	_simulator->stop();
	State state = getRobWorkStudio()->getState();
	_simulator->getSimulator()->resetScene(state);
	_dState = state;
	getRobWorkStudio()->updateAndRepaint();
}

void grabItem(){
    Q start(7);

}

void SupportPosePlugin::update(){
    if( _simulator==NULL )
        return;

    if( _simulator->isRunning() ){
        RW_WARN("Simulator is already running...");
        return;
    }
    RW_DEBUG("getState");
    _dState = _simulator->getState();
    // get dt and make a simulation step
    RW_DEBUG("get dt");
    double dt = _dtBox->value();
    RW_DEBUG("step");
    _simulator->getSimulator()->step(dt, _dState);

}

void SupportPosePlugin::saveState(){
    /*if( _simulator==NULL )
        return;

    if( !_simulator->isRunning() ){
        return;
    }

    State state = _simulator->getState();

    // check the velocity of all the bodies
    double lVelThres = 0.0001, aVelThres = 0.00001;
    bool allBelowThres = true;
    BOOST_FOREACH(RigidBody *rbody, _bodies){
        // get velocity of rbody
        // if above threshold then break and continue
        Vector3D<> avel = rbody->getAngVel();
        Vector3D<> lvel = rbody->getLinVel();

        if (lvel.norm2()>lVelThres || avel.norm2()>aVelThres){
            allBelowThres = false;
            break;
        }
    }
    // get dt and make a simulation step
    double time = _simulator->getTime();

    // if requestet add the state to the state trajectory
    if(_save)
        _statePath.push_back( Timed<State>(time+_simStartTime, state) );

    if( allBelowThres && time>_simStartTime+2){
        std::cout << "STOP SIMULATOR" << std::endl;
        _simulator->stop();
        _supportPoses.push_back(state);
        std::cout << "Simulator stopped at time " << time << std::endl;
        // recalc random start configurations and reset the simulator
        state = calcRandomCfg(_bodies, _defState);
        _simulator->getSimulator()->resetScene(state);
        _simStartTime = time;

        // update the nr of sims label
        std::stringstream s; s << _nrOfSims++;
        _nrOfSimsLabel->setText(s.str().c_str());

        if( _nrOfSims>_maxNrOfSims )
            return;

        _simulator->start();
    }

    // update the time label
    std::stringstream s; s << time;
    _timeLabel->setText(s.str().c_str());

    // and last signal that workcell state has changed if user request it
    if(_forceUpdateBox->isChecked())
        getRobWorkStudio()->setState(state);

    _dState = state;
    */
}




void SupportPosePlugin::stateChangedHandler(RobWorkStudioPlugin* sender)
{
}

void SupportPosePlugin::initialize(){
    getRobWorkStudio()->genericEvent().add(
          boost::bind(&SupportPosePlugin::genericEventListener, this, _1), this);
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(SupportPosePlugin);
#endif
