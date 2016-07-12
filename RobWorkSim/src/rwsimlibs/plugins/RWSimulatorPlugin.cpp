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

#include "RWSimulatorPlugin.hpp"

#include <vector>

#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QComboBox>
#include <QCheckBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QDoubleSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QGridLayout>

#include <RobWorkStudio.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/common/StringUtil.hpp>
//#include <rw/models/Device.hpp>
#include <rwlibs/opengl/Drawable.hpp>

#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwlibs/opengl/TactileArrayRender.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwsim/sensor/TactileArraySensor.hpp>

#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>

#include <rw/common/Log.hpp>
#include <rw/common/Exception.hpp>
#include <rw/common/LogStreamWriter.hpp>

#include <sstream>
#include <fstream>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::common;
using rw::graphics::Render;
using rw::trajectory::Timed;
using rw::kinematics::State;
using namespace rwlibs::opengl;
using rwsim::dynamics::DynamicWorkCell;
using rwsim::loaders::DynamicWorkCellLoader;
using namespace rwsim::simulator;
using rwlibs::simulation::SimulatedSensor;
using rws::RobWorkStudioPlugin;

#define RW_DEBUGRWS( str ) std::cout << str  << std::endl;

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

RWSimulatorPlugin::RWSimulatorPlugin():
    RobWorkStudioPlugin("RWSimulatorPlugin", QIcon(":/SimulationIcon.png")),
    //_miscForces(40),
    _timeStep(0.03),
    _nextTime(0.0),
    _save(true)
    //_jointDialog(NULL),
    //_jointDialog1(NULL),
    //_ghost(NULL)
{
    // Construct layout and widget
    QWidget *widg = new QWidget(this);
    QVBoxLayout *toplay = new QVBoxLayout(widg);
    widg->setLayout(toplay);
    this->setWidget(widg);
    {
        int row = 0;
        QGroupBox* box = new QGroupBox("Settings", this);
        toplay->addWidget(box);
        QGridLayout* lay = new QGridLayout(box);
        box->setLayout(lay);
        {
            QPushButton* button = new QPushButton("&Open Scene");
            lay->addWidget(button,row++,0);
            connect(button,SIGNAL(pressed()),this,SLOT(open()));
        }
        {
            QPushButton* button = new QPushButton("Open Default");
            lay->addWidget(button,row++,0);
            connect(button,SIGNAL(pressed()),this,SLOT(openControlDialog()));
        }
        {
            QPushButton* button = new QPushButton("Open Default1");
            lay->addWidget(button,row++,0);
            connect(button,SIGNAL(pressed()),this,SLOT(openControlDialog2()));
        }
        {
            QPushButton* button = new QPushButton("&Open Hand Control");
            lay->addWidget(button,row++,0);
            connect(button,SIGNAL(pressed()),this,SLOT(openControlDialog1()));
        }


        _checkBox = new QCheckBox("Record state path",this);
        lay->addWidget(_checkBox,row++,0);

        _debugDrawBox = new QCheckBox("Debug draw lvl",this);
        connect( _debugDrawBox, SIGNAL(clicked()), this, SLOT( updateCfgInfo() ) );
        lay->addWidget(_debugDrawBox,row++,0);

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
                PhysicsEngine::Factory::getEngineIDs();
            _engineBox = new QComboBox();
            BOOST_FOREACH(std::string engineID, engineIDs){
                _engineBox->addItem(engineID.c_str());
            }
            lay->addWidget(_engineBox);
        }
    }
    {
        int row = 0;
        QGroupBox* box = new QGroupBox("Control", this);

        toplay->addWidget(box);
        QGridLayout* lay = new QGridLayout(box);
        box->setLayout(lay);

        {
            QPushButton* button = new QPushButton("Step simulation");
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

        {
            QLabel *label = new QLabel("dt: ");
            lay->addWidget(label, row, 0);

            _timeLabel = makeNumericQLabel(0.0);
            lay->addWidget(_timeLabel,row++,1);
        }

    }
    std::ofstream *myfile = new std::ofstream();
    myfile->open ("debuglog.txt");
    Log::log().setWriter(Log::Debug, new LogStreamWriter(myfile));
    _timer = new QTimer( NULL );
    connect( _timer, SIGNAL(timeout()), this, SLOT(update()) );

    toplay->addStretch(1);



}


RWSimulatorPlugin::~RWSimulatorPlugin() {

}

void RWSimulatorPlugin::updateCfgInfo(){
    // check if debug draw is checked
    if( _dBtWorld != NULL ){
        if( _debugDrawBox->isChecked() ) _dBtWorld->setVisible( true );
        else _dBtWorld->setVisible( false );
    }
    getRobWorkStudio()->updateAndRepaint();
}

void RWSimulatorPlugin::setSave(){
    getRobWorkStudio()->setTimedStatePath(_statePath);
    getRobWorkStudio()->updateAndRepaint();
}

void RWSimulatorPlugin::openControlDialog(){

    open("C:/workspace/RobWorkApp/rwsim/scenes/MoveBotSetup/DynMovebotSetup.xml");
}

void RWSimulatorPlugin::openControlDialog2(){
	open("C:/workspace/RobWorkApp/rwsim/scenes/MoveBotSetup/DynMovebotSetupNoKin.xml");
}

void RWSimulatorPlugin::openControlDialog1(){

    /*
    if (!_jointDialog1 ) {
        _jointDialog1 = new JointControlDialog(_controllers[0], this);
    }

    _jointDialog1->show();
    _jointDialog1->raise();
    _jointDialog1->activateWindow();
    */
    //getRobWorkStudio()->updateAndRepaint();
}


void RWSimulatorPlugin::open(rw::models::WorkCell* workcell)
{
    RW_DEBUGRWS("Open");
    if( workcell==NULL || _dworkcell == NULL)
        return;

    if( _dworkcell->getWorkcell()!= workcell ){
        return;
    }

    _engineBox->setDisabled(true);

    State state = getRobWorkStudio()->getState();
    //_dState = state;

    _jointState = getRobWorkStudio()->getState();

    RW_DEBUGRWS("---------------- InitPhysics:");

    _simulator->init(state);
    RW_DEBUGRWS("---------------- InitPhysics: FINISHED");

    RW_DEBUGRWS("Create render");
    Render::Ptr render = _simulator->createDebugRender();
    if(render!=NULL){
        _dBtWorld = new Drawable(render, "btWorlRender");
        getRobWorkStudio()->getWorkCellScene()->addDrawable(_dBtWorld, workcell->getWorldFrame());
        _dBtWorld->setVisible(false);
    } else {
        RW_WARN("Physics engine does not support debug rendering!");
    }

    BOOST_FOREACH(SimulatedSensor::Ptr sensor,  _dworkcell->getSensors()){
        if( sensor.cast<rwsim::sensor::TactileArraySensor>() ){
        	rwsim::sensor::TactileArraySensor::Ptr tsensor = sensor.cast<rwsim::sensor::TactileArraySensor>();
            TactileArrayRender *render = new TactileArrayRender(tsensor->getTactileArrayModel());
            Drawable *drawable = new Drawable(boost::shared_ptr<Render>(render), tsensor->getName());
            getRobWorkStudio()->getWorkCellScene()->addDrawable(drawable, tsensor->getFrame());
        }
    }


   // std::cout  << "Adding simulator too property map" << std::endl;
    getRobWorkStudio()->getPropertyMap().add<DynamicSimulator::Ptr>(
            "Simulator",
            "A dynamic simulator",
            _simulator );
    getRobWorkStudio()->genericEvent().fire("SimulatorLoadet");


/*
    std::vector<BodyController*> ctrls = _dworkcell->getControllers();
    for(size_t i=0; i<ctrls.size(); i++){
    	FixedDevice *fdev = dynamic_cast<FixedDevice*>( ctrls[i] );
    	if( fdev == NULL )
    		continue;
    	_fdevs.push_back(fdev);
    }

    std::vector<Body*> bodies = _dworkcell->getBodies();
   // std::cout  << "Bodies recieved" << std::endl;
    // create an array of force drawables
    rw::kinematics::Frame *world = workcell->getWorldFrame();
    _bodyForces.clear();
    for(size_t i=0;i<bodies.size(); i++){
        rw::kinematics::Frame *frame = &(bodies[i]->getBodyFrame());
        rw::math::Transform3D<> wTb = rw::kinematics::Kinematics::WorldTframe(frame,*_state);
        DrawableForce *dforce = new DrawableForce();
        getWorkCellGLDrawer()->addDrawableToFrame(world, dforce);
        dforce->setPosition( wTb.P() );
        _bodyForces.push_back(dforce);
    }

    for(size_t i=0;i<_miscForces.size(); i++){
        DrawableForce *dforce = new DrawableForce();
        _miscForces[i] = dforce;
        getWorkCellGLDrawer()->addDrawableToFrame(world, dforce);
    }
   // std::cout  << "Bodyforces initialized!!" << std::endl;
*/
}

void RWSimulatorPlugin::genericEventListener(const std::string& event){
    //std::cout << "Generic event: " << event << std::endl;
    if(event=="StartSimulation"){
        startSimulation();
    } else if(event=="StopSimulation"){
        stopSimulation();
    } else if(event=="ResetSimulation"){
        resetSimulation();
    }
}


void RWSimulatorPlugin::close()
{
    _engineBox->setDisabled(false);

}

void RWSimulatorPlugin::printContactGraph(){
    //_dworkcell->getContactGraph().writeToFile("contactgraph.dot");
}

void RWSimulatorPlugin::startSimulation(){
	//bool saveToLog = false;
	/*if( _checkBox->isChecked() ){
		saveToLog = true;
	}*/

    //_btnStart->setDisabled(true);
    //_btnUpdate->setDisabled(true);
    _checkBox->setDisabled(true);
    _nextTime += _timeStep;
	_timer->start(10);
}

void RWSimulatorPlugin::stopSimulation(){
    //_btnStart->setDisabled(false);
    //_btnUpdate->setDisabled(false);
    _checkBox->setDisabled(false);

	_timer->stop();
}

void RWSimulatorPlugin::resetSimulation(){
	if(_dworkcell==NULL)
		return;
	stopSimulation();
	State state = getRobWorkStudio()->getState();
	_simulator->reset(state);
	getRobWorkStudio()->updateAndRepaint();
}

void grabItem(){
    Q start(7);

}

void RWSimulatorPlugin::update(){
    if(_dworkcell==NULL)
        return;

    // get dt and make a simulation step
    double dt = _dtBox->value();
    try{
    	_simulator->step(dt);
    } catch(...){
    	std::cout << "catched error" << std::endl;
    }
    double time = _simulator->getTime();

    // if requestet add the state to the state trajectory
    if(_save)
        _statePath.push_back( Timed<State>(time, _simulator->getState() ) );


    // update the time label
    std::stringstream s; s << time;
    _timeLabel->setText(s.str().c_str());

    // and last signal that workcell state has changed if user request it
    if(_forceUpdateBox->isChecked()){
        getRobWorkStudio()->setState( _simulator->getState() );
        getRobWorkStudio()->updateAndRepaint();
    }
}


void RWSimulatorPlugin::stateChangedHandler(RobWorkStudioPlugin* sender)
{
	//_dState = *_state;
	if (isVisible() ) {

    	/*
    	for(size_t i=0; i<_fdevs.size(); i++){
	    	Device *dev = _fdevs[i]->getSlaveDevice();
	    	Q backup = dev->getQ( _dState );
	    	Q goal = dev->getQ(*_state);
	    	// set fdevs goal to the changed q
	    	_fdevs[i]->setGoalQ( goal );
	    	_dState = *_state;
	    	//dev->setQ(backup, _dState);
	    	dev->setQ(goal, _dState);
    	}*/
    }
}

void RWSimulatorPlugin::open(){
    QString selectedFilter;

    const QString dir(_previousOpenDirectory.c_str());

    QString filename = QFileDialog::getOpenFileName(
        this,
        "Open Drawable", // Title
        dir, // Directory
        "All supported ( *.xml )"
        " \nRW XML files ( *.xml )"
        " \n All ( *.* )",
        &selectedFilter);

    std::string file = filename.toStdString();
    open(file);
}


void RWSimulatorPlugin::open(const std::string& file)
{
    if (!file.empty()){
        _previousOpenDirectory = rw::common::StringUtil::getDirectoryName(file);

        try {
        	_dworkcell = DynamicWorkCellLoader::load(file);
        } catch (const Exception& exp) {
            QMessageBox::information(
                NULL,
                "Exception",
                exp.getMessage().getText().c_str(),
                QMessageBox::Ok);
            return;
        }
        if(_dworkcell==NULL)
        	RW_THROW("Dynamic workcell is null");

        RW_DEBUGRWS("Adding controllers to list: ");
        // add controllers to the devices
        //_dState = _dworkcell->getWorkcell()->getDefaultState();

        std::string engineId = _engineBox->currentText().toStdString();
        RW_DEBUGRWS("- Selected physics engine: " << engineId);
        _simulator = ownedPtr( new DynamicSimulator(_dworkcell, PhysicsEngine::Factory::makePhysicsEngine(engineId,_dworkcell.get())) );

        getRobWorkStudio()->setWorkcell( _dworkcell->getWorkcell() );

        RW_DEBUGRWS("Adding dynamic workcell too property map");
        getRobWorkStudio()->getPropertyMap().add<Ptr<DynamicWorkCell> >(
                "DynamicWorkcell",
                "A dynamic simulator",
                _dworkcell );

        RW_DEBUGRWS("fire event!! ");
        getRobWorkStudio()->genericEvent().fire("DynamicWorkCellLoaded");

    }
}

void RWSimulatorPlugin::initialize(){

    getRobWorkStudio()->genericEvent().add(
          boost::bind(&RWSimulatorPlugin::genericEventListener, this, _1), this);

}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(RWSimulatorPlugin);
#endif
