#include "GraspSelectionDialog.hpp"

#include <iostream>

#include <QFileDialog>

#include <boost/foreach.hpp>

#include <rw/math/RPY.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rwsim/dynamics/RigidBody.hpp>

#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/simulator/PhysicsEngineFactory.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/graspplanning/GraspTable.hpp>
#include <rwsim/loaders/ScapePoseFormat.hpp>
#include <rwsim/sensor/TactileArraySensor.hpp>
#include <stdio.h>

#include "ui_GraspSelectionDialog.h"

using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsim::sensor;
//using namespace rwsim::control;
using namespace rwsim::util;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::loaders;
using namespace rw::trajectory;
using namespace rw::graspplanning;

#define RW_DEBUGS( str ) //std::cout << str  << std::endl;

namespace {

namespace {

	std::string openFile(std::string& previousOpenDirectory, QWidget *parent){
	    QString selectedFilter;

	    const QString dir(previousOpenDirectory.c_str());

	    QString filename = QFileDialog::getOpenFileName(
	    	parent,
	        "Open State path", // Title
	        dir, // Directory
	        " RW state path ( *.rwplay )"
	        " \n All ( *.* )",
	        &selectedFilter);

	    std::string file = filename.toStdString();
	    if (!file.empty())
	        previousOpenDirectory = rw::common::StringUtil::getDirectoryName(file);

	    return file;
	}
}

}

GraspSelectionDialog::GraspSelectionDialog(const rw::kinematics::State& state,
                                    DynamicWorkCell *dwc,
                                    rw::proximity::CollisionDetector *detector,
                                    QWidget *parent):
    QDialog(parent),
    _defstate(state),
    _state(state),
    _dwc(dwc),
    _colDect(detector),
    _avgSimTime(4),
    _avgTime(4),
    _kdtree(NULL)
{
	RW_ASSERT( _dwc );
	_ui = new Ui::GraspSelectionDialog();
    _ui->setupUi(this);

    connect(_ui->_saveBtn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_startBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_resetBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_stopBtn     ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_simulatorBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_scapeBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_graspTableLoadBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_ui->_searchBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_buildTreeBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_ui->_updateRateSpin,SIGNAL(valueChanged(int)), this, SLOT(changedEvent()) );
    _ui->_graspTableSlider->setRange(0, 0);
    connect(_ui->_graspTableSlider,SIGNAL(valueChanged(int)), this, SLOT(changedEvent()) );

    connect(_ui->_graspTableSlider,SIGNAL(valueChanged(int)),_ui->_graspSpin,SLOT(setValue(int)));



    _timer = new QTimer( NULL );
    _timer->setInterval( _ui->_updateRateSpin->value() );
    connect( _timer, SIGNAL(timeout()), this, SLOT(changedEvent()) );
}

void GraspSelectionDialog::initializeStart(){
    _simulators.clear();
    _initStates.clear();
    _bodies.clear();
    _frameToBody.clear();
    _nrOfTests = 0;
    State state = _defstate;
    int threads = _ui->_nrOfThreadsSpin->value();
    _simStartTimes.resize(threads, 0);
    RW_DEBUGS("threads: " << threads);

    _bodies = _dwc->findBodies<RigidBody>();
    BOOST_FOREACH(RigidBody::Ptr rbody, _bodies){
        _frameToBody[*rbody->getMovableFrame()] = rbody;
    }

    for(int i=0;i<threads;i++){
        // create simulator
        RW_DEBUGS("sim " << i);
        PhysicsEngine::Ptr pengine = PhysicsEngineFactory::makePhysicsEngine("ODE",_dwc);
        DynamicSimulator::Ptr dsim = ownedPtr( new DynamicSimulator(_dwc, pengine));
        RW_DEBUGS("Initialize simulator " << i);
        dsim->init(state);
        _simulators.push_back( ownedPtr( new ThreadSimulator(dsim, state) ) );
        RW_DEBUGS("Calc random cfg " << i);
        calcRandomCfg(state);
        _initStates.push_back(state);
    }

    _startTime = rw::common::TimerUtil::currentTimeMs();
    _lastTime = _startTime;
    RW_DEBUGS("init finished");

}



using namespace rwlibs::algorithms;
using namespace rw::graspplanning;


void GraspSelectionDialog::btnPressed(){
    QObject *obj = sender();
    if( obj == _ui->_graspTableLoadBtn ){
    	std::string dir;
    	std::string filename = openFile(dir, this);
    	if(filename.empty())
    		return;
    	try{
    	    _gtable = GraspTable::load(filename);
    		Log::infoLog() << "Table size: " << _gtable->size() << "\n";
    	} catch(const Exception&) {
    		_gtable = NULL;
    		Log::errorLog() << "Load failed!\n";
    		return;
    	}
    	Log::infoLog() << "Trying to find device: " << _gtable->getHandName() << std::endl;
    	_dev = _dwc->getWorkcell()->findDevice(_gtable->getHandName()).get();
    	Log::infoLog() << "Trying to find frame: " << _gtable->getObjectName() << std::endl;
    	Frame *obj = _dwc->getWorkcell()->findFrame(_gtable->getObjectName());


    	if( _dev==NULL ){
    		Log::errorLog() << "Hand: "<< _gtable->getHandName() << " not found in workcell!\n";
    		return;
    	}
    	if( obj==NULL || dynamic_cast<MovableFrame*>(obj)==NULL){
    		Log::errorLog() << "Object: "<< _gtable->getObjectName() << " not found in workcell!\n";
    		//return;
    		_object = NULL;
    	} else {
    	    _object = dynamic_cast<MovableFrame*>(obj);
    	}
		if (_gtable->size() == 0) {
		    _ui->_graspTableSlider->setRange(0, 0);
		    _ui->_graspTableSlider->setEnabled(false);
		} else {
		    _ui->_graspTableSlider->setRange(0, (int)(_gtable->size()-1));
		    _ui->_graspTableSlider->setEnabled(true);
		}
    	Frame* fbase = _dev->getBase()->getParent(_state);
    	while( !dynamic_cast<MovableFrame*>(fbase) ){
    		fbase = fbase->getParent(_state);
    		if(fbase == NULL){
    			Log::errorLog() << "Could not find movable hand base\n";
    			break;
    		}
    	}
    	if(fbase!=NULL)
    	    _handBase = dynamic_cast<MovableFrame*>(fbase);
    	else
    	    _handBase = NULL;
    } else if( obj == _ui->_startBtn ) {

    } else if( obj == _ui->_searchBtn)  {
    	// TODO:
    	//std::string str = _searchEdit->text().toStdString();

    	//scanf("")

    	Q q(6);
    	q(0) = 1;q(1) = 1;q(2) = 1;q(3) = -180*Deg2Rad; q(4) = 0;q(5) = 0;
    	GraspTable::GraspData *data = _kdtree->nnSearch(q).value;
    	setGraspState(*data,_state);
    	stateChanged(_state);

    } else if ( obj == _ui->_buildTreeBtn ){
    	Log::infoLog() << "Building tree!\n";
    	// build the KD-tree with the database
    	Q key(6);
    	SearchTree::KDNode node(key, NULL);
    	_nodes.resize(_gtable->size(), node);
    	for(size_t i=0;i<_gtable->size();i++){
    		GraspTable::GraspData &data = _gtable->getData()[i];

    		Transform3D<> oTh = inverse(data.op.toTransform3D())*data.hp.toTransform3D();
    		RPY<> rpy( oTh.R() );
    		key(0) = data.quality(0);
    		key(1) = data.quality(1);
    		key(2) = data.quality(2);
    		key(3) = rpy(0);
    		key(4) = rpy(1);
    		key(5) = rpy(2);
    		_nodes[i].key = key;
    		_nodes[i].value = &data;
    	}

    	_kdtree = SearchTree::buildTree(_nodes);
    	Log::infoLog() << "Building tree finished!\n";
    }
}

void GraspSelectionDialog::setGraspState(GraspTable::GraspData& data, rw::kinematics::State &state){
	std::cout << data.cq << std::endl;
    _dev->setQ(data.cq,state);
	Transform3D<> hbTo = data.hp.toTransform3D();
	Transform3D<> wTo = data.op.toTransform3D();

	if(_object)
	    _object->setTransform( wTo, state );
	if(_handBase)
	    _handBase->setTransform( wTo*inverse(hbTo), state);

	TactileArraySensor *tsensor;
	std::size_t j=0;
	for(std::size_t i=0;i<_dwc->getSensors().size();i++){
		if( (tsensor = dynamic_cast<TactileArraySensor*>(_dwc->getSensors()[i].get() ) ) ){
			if(j>=data._tactiledata.size())
				continue;
			std::cout << "tsensorname" << j <<": "  << tsensor->getName() << std::endl;
			tsensor->getTactileArrayModel()->setTexelData(data._tactiledata[j], state);
			j++;
		}
	}
	{
		std::stringstream sstr; sstr << "Quality: " << data.quality;
		_ui->_qualityLbl->setText(sstr.str().c_str());
	}
	{
		std::stringstream sstr; sstr << "Nr contacts: " << data.grasp.contacts.size();
		_ui->_contactLbl->setText(sstr.str().c_str());
	}
	Transform3D<> oTh = inverse(data.op.toTransform3D())*data.hp.toTransform3D();
	RPY<> rpy( oTh.R() );
	{
		std::stringstream sstr; sstr << "Pose: " << oTh.P() <<" "<< rpy;
		_ui->_poseLbl->setText(sstr.str().c_str());
	}
}

void GraspSelectionDialog::changedEvent(){
    QObject *obj = sender();
    if( obj == _timer ){
        // update stuff
        updateStatus();
    } else if( obj == _ui->_updateRateSpin ){
        _timer->setInterval( _ui->_updateRateSpin->value() );
    } else if( obj == _ui->_forceUpdateCheck ) {
        if( !_ui->_forceUpdateCheck->isChecked() ){
            _timer->setInterval( 100 );
        } else {
            _timer->setInterval( _ui->_updateRateSpin->value() );
        }
    } else if ( obj == _ui->_graspTableSlider ){
    	int gidx = _ui->_graspTableSlider->value();
    	int gsize = (int)_gtable->getData().size();

    	if(gidx>=gsize)
    		return;

    	GraspTable::GraspData &data = _gtable->getData()[gidx];
    	setGraspState(data,_state);
    	stateChanged(_state);

    }
}

void GraspSelectionDialog::updateStatus(){
    if( _simulators.size()<1 )
        return;
    State state;
    int nrOfTestsOld = _nrOfTests;
    for(size_t i=0;i<_simulators.size(); i++){
        Ptr<ThreadSimulator> sim = _simulators[i];

        if(!sim->isRunning() && _nrOfTests>=_ui->_nrOfTestsSpin->value()){
            continue;
        }

        state = sim->getState();

        // get stop criterias
        double lVelThres = _ui->_linVelSpin->value();
        double aVelThres = _ui->_angVelSpin->value();

        // check the velocity of all the bodies
        bool allBelowThres = true;
        Vector3D<> avgLVel, avgAVel;
        BOOST_FOREACH(RigidBody::Ptr rbody, _bodies){
            //RW_DEBUGS("rbody: " << rbody->getMovableFrame().getName() );
            // get velocity of rbody
            // if above threshold then break and continue
            Vector3D<> avel = rbody->getAngVel(state);
            Vector3D<> lvel = rbody->getLinVel(state);
            avgAVel += avel;
            avgLVel += lvel;
            if (lvel.norm2()>lVelThres || avel.norm2()>aVelThres){
                allBelowThres = false;
                break;
            }
        }
        avgLVel = avgLVel/_bodies.size();
        avgAVel = avgAVel/_bodies.size();
        // get dt and make a simulation step
        double time = sim->getTime();
        // if requestet add the state to the state trajectory
        //if( _recordStatePath->checked() )
        //    _statePath.push_back( Timed<State>(time+_simStartTime, state) );

        if(!allBelowThres || time<_ui->_minTimeValidSpin->value())
            _lastBelowThresUpdate = time;


        if(i==0)
            _state = state;


        if( time-_lastBelowThresUpdate >  _ui->_minRestTimeSpin->value() ){
            // one simulation has finished...
            sim->stop();

            _avgSimTime.addSample(time);
            _totalSimTime += time;
            _nrOfTests++;
            _resultPoses.push_back(state);
            _startPoses.push_back(_initStates[i]);
            restingPoseEvent(_initStates[i],state);


            // recalc random start configurations and reset the simulator
            calcRandomCfg(state);
            _initStates[i] = state;
            sim->setState(state);
            _simStartTimes[i] = time;

            RW_DEBUGS(_nrOfTests<<">="<<_ui->_nrOfTestsSpin->value());
            if( _nrOfTests>=_ui->_nrOfTestsSpin->value() )
                continue;
            RW_DEBUGS("Start sim again");
            sim->start();
        } else if( time>_ui->_maxRunningTimeSpin->value() ){
            sim->stop();
            // recalc random start configurations and reset the simulator
            calcRandomCfg(state);
            _initStates[i] = state;
            sim->setState(state);
            if( _nrOfTests>=_ui->_nrOfTestsSpin->value() )
                continue;
            sim->start();
        } else if( !sim->isRunning() && _nrOfTests<_ui->_nrOfTestsSpin->value()){
            // recalc random start configurations and reset the simulator
            calcRandomCfg(state);
            _initStates[i] = state;
            sim->setState(state);
            sim->start();
        }
    }
    double avgTestTime = 0;
    double avgSimTimePerTest = 0;
    double simTimeToReal = 0;
    if(_nrOfTests>0 && nrOfTestsOld!=_nrOfTests){
        long realtime = rw::common::TimerUtil::currentTimeMs();
        avgTestTime = (realtime-_lastTime)/(double)(_nrOfTests-nrOfTestsOld);
        _lastTime = realtime;
        _avgTime.addSample(avgTestTime);
        avgTestTime = _avgTime.getAverage();

        int progress = (_nrOfTests*100)/_ui->_nrOfTestsSpin->value();
        _ui->_simProgress->setValue( progress );

        //avgSimTimePerTest = _totalSimTime/(double)_nrOfTests;
        avgSimTimePerTest = _avgSimTime.getAverage();

        simTimeToReal = avgSimTimePerTest*1000.0/avgTestTime;

        double timeLeft = ((_ui->_nrOfTestsSpin->value()-_nrOfTests)*avgTestTime)/1000.0;

        // update the time label
        {
            std::stringstream s; s << avgTestTime/1000.0 << "s";
            _ui->_avgTimePerTestLbl->setText(s.str().c_str());
        }
        {
            std::stringstream s; s << timeLeft << "s";
            _ui->_estimatedTimeLeftLbl->setText(s.str().c_str());
        }
        {
            std::stringstream s; s << timeLeft << "s";
            _ui->_estimatedTimeLeftLbl->setText(s.str().c_str());
        }
        {
            std::stringstream s; s << simTimeToReal;
            _ui->_simSpeedLbl->setText(s.str().c_str());
        }
        {
            std::stringstream s; s << avgSimTimePerTest << "s";
            _ui->_avgSimTimeLbl->setText(s.str().c_str());
        }
        {
            std::stringstream s; s << _nrOfTests << "/" << _ui->_nrOfTestsSpin->value();
            _ui->_testsLeftLbl->setText(s.str().c_str());
        }
    }

    // and last signal that workcell state has changed if user request it
    if(_ui->_forceUpdateCheck->isChecked())
        stateChanged(state);

    if( _nrOfTests>=_ui->_nrOfTestsSpin->value() ){
        _ui->_stopBtn->click();
        return;
    }
}
void GraspSelectionDialog::calcColFreeRandomCfg(rw::kinematics::State& state){
    //std::cout << "-------- Col free collision: " << std::endl;
    // first calculate a random state
    calcRandomCfg(_bodies, state);
    int nrOfTries=0;
    CollisionDetector::QueryResult result;
    std::vector<RigidBody::Ptr> bodies;
    while( _colDect->inCollision(state, &result, false) ){
        nrOfTries++;
        BOOST_FOREACH(rw::kinematics::FramePair pair, result.collidingFrames){
            // generate new collision free configuration between
            RigidBody::Ptr body1 = _frameToBody[*pair.first];
            RigidBody::Ptr body2 = _frameToBody[*pair.second];
            // calc new configuration
            bodies.push_back(body1);
            //bodies.push_back(body2);
        }
        calcRandomCfg(bodies, state);
        result.collidingFrames.clear();
        bodies.clear();
    }
    //std::cout << "-------- Col free collision END " << std::endl;

}

void GraspSelectionDialog::calcRandomCfg(std::vector<RigidBody::Ptr> &bodies, rw::kinematics::State& state){
    const double lowR = Deg2Rad * ( _ui->_lowRollSpin->value() );
    const double highR = Deg2Rad * ( _ui->_highRollSpin->value() );
    const double lowP = Deg2Rad * ( _ui->_lowPitchSpin->value() );
    const double highP = Deg2Rad * ( _ui->_highPitchSpin->value() );
    const double lowY = Deg2Rad * ( _ui->_lowYawSpin->value() );
    const double highY = Deg2Rad * ( _ui->_highYawSpin->value() );


    BOOST_FOREACH(RigidBody::Ptr rbody, bodies){
        double roll = Math::ran(lowR, highR);
        double pitch = Math::ran(lowP, highP);
        double yaw = Math::ran(lowY, highY);
        Transform3D<> t3d = Kinematics::worldTframe(rbody->getMovableFrame(), _defstate);
        Transform3D<> nt3d = t3d;
        nt3d.R() = t3d.R()*Rotation3D<>( RPY<>(roll,pitch,yaw).toRotation3D() );
        rbody->getMovableFrame()->setTransform( nt3d, state);
    }
}

void GraspSelectionDialog::calcRandomCfg(rw::kinematics::State& state){
    if( _ui->_colFreeStart->isChecked() ){
        calcColFreeRandomCfg(state);
    } else {
        calcRandomCfg(_bodies, state);
    }
}
