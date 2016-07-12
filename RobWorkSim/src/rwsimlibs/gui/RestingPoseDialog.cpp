#include "RestingPoseDialog.hpp"

#include <iostream>

#include <boost/foreach.hpp>

#include <rw/math/RPY.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Random.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/RigidBody.hpp>

#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rwsim/loaders/ScapePoseFormat.hpp>
#include <fstream>
#include <stdio.h>

#include "ui_RestingPoseDialog.h"

#include <QTime>
#include <QTimer>

using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::loaders;
using namespace rw::trajectory;

#define RW_DEBUGS( str ) //std::cout << str  << std::endl;

namespace {


}

RestingPoseDialog::RestingPoseDialog(const rw::kinematics::State& state,
                                    DynamicWorkCell *dwc,
                                    rw::proximity::CollisionDetector *detector,
                                    QWidget *parent):
    QDialog(parent),
    _defstate(state),
    _state(state),
    _dwc(dwc),
    _colDect(detector),
    _avgSimTime(4),
    _avgTime(4)
{
	RW_ASSERT( _dwc );
	_ui = new Ui::RestingPoseDialog();
    _ui->setupUi(this);

    connect(_ui->_saveBtn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_startBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_resetBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_stopBtn     ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_simulatorBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_scapeBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_ui->_updateRateSpin,SIGNAL(valueChanged(int)), this, SLOT(changedEvent()) );

    _timer = new QTimer( NULL );
    _timer->setInterval( _ui->_updateRateSpin->value() );
    connect( _timer, SIGNAL(timeout()), this, SLOT(changedEvent()) );
}

void RestingPoseDialog::initializeStart(){
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

    std::string engineId = _dwc->getEngineSettings().get<std::string>("Engine","ODE");
    if( !PhysicsEngine::Factory::hasEngineID(engineId) ){
        RW_WARN("Engine id: " << engineId << " not supported!");
        engineId = PhysicsEngine::Factory::getEngineIDs()[0];
    }

    for(int i=0;i<threads;i++){
        // create simulator
        RW_DEBUGS("sim " << i);
        PhysicsEngine::Ptr pengine = PhysicsEngine::Factory::makePhysicsEngine(engineId,_dwc);
        DynamicSimulator::Ptr sim = ownedPtr( new DynamicSimulator(_dwc, pengine) );
        RW_DEBUGS("Initialize simulator " << i);
        sim->init(state);
        _simulators.push_back( ownedPtr( new ThreadSimulator(sim,state) ) );
        RW_DEBUGS("Calc random cfg " << i);
        calcRandomCfg(state);
        _initStates.push_back(state);
    }

    _startTime = rw::common::TimerUtil::currentTimeMs();
    _lastTime = _startTime;
    RW_DEBUGS("init finished");

}

void RestingPoseDialog::btnPressed(){
    QObject *obj = sender();
    if( obj == _ui->_saveBtn1 ){

    	QString str = _ui->_savePath->text();
    	std::string sstr( str.toStdString() );
        rw::models::WorkCell &wc = *_dwc->getWorkcell();
        // TODO: open file dialog and save
        RW_ASSERT(_startPoses.size() == _resultPoses.size());
        TimedStatePath startPath, endPath;
        for(size_t i=0;i<_startPoses.size();i++){
            startPath.push_back(TimedState(i, _startPoses[i]));
        }
        for(size_t i=0;i<_resultPoses.size();i++){
            endPath.push_back(TimedState(i, _resultPoses[i]));
        }

        if( _ui->_asCommaBtn->isChecked() ){
            std::string textFile = sstr + ".txt";
            std::ofstream fout( textFile.c_str() );
            std::vector<RigidBody::Ptr> bodies = _dwc->findBodies<RigidBody>();
            fout << "Configuration of all rigid bodies, each line contain pos and orientation (EAA) for a simulation run."
                    << "That is both start and end configuration eg. b1_start, b1_end, b2_start, b2_end, b3_start, b3_end.... first comes the body names \n";
            BOOST_FOREACH(RigidBody::Ptr b, bodies){
                fout << b->getName() << " ; ";
            }
            fout << "\n";
            fout.precision(16);
            for(size_t i=0;i<_startPoses.size();i++){
                BOOST_FOREACH(RigidBody::Ptr b, bodies){
                    Transform3D<> t3d = b->getWTBody( _startPoses[i] );
                    EAA<> rot( t3d.R() );

                    fout << t3d.P()[0] << " ; " << t3d.P()[1] << " ; " << t3d.P()[2] << " ; ";
                    fout << rot[0] << " ; " << rot[1] << " ; " << rot[2] << " ; ";

                    t3d = b->getWTBody( _resultPoses[i] );
                    rot = EAA<>( t3d.R() );
                    fout << t3d.P()[0] << " ; " << t3d.P()[1] << " ; " << t3d.P()[2] << " ; ";
                    fout << rot[0] << " ; " << rot[1] << " ; " << rot[2] << " ; ";
                }
                fout << "\n";
            }
            fout.close();
        }

        rw::loaders::PathLoader::storeTimedStatePath( wc,
            startPath, sstr+"_start.rwplay");
        rw::loaders::PathLoader::storeTimedStatePath( wc,
                                   endPath, sstr+"_end.rwplay");
    } else if( obj == _ui->_startBtn ) {
        _ui->_startBtn->setDisabled(true);
        _ui->_stopBtn->setDisabled(false);
        _ui->_simulatorBtn->setDisabled(true);
        _ui->_resetBtn->setDisabled(true);
        bool isStart = _ui->_startBtn->text() == "Start";
        _ui->_startBtn->setText("Continue");
        // initialize
        if( isStart )
            initializeStart();

        _timer->start();

        Log::infoLog() << "Resting pose calculation started: " << std::endl
					<< "- Time         : " << QTime::currentTime().toString().toStdString() << std::endl
					<< "- Nr of tests  : " << _ui->_nrOfTestsSpin->value() << std::endl
					<< "- Nr of threads: " << _ui->_nrOfThreadsSpin->value() << std::endl;

    } else if( obj == _ui->_stopBtn ) {
        if(_nrOfTests<_ui->_nrOfTestsSpin->value())
            _ui->_startBtn->setDisabled(false);
        _ui->_stopBtn->setDisabled(true);
        _ui->_resetBtn->setDisabled(false);
        _timer->stop();
        BOOST_FOREACH(Ptr<ThreadSimulator> sim,  _simulators){
            if(sim->isRunning())
                sim->stop();
        }
        Log::infoLog() << "Resting pose calculation stopped: " << std::endl
					<< "- Time         : " << QTime::currentTime().toString().toStdString() << std::endl
					<< "- # tests done : " << _nrOfTests << std::endl;

    } else if( obj == _ui->_resetBtn ) {
        _ui->_startBtn->setDisabled(false);
        _ui->_startBtn->setText("Start");
        _ui->_simulatorBtn->setDisabled(false);
    } else if( obj == _ui->_simulatorBtn ) {
        // create simulator

    } else if( obj == _ui->_scapeBtn ) {
    	std::string str = _ui->_savePath->text().toStdString();
    	if( _ui->_saveMultipleCheck->isChecked() ){
    		for(size_t i=0;i<_resultPoses.size();i++){
    			std::stringstream sstr;
    			//sstr.setf(std::fixed(int))
    			char cstr[20];
    			sprintf(cstr,"%.6u",(unsigned int)i);
    			sstr << str << "_" << cstr;
    			ScapePoseFormat::savePoses(sstr.str(),_bodies,_resultPoses[i],
    									   _dwc->getWorkcell()->getName(),
    									   "Description");
    			Log::infoLog() << "Exported resting poses in Scape format, to multiple files "
    						<< StringUtil::quote(str) << "_XXXXXX" << std::endl;
    		}
    	} else {
			ScapePoseFormat::savePoses(str,_bodies,_resultPoses,
									   _dwc->getWorkcell()->getName(),
									   "Description");
			Log::infoLog() << "Exported resting poses in Scape format, to file "
						<< StringUtil::quote(str) << std::endl;
    	}

    } else  {

    }
}

void RestingPoseDialog::changedEvent(){
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
    }
}

void RestingPoseDialog::updateStatus(){
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

        // if simulator is in error then generate new configuration
        if(  sim->isInError() ){
            // recalc random start configurations and reset the simulator
            calcRandomCfg(state);
            _initStates[i] = state;
            sim->reset(state);
            //sim->start();
            return;
        }

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
            sim->reset(state);
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
            sim->reset(state);
            if( _nrOfTests>=_ui->_nrOfTestsSpin->value() )
                continue;
            sim->start();
        } else if( !sim->isRunning() && _nrOfTests<_ui->_nrOfTestsSpin->value()){
            // recalc random start configurations and reset the simulator
            calcRandomCfg(state);
            _initStates[i] = state;
            sim->reset(state);
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
        stateChanged(_state);

    if( _nrOfTests>=_ui->_nrOfTestsSpin->value() ){
        _ui->_stopBtn->click();
        return;
    }
}
void RestingPoseDialog::calcColFreeRandomCfg(rw::kinematics::State& state){
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
            bodies.push_back(body2);
        }
        calcRandomCfg(bodies, state);
        result.collidingFrames.clear();
        bodies.clear();
    }
    //std::cout << "-------- Col free collision END " << std::endl;

}

namespace {

    Rotation3D<> ranRotation3D(double maxAngle){
        EAA<> rot(Random::ran(-maxAngle, maxAngle), Random::ran(-maxAngle, maxAngle), Random::ran(-maxAngle, maxAngle));
        return rot.toRotation3D();
    }

    /**
     * @brief generate a random rotation around the point \b point
     * @param point [in] the point around which a random rotation is generated
     * @param maxAngle [in] the bound in radians
     * @return
     */
    Rotation3D<> ranRotation3D(const rw::math::Rotation3D<>& point, double maxAngle){
        EAA<> rot(Random::ran(-maxAngle, maxAngle), Random::ran(-maxAngle, maxAngle), Random::ran(-maxAngle, maxAngle));
        return point*rot.toRotation3D();
    }

    Rotation3D<> ranNormalDistRotation3D(double sigma_a){
        EAA<> rot(Random::ranNormalDist(0,sigma_a), Random::ranNormalDist(0,sigma_a), Random::ranNormalDist(0,sigma_a));
        return rot.toRotation3D();
    }

    Rotation3D<> ranNormalDistRotation3D(const rw::math::Rotation3D<>& point, double sigma_a){
        EAA<> rot(Random::ranNormalDist(0,sigma_a), Random::ranNormalDist(0,sigma_a), Random::ranNormalDist(0,sigma_a));
        return point*rot.toRotation3D();
    }

    Transform3D<> ranTransform3D(const rw::math::Transform3D<>& point, double maxPos,  double maxAngle){
        Vector3D<> pos(Random::ran(-maxPos,maxPos), Random::ran(-maxPos,maxPos), Random::ran(-maxPos,maxPos));
        EAA<> rot(Random::ran(-maxAngle, maxAngle), Random::ran(-maxAngle, maxAngle), Random::ran(-maxAngle, maxAngle));
        return point*Transform3D<>(pos,rot);
    }

    Transform3D<> ranNormalDistTransform3D(const rw::math::Transform3D<>& point, double sigma_p,  double sigma_a){
        Vector3D<> pos(Random::ranNormalDist(0,sigma_p), Random::ranNormalDist(0,sigma_p), Random::ranNormalDist(0,sigma_p));
        EAA<> rot(Random::ranNormalDist(0,sigma_a), Random::ranNormalDist(0,sigma_a), Random::ranNormalDist(0,sigma_a));
        return point*Transform3D<>(pos,rot);
    }

}

void RestingPoseDialog::calcRandomCfg(std::vector<RigidBody::Ptr> &bodies, rw::kinematics::State& state){
    Rotation3D<> rot;
    Vector3D<> pos;
    if( _ui->_rotationTabs->currentIndex()==0 ){
        // calculate a random orientation in SO3
        rot = Math::ranRotation3D<double>();
    } else if( _ui->_rotationTabs->currentIndex()==1 ){
        // get direction and angle
        double angle = _ui->_angleSpin->value() * Deg2Rad;
        if( _ui->_normalDistRot->isChecked() ){
            rot = ranNormalDistRotation3D(angle);
        } else {
            rot = ranRotation3D(angle);
        }
    } else {
        const double lowR = Deg2Rad * ( _ui->_lowRollSpin->value() );
        const double highR = Deg2Rad * ( _ui->_highRollSpin->value() );
        const double lowP = Deg2Rad * ( _ui->_lowPitchSpin->value() );
        const double highP = Deg2Rad * ( _ui->_highPitchSpin->value() );
        const double lowY = Deg2Rad * ( _ui->_lowYawSpin->value() );
        const double highY = Deg2Rad * ( _ui->_highYawSpin->value() );

        double roll = Random::ran(lowR, highR);
        double pitch = Random::ran(lowP, highP);
        double yaw = Random::ran(lowY, highY);
        rot = RPY<>(roll,pitch,yaw).toRotation3D();
    }

    pos[0] = Random::ran(_ui->_xLimit->value(), std::max(_ui->_xLimit->value(), _ui->_xLimit_2->value()));
    pos[1] = Random::ran(_ui->_yLimit->value(), std::max(_ui->_yLimit->value(), _ui->_yLimit_2->value()));
    pos[2] = Random::ran(_ui->_zLimit->value(), std::max(_ui->_zLimit->value(), _ui->_zLimit_2->value()));

    BOOST_FOREACH(RigidBody::Ptr rbody, bodies){
        Transform3D<> t3d = Kinematics::worldTframe(rbody->getMovableFrame(), _defstate);
        Transform3D<> nt3d = t3d * Transform3D<>(pos, rot);
        rbody->getMovableFrame()->setTransform(nt3d,state);
    }

}

void RestingPoseDialog::calcRandomCfg(rw::kinematics::State& state){
    if( _ui->_colFreeStart->isChecked() ){
        calcColFreeRandomCfg(state);
    } else {
        calcRandomCfg(_bodies, state);
    }
}
