#include "GraspRestingPoseDialog.hpp"

#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#include <rw/math/RPY.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <dynamics/RigidBody.hpp>
#include <rw/math/MetricUtil.hpp>
#include <simulator/PhysicsEngineFactory.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/Proximity.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <loaders/ScapePoseFormat.hpp>
#include <rw/models/Accessor.hpp>
#include <sensors/TactileArraySensor.hpp>

#include <planning/WrenchMeasure3D.hpp>
#include <planning/CMDistCCPMeasure3D.hpp>

#include <geometry/GiftWrapHull3D.hpp>


//#include "boost/filesystem.hpp"   // includes all needed Boost.Filesystem declarations
//#include <iostream>               // for std::cout
//using namespace boost::filesystem;          // for ease of tutorial presentation;
                                  //  a namespace alias is preferred practice in real code
using namespace dynamics;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::loaders;
using namespace rw::trajectory;
using namespace rwlibs::simulation;
using namespace rw::models;

using namespace boost::numeric::ublas;

#define RW_DEBUGS( str ) std::cout << str  << std::endl;

namespace {


}

namespace {

	class TimeLabel: public QLabel {
		TimeLabel(){}

		void setTimeInMs();
		void setTimeInSec();
		void setTimeInMin();


	};

    void saveRestingPose(std::string& desc,
                         std::vector<GraspRestingPoseDialog::TactileSensorData >& datas,
                         rw::math::Q& qualities,
                         bool isStable,
                         int preshapeId,
                         rw::math::Q& preq,
                         std::vector<rw::math::Q>& handq,
                         rw::math::Vector3D<>& approach,
                         std::ostream& file,
                         const std::string& imgfile)
    {

        file << "# ********* " << desc << "\n\n";

        // save stability
        file << "# stable grasp \n";
        if(isStable) file << "stable\n\n"; else file << "nonstable\n\n";

        // Grasp Qualities
        file << "# Quality of grasp \n";
        for(size_t i=0;i<qualities.size(); i++)
            file << qualities[i] << " ";
        file << "\n\n";

        // preshape ID
        file << "# Preshape id \n";
        file << preshapeId << "\n\n";

        // preshape of hand
        file << "# preshape configuration \n";
        for(size_t i=0;i<preq.size(); i++)
            if(i!=5)
                file << preq[i]*Rad2Deg << " ";
        file << "\n\n";

        // resting configuration of hand
        file << "# grasp contact configuration \n";
        for(size_t i=0;i<handq.back().size(); i++)
            if(i!=5)
                file << handq.back()[i]*Rad2Deg << " ";
        file << "\n\n";

        // approach vector
        file << "# approach vector relative to plane \n";
        file << approach(0) << " " << approach[1] << " " << approach[2] << "\n\n";
        file << "# img file postfix \n";
        file << imgfile << "\n";

        file << "#\n number of tactile sensor data readings\n";
        file << datas.size();

        file << "\n\n# tactile sensor data \n";

        // save tactile data
        int idx=0;
        BOOST_FOREACH(GraspRestingPoseDialog::TactileSensorData& tdata, datas){

            file << "# Q " << idx << "/" << datas.size() << "\n";
            for(size_t i=0;i<handq.back().size(); i++)
                if(i!=5)
                    file << handq[idx][i]*Rad2Deg << " ";
            file << "\n\n";

            file << "# Tdata " << idx << "/" << datas.size() << "\n";
			BOOST_FOREACH(matrix<float>& data, tdata){
				for(size_t x=0;x<data.size1(); x++){
					for(size_t y=0;y<data.size2(); y++){
						file << data(x,y) << " ";
					}
					file << "\n";
				}
				file << "\n\n";
			}
			idx++;
        }
    }


    void saveRestingPose(std::string& desc,
					 std::vector<GraspRestingPoseDialog::TactileSensorData >& datas,
                     rw::math::Q& qualities,
                     bool isStable,
                     int preshapeId,
                     rw::math::Q& preq,
                     std::vector<rw::math::Q>& handq,
                     rw::math::Vector3D<>& approach,
                     const std::string& filename,
                     const std::string& imgfile)
    {
        //path my_path( filename );

        //if ( !exists( my_path.directory_string() ) )
        //    create_directory( my_path.directory_string() );

        //std::ofstream file( my_path.string().c_str() );
        std::ofstream file( filename.c_str() );
        if(!file.is_open())
            RW_THROW("CANNOT OPEN FILE! "<< filename);
        saveRestingPose(desc, datas, qualities, isStable, preshapeId, preq, handq, approach, file,imgfile);
        file.close();
    }
/*

        class MySampler : public QSampler
        {
        public:
            MySampler(const std::vector<Q>& qs) :
                _qs(qs.rbegin(), qs.rend())
            {}

        private:
            Q doSample()
            {
                if (_qs.empty()) return Q();
                else {
                    int idx = Math::ranI(0,_qs.size());
                    return _qs[idx];
                }
            }

            bool doEmpty() const { return _qs.empty(); }

        private:
            std::vector<Q> _qs;
        };

*/

}


GraspRestingPoseDialog::GraspRestingPoseDialog(const rw::kinematics::State& state,
                                    dynamics::DynamicWorkcell *dwc,
                                    rw::proximity::CollisionDetector *detector,
                                    QWidget *parent):
    QDialog(parent),
    _defstate(state),
    _state(state),
    _dwc(dwc),
    _colDect(detector),
    _avgSimTime(4),
    _avgTime(4),
    _handBase(NULL),
    _exitHard(false),
_graspNotStable(false)
{
	RW_ASSERT( _dwc );
    setupUi(this);

    Math::seed( TimerUtil::currentTimeMs() );

    RW_DEBUGS("- Setting devices ");
    std::vector<DynamicDevice*> devices = _dwc->getDynamicDevices();
    BOOST_FOREACH(DynamicDevice* device, devices){
        if(dynamic_cast<RigidDevice*>(device)){
            rw::models::Device *dev = &device->getModel();
            RW_ASSERT(dev);
            RW_DEBUGS("-- Dev name: " << dev->getName() );
            _deviceBox->addItem(dev->getName().c_str());
        }
    }

    RW_DEBUGS("- Setting objects ");
    BOOST_FOREACH(Body *body, _dwc->getBodies() ){
        Frame *obj = &body->getBodyFrame();
        if(obj==NULL)
            continue;
        if( dynamic_cast<const MovableFrame*>(obj) ){
            _objectBox->addItem(obj->getName().c_str());
        }
    }

    _preshapeStratBox->addItem("Parallel");
    _preshapeStratBox->addItem("Spherical");
    _preshapeStratBox->addItem("Multiple10");
    _preshapeStratBox->addItem("CylStanding");


    _graspPolicyBox->addItem("SimpleClose");

    RW_DEBUGS("- Setting connections ");
    connect(_saveBtn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_startBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_resetBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_stopBtn     ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_simulatorBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_scapeBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_updateRateSpin,SIGNAL(valueChanged(int)), this, SLOT(changedEvent()) );

    RW_DEBUGS("- Setting timer ");
    _timer = new QTimer( NULL );
    _timer->setInterval( _updateRateSpin->value() );
    connect( _timer, SIGNAL(timeout()), this, SLOT(changedEvent()) );
}


   #include<boost/filesystem/operations.hpp>
   #include<iostream>
   #include<string>
   namespace bfs=boost::filesystem;


void GraspRestingPoseDialog::initializeStart(){
    _simulators.clear();
    _initStates.clear();
    _bodies.clear();
    _frameToBody.clear();
    _nrOfTests = 0;
    _preshapes.clear();
    _controllers.clear();
    _targetQ.clear();
    _nextTimeUpdate.clear();
    _nrOfTestsOld = 0;
    _fingersInContact.clear();
    _currentPreshapeIDX.clear();;

    // if the designated directory for saving the output does not exist, then create it
    bfs::create_directory( bfs::path(_savePath->text().toStdString()) );

    // write configuration to configfile
    QString str = _savePath->text();
    std::string pathPre( str.toStdString() );
    std::stringstream filename;
    filename << pathPre << "/config_file.txt";

    std::ofstream file( filename.str().c_str() );
    if(!file.is_open())
        RW_THROW("CANNOT OPEN FILE! "<< filename);

    State state = _defstate;
    int threads = _nrOfThreadsSpin->value();
    _simStartTimes.resize(threads, 0);
    RW_DEBUGS("threads: " << threads);
    file << "Nr of threads used: " << threads << std::endl;

    RW_DEBUGS("- Getting object!");
    std::string objName = _objectBox->currentText().toStdString();
    BOOST_FOREACH(Body* body, _dwc->getBodies()){
        if(RigidBody* rbody = dynamic_cast<RigidBody*>(body)){
            if(rbody->getBodyFrame().getName()==objName){
                _bodies.push_back(rbody);
                _body = rbody;
                _frameToBody[rbody->getMovableFrame()] = rbody;
                _object = &rbody->getMovableFrame();

                std::string geofilename = Accessor::collisionModelInfo().get(*_object)[0].getId();
                file << "Object:\n"
                	 << "- name: " << objName << "\n"
                     << "- geometryfile: " << geofilename << "\n";

                file << "Dynamic object:\n";
                _body->getInfo().print(file);
                break;
            }
        }
    }
    if(_bodies.size()==0){
        RW_THROW("You must choose a valid object to grasp!");
    }

    RW_DEBUGS("- Getting device!");
    std::string devName = _deviceBox->currentText().toStdString();
    DynamicDevice *dev = _dwc->findDevice(devName);
    _hand = dynamic_cast<RigidDevice*>(dev);
    file << "Hand device name: " << devName << std::endl;
    RW_ASSERT(_hand);
    // Now discover the movable handbase...
    Frame *parent = _hand->getModel().getBase();
    while(parent!=NULL){
        if( dynamic_cast<MovableFrame*>(parent) ){
            _handBase = dynamic_cast<MovableFrame*>(parent);
            break;
        }
        parent = parent->getParent();
    }
    RW_ASSERT(parent);


    RW_DEBUGS("- Getting preshapes!")
    // TODO: for now we only have one preshape
    Q preQ = _hand->getModel().getQ(state);
    Q tQ = preQ;
    if( _preshapeStratBox->currentText()=="Parallel"){
        preQ(0) = -1.5; preQ(1) = -1.5;
        preQ(2) = 1.571; preQ(5) = 1.571;
        preQ(3) = -1.5; preQ(4) = 0;
        preQ(6) = -1.5; preQ(7) = 0;

        tQ(0) = -1.5; tQ(1) = -1.5;
        tQ(2) = 1.571; tQ(5) = 1.571;
        tQ(3) = 0.13; tQ(4) = 0.6;
        tQ(6) = 0.13; tQ(7) = 0.6;
        _preshapes.push_back( preQ );
        _targetQ.push_back( tQ );
    } else if(_preshapeStratBox->currentText()=="Spherical"){
        preQ(0) = -1.5; preQ(1) = 0;
        preQ(2) = 0.785; preQ(5) = 0.785;
        preQ(3) = -1.5; preQ(4) = 0;
        preQ(6) = -1.5; preQ(7) = 0;

        tQ(0) = 0.13; tQ(1) = 0.6;
        tQ(2) = 0.785; tQ(5) = 0.785;
        tQ(3) = 0.13; tQ(4) = 0.6;
        tQ(6) = 0.13; tQ(7) = 0.6;
        _preshapes.push_back( preQ );
        _targetQ.push_back( tQ );
    }else if(_preshapeStratBox->currentText()=="CylStanding"){
        preQ(0) = -1.5; preQ(1) = 0;
        preQ(2) = 0; preQ(5) = 0;
        preQ(3) = -1.5; preQ(4) = 0;
        preQ(6) = -1.5; preQ(7) = 0;

        tQ(0) = 0.13; tQ(1) = 0.6;
        tQ(2) = 0; tQ(5) = 0;
        tQ(3) = 0.13; tQ(4) = 0.6;
        tQ(6) = 0.13; tQ(7) = 0.6;
        _preshapes.push_back( preQ );
        _targetQ.push_back( tQ );
    } else if(_preshapeStratBox->currentText()=="Multiple10"){
        for(int i=0;i<10;i++){
            preQ(0) = -1.5; preQ(1) = 0;
            preQ(2) = Pi/20*i; preQ(5) = Pi/20*i;
            preQ(3) = -1.5; preQ(4) = 0;
            preQ(6) = -1.5; preQ(7) = 0;

            tQ(0) = 0.13; tQ(1) = 0.6;
            tQ(2) = Pi/20*i; tQ(5) = Pi/20*i;
            tQ(3) = 0.13; tQ(4) = 0.6;
            tQ(6) = 0.13; tQ(7) = 0.6;

            _preshapes.push_back( preQ );
            _targetQ.push_back( tQ );
        }
    }

    _hand->getModel().setQ(preQ, _defstate);
    file << "Preshape configuration type: " << _preshapeStratBox->currentText().toStdString() << "\n";
    file << "preshapes:  \n";
    BOOST_FOREACH(Q pq, _preshapes){
    	file << "- " << pq << "\n";
    }
    file << "\nTargets: \n";
    BOOST_FOREACH(Q pq, _targetQ){
    	file << "- " << pq << "\n";
    }

    file << "\nTime between tactile readings: " << _updateRateSpin->value() << "ms\n";

    RW_DEBUGS("- Creating simulators: ");
    for(int i=0;i<threads;i++){
        state = _defstate;
        // create simulator
        RW_DEBUGS("-- sim nr " << i);
        Simulator *sim = PhysicsEngineFactory::newPhysicsEngine("ODE",_dwc);

        RW_DEBUGS("-- Initialize simulator " << i);
        sim->initPhysics(state);

        // create a controller
        PDControllerPtr pdctrl = ownedPtr(new PDController(_hand, state));
        sim->addController(pdctrl);
        _controllers.push_back(pdctrl);
        pdctrl->setTargetPos(_targetQ[0] );

        // add body sensor
        _bodySensor = ownedPtr( new BodyContactSensor("BodySensor", _object) );
        sim->addSensor( _bodySensor );

        ThreadSimulator *tsim = new ThreadSimulator(sim,state);
        CallBackFunctor *func = new CallBackFunctor(i, this);
        _functors.push_back( ownedPtr(func) );
        ThreadSimulator::StepCallback cb( boost::bind(&CallBackFunctor::stepCallBack, func, _1) );

        tsim->setStepCallBack( cb );
        _simulators.push_back( ownedPtr( tsim ) );
        tsim->setPeriodMs(0);
        RW_DEBUGS("-- Calc random cfg " << i);
        calcRandomCfg(state);
        _initStates.push_back(state);

        _nextTimeUpdate.push_back( _updateRateSpin->value()/1000.0 );


        _currentPreshapeIDX.push_back(i);
        _fingersInContact.push_back(false);
        _tactiledatas.push_back(std::vector< TactileSensorData >());
        _handconfigs.push_back( std::vector< rw::math::Q >()) ;
        tsim->start();
    }

    _startTime = rw::common::TimerUtil::currentTimeMs();
    _lastTime = _startTime;
    RW_DEBUGS("init finished");
    file.close();
}

void GraspRestingPoseDialog::startAuto(){
    _exitHard = true;
    _startBtn->click();

}

void GraspRestingPoseDialog::stepCallBack(int i, const rw::kinematics::State& state){
    ThreadSimulatorPtr tsim = _simulators[i];

    SimulatorPtr sim = tsim->getSimulator();

    // if the simulation is not running then don't do anything
    bool isSimRunning = tsim->isRunning();
    if(!isSimRunning || _nrOfTests>=_nrOfTestsSpin->value()){
        return;
    }

    double time = sim->getTime();

    //std::cout << "Curtime: " << time << " nextTime: " << _nextTimeUpdate[i] << std::endl;
    // we only want to check up on the simulation once every fixed timestep
    // (not necesarily the timestep of the simulator
    if( time<_nextTimeUpdate[i] ){
        return;
    }
    _nextTimeUpdate[i] += _logIntervalSpin->value()/1000.0;

    // record the tactile stuff if enabled
    int fingersWithData = 0;
    std::vector<matrix<float> > datas;
    if( _continuesLogging->isChecked() ){

        // save tactile data
        std::vector<SimulatedSensorPtr> sensors = sim->getSensors();

        BOOST_FOREACH(SimulatedSensorPtr& sensor, sensors){
            if( TactileArraySensor *tsensor = dynamic_cast<TactileArraySensor*>( sensor.get() ) ) {
                datas.push_back( tsensor->getTexelData() );

                float sum = 0;
                // we require samples on at least two of the fingers
                for(int i=0; i<datas.back().size1(); i++)
                    for(int j=0; j<datas.back().size2(); j++)
                        sum += datas.back()(i,j);
                if( sum>1 ){
                    fingersWithData++;
                }
            }
        }
    }
    if( fingersWithData>0 ){
        _fingersInContact[i] = true;
    } else {
        if( _fingersInContact[i] ){
            _tactiledatas[i].clear();
            _handconfigs[i].clear();
            _fingersInContact[i] = false;
        }
    }

    if(_fingersInContact[i]){
        _tactiledatas[i].push_back(datas);
        // save configuration
        _handconfigs[i].push_back(_hand->getModel().getQ(state));
    }

    bool isSimFinished = isSimulationFinished(sim, state);

    // get dt and make a simulation step

    // if requestet add the state to the state trajectory
    //if( _recordStatePath->checked() )
    //    _statePath.push_back( Timed<State>(time+_simStartTime, state) );

    if(!isSimFinished || time<_minTimeValidSpin->value())
        _lastBelowThresUpdate = time;

    if(i==0)
        _state = state;

    if( time-_lastBelowThresUpdate >  _minRestTimeSpin->value() ){
        // one simulation has finished...
        if(saveRestingState(i, sim, state)){
        	_totalSimTime += time;
            _nrOfTests++;
        }

        _resultPoses.push_back(state);
        _startPoses.push_back(_initStates[i]);

        // recalc random start configurations and reset the simulator
        _nextTimeUpdate[i] = 0;
        State nstate = _defstate;
        _currentPreshapeIDX[i] = Math::ranI(0,_preshapes.size());
        _hand->getModel().setQ(_preshapes[_currentPreshapeIDX[i]], nstate);
        _controllers[i]->setTargetPos(_targetQ[_currentPreshapeIDX[i]] );

        calcRandomCfg(nstate);
        _fingersInContact[i] = false;
        _initStates[i] = nstate;
        tsim->setState(nstate);
        _simStartTimes[i] = time;
    } else if( time>_maxRunningTimeSpin->value() ){
        // recalc random start configurations and reset the simulator
        _nextTimeUpdate[i] = 0;
        State nstate = _defstate;
        _currentPreshapeIDX[i] = Math::ranI(0,_preshapes.size());
        _hand->getModel().setQ(_preshapes[_currentPreshapeIDX[i]], nstate);
        _controllers[i]->setTargetPos(_targetQ[_currentPreshapeIDX[i]] );
        calcRandomCfg(nstate);
        _fingersInContact[i] = false;
        _initStates[i] = nstate;
        tsim->setState(nstate);
    }
    //RW_DEBUGS(_nrOfTests<<">="<<_nrOfTestsSpin->value());
    //if( _nrOfTests>=_nrOfTestsSpin->value() )
    //    sim->stop();

}

void GraspRestingPoseDialog::btnPressed(){
    QObject *obj = sender();
    if( obj == _saveBtn1 ){
    	QString str = _savePath->text();
    	std::string sstr( str.toStdString() );
        rw::models::WorkCell &wc = *_dwc->getWorkcell();
        // TODO: open file dialog and save
        TimedStatePath startPath, endPath;
        for(size_t i=0;i<_startPoses.size();i++){
            startPath.push_back(TimedState(i, _startPoses[i]));
        }
        for(size_t i=0;i<_resultPoses.size();i++){
            endPath.push_back(TimedState(i, _resultPoses[i]));
        }

        rw::loaders::PathLoader::storeTimedStatePath( wc,
            startPath, sstr+"_start.rwplay");
        rw::loaders::PathLoader::storeTimedStatePath( wc,
                                   endPath, sstr+"_end.rwplay");
    } else if( obj == _startBtn ) {
        _startBtn->setDisabled(true);
        _stopBtn->setDisabled(false);
        _simulatorBtn->setDisabled(true);
        _resetBtn->setDisabled(true);
        bool isStart = _startBtn->text() == "Start";
        _startBtn->setText("Continue");
        // initialize
        if( isStart )
            initializeStart();
        _timer->start();

        /*Log::infoLog() << "Resting pose calculation started: " << std::endl
					<< "- Time         : " << TimerUtil::currentTime() << std::endl
					<< "- Nr of tests  : " << _nrOfTestsSpin->value() << std::endl
					<< "- Nr of threads: " << _nrOfThreadsSpin->value() << std::endl;
*/
    } else if( obj == _stopBtn ) {
        if(_nrOfTests<_nrOfTestsSpin->value())
            _startBtn->setDisabled(false);
        _stopBtn->setDisabled(true);
        _resetBtn->setDisabled(false);
        _timer->stop();
        BOOST_FOREACH(Ptr<ThreadSimulator> sim,  _simulators){
            if(sim->isRunning())
                sim->stop();
        }
        /*
        Log::infoLog() << "Resting pose calculation stopped: " << std::endl
					<< "- Time         : " << TimerUtil::currentTime() << std::endl
					<< "- # tests done : " << _nrOfTests << std::endl;
*/
    } else if( obj == _resetBtn ) {
        _startBtn->setDisabled(false);
        _startBtn->setText("Start");
        _simulatorBtn->setDisabled(false);
    } else if( obj == _simulatorBtn ) {
        // create simulator

    } else if( obj == _scapeBtn ) {
    	std::string str = _savePath->text().toStdString();
    	if( _saveMultipleCheck->isChecked() ){
    		for(size_t i=0;i<_resultPoses.size();i++){
    			std::stringstream sstr;
    			//sstr.setf(std::fixed(int))
    			char cstr[20];
    			std::sprintf(cstr,"%.6u",i);
    			sstr << str << "_" << cstr;
    			ScapePoseFormat::savePoses(sstr.str(),_bodies,_resultPoses[i],
    									   _dwc->getWorkcell()->getName(),
    									   "Description");
    			//Log::infoLog() << "Exported resting poses in Scape format, to multiple files "
    			//			<< StringUtil::quote(str) << "_XXXXXX" << std::endl;
    		}
    	} else {
			ScapePoseFormat::savePoses(str,_bodies,_resultPoses,
									   _dwc->getWorkcell()->getName(),
									   "Description");
			//Log::infoLog() << "Exported resting poses in Scape format, to file "
			//			<< StringUtil::quote(str) << std::endl;
    	}

    } else  {

    }
}

void GraspRestingPoseDialog::changedEvent(){
    QObject *obj = sender();
    if( obj == _timer ){
        // update stuff
        updateStatus();
    } else if( obj == _updateRateSpin ){
        _timer->setInterval( _updateRateSpin->value() );
    } else if( obj == _forceUpdateCheck ) {
        if( !_forceUpdateCheck->isChecked() ){
            _timer->setInterval( 100 );
        } else {
            _timer->setInterval( _updateRateSpin->value() );
        }
    }
}

bool GraspRestingPoseDialog::isSimulationFinished( SimulatorPtr sim , const State& state){
    // test if the hand has stopped moving
    double lVelThres = _linVelSpin->value();

    Q vel = _hand->getActualVelocity(state);
    //std::cout << vel << std::endl;
    if( MetricUtil::normInf( vel ) < lVelThres ){
        return true;
    }
    return false;
}

bool GraspRestingPoseDialog::saveRestingState(int simidx, SimulatorPtr sim , const rw::kinematics::State& state ){
    // test if the hand has stopped moving
    std::stringstream sstr;
    sstr << "# Object( " << _object->getName()
         << " ), Device( " << _hand->getModel().getName()
         << " ), " << _descBox->text().toStdString();


    // get the data
    int fingersWithData = 0;
    std::vector<SimulatedSensorPtr> sensors = sim->getSensors();
    std::vector<matrix<float> > datas;
    BOOST_FOREACH(SimulatedSensorPtr& sensor, sensors){
        if( TactileArraySensor *tsensor = dynamic_cast<TactileArraySensor*>( sensor.get() ) ) {
            datas.push_back( tsensor->getTexelData() );

            float sum = 0;
            // we require samples on at least two of the fingers
            for(int i=0; i<datas.back().size1(); i++)
                for(int j=0; j<datas.back().size2(); j++)
                    sum += datas.back()(i,j);
            if( sum>1 ){
                fingersWithData++;
            }
        }
    }
    if(fingersWithData<2)
        return false;

    // calculate grasp quality
    rw::math::Q qualities( Q::zero(3) );
    Grasp3D g3d( _bodySensor->getContacts() );
    if(g3d.contacts.size()<2)
        return false;
    Transform3D<> wTf = Kinematics::worldTframe(_handBase, state);
    BOOST_FOREACH(Contact3D &c, g3d.contacts){
        sstr << c.p << "  " << c.n << " ";
    }

    std::string desc = sstr.str();

    //std::cout << "***** NR OF CONTACTS IN GRASP: " << g3d.contacts.size() << std::endl;
    Vector3D<> cm = _body->getInfo().masscenter;
    std::cout << cm;
    //std::cout << "Wrench calc" << std::endl;
    WrenchMeasure3D wmeasure(new GiftWrapHull3D(), 6 );
    wmeasure.setObjectCenter(cm);
    wmeasure.quality(g3d);
    //std::cout << "Wrench calc done!" << std::endl;

    qualities(0) = wmeasure.getMinForce();
    qualities(1) = wmeasure.getMinTorque();

    CMDistCCPMeasure3D CMCPP( cm, 0.3);
    qualities(2) = CMCPP.quality( g3d );


    // is it stable
    bool isStable = qualities(0)>0.2 && qualities(1)>0.001;

    //if(!isStable){
    //    _graspNotStable = !isStable;
    //}
    Frame *world = _dwc->getWorkcell()->getWorldFrame();
    std::stringstream squal;
    squal << isStable  <<"grasp_"<<_nrOfTests<< "_qf" << qualities(0) << "_qt_" << qualities(1)<< "_";
    world->getPropertyMap().set<std::string>("GraspQuality",squal.str());

    //
    int preshapeId = 0;

    Q preq = _preshapes[_currentPreshapeIDX[simidx]];
    std::vector<Q> handq;
    handq.push_back(_hand->getModel().getQ(state));

    Vector3D<> approach = Kinematics::worldTframe(_handBase, state).R() * Vector3D<>(0,0,1);

    QString str = _savePath->text();
    std::string pathPre( str.toStdString() );
    world->getPropertyMap().set<std::string>("PathPre",pathPre);


    std::stringstream filename;
    if(isStable){
        filename << pathPre << "/S_OutDataTest_" << (_preshapeStratBox->currentText().toStdString()) << "_" << _nrOfTests << ".txt";
    }else {
        filename << pathPre << "/U_OutDataTest_" << (_preshapeStratBox->currentText().toStdString()) << "_" << _nrOfTests << ".txt";
    }

    //std::cout << "Push restcfg\n";
    _restingConfigs.push( RestingConfig(state, filename.str().c_str() ) );
    //std::cout << "save stuff\n";
    if( _continuesLogging->isChecked() ){
    	saveRestingPose(desc,_tactiledatas[simidx],qualities,isStable,preshapeId,preq,_handconfigs[simidx],approach, filename.str(), squal.str() );
    } else {
		std::vector<TactileSensorData> tdatas;
		tdatas.push_back(datas);
		saveRestingPose(desc,tdatas,qualities,isStable,preshapeId,preq,handq,approach, filename.str(), squal.str() );
    }
	_tactiledatas[simidx].clear();
	_handconfigs[simidx].clear();
	return true;
}

void GraspRestingPoseDialog::updateStatus(){

    if( _simulators.size()<1 )
        return;

    RestingConfig restcfg;
    if( _restingConfigs.try_pop(&restcfg) ){
        //std::cout << "CALLL RESTING POSE EVENT!!!!!" << std::endl;
        restingPoseEvent( restcfg );
        //std::cout << "MYMY IS FINISH" << std::endl;
    }
    //std::cout << "---------------- NR TEST: " << _nrOfTestsOld << std::endl;
    //std::cout << "---------------- NR TEST: " << _nrOfTests << std::endl;
    int nrOfTestsOld = _nrOfTestsOld;
    double avgTestTime = 0;
    double avgSimTimePerTest = 0;
    double simTimeToReal = 0;
    if(_nrOfTests>0 && nrOfTestsOld!=_nrOfTests){
        _nrOfTestsOld = _nrOfTests;
        long realtime = rw::common::TimerUtil::currentTimeMs();
        avgTestTime = (realtime-_lastTime)/(double)(_nrOfTests-nrOfTestsOld);
        _lastTime = realtime;
        _avgTime.addSample(avgTestTime);
        avgTestTime = _avgTime.getAverage();

        int progress = (_nrOfTests*100)/_nrOfTestsSpin->value();
        _simProgress->setValue( progress );

        //avgSimTimePerTest = _totalSimTime/(double)_nrOfTests;
        avgSimTimePerTest = 0;//_avgSimTime.getAverage();

        simTimeToReal = avgSimTimePerTest*1000.0/avgTestTime;

        double timeLeft = ((_nrOfTestsSpin->value()-_nrOfTests)*avgTestTime)/1000.0;

        // update the time label
        {
            std::stringstream s; s << avgTestTime/1000.0 << "s";
            _avgTimePerTestLbl->setText(s.str().c_str());
        }
        {
            std::stringstream s; s << timeLeft << "s";
            _estimatedTimeLeftLbl->setText(s.str().c_str());
        }
        {
            std::stringstream s; s << timeLeft << "s";
            _estimatedTimeLeftLbl->setText(s.str().c_str());
        }
        {
            std::stringstream s; s << simTimeToReal;
            _simSpeedLbl->setText(s.str().c_str());
        }
        {
            std::stringstream s; s << avgSimTimePerTest << "s";
            _avgSimTimeLbl->setText(s.str().c_str());
        }
        {
            std::stringstream s; s << _nrOfTests << "/" << _nrOfTestsSpin->value();
            _testsLeftLbl->setText(s.str().c_str());
        }
    }


    // and last signal that workcell state has changed if user request it
    if(_forceUpdateCheck->isChecked()){
        stateChanged(_state);
    }

    if( _nrOfTests>=_nrOfTestsSpin->value() ){
        if(_exitHard)
            exit(0);

        _stopBtn->click();
        return;
    }
}

void GraspRestingPoseDialog::calcColFreeRandomCfg(rw::kinematics::State& state){
    //std::cout << "-------- Col free collision: " << std::endl;
    // first calculate a random state
    FramePairSet colresult;
    State istate;
    do {
        istate=state;
        std::cout << ".";
        calcRandomCfg(_bodies, istate);
    } while(_colDect->inCollision(istate, &colresult, false));
    state = istate;
    /*
    calcRandomCfg(_bodies, state);
    int nrOfTries=0;
    FramePairSet result;
    std::vector<RigidBody*> bodies;
    while( _colDect->inCollision(state, &result, false) ){
        nrOfTries++;
        BOOST_FOREACH(rw::kinematics::FramePair pair, result){
            // generate new collision free configuration between
            RigidBody *body1 = _frameToBody[*pair.first];
            RigidBody *body2 = _frameToBody[*pair.second];
            // calc new configuration
            bodies.push_back(body1);
            //bodies.push_back(body2);
        }
        calcRandomCfg(bodies, state);
        result.clear();
        bodies.clear();
    }*/
    //std::cout << "-------- Col free collision END " << std::endl;

}
#define CYLENDRICAL_APPROACH

#ifdef CYLENDRICAL_APPROACH
void GraspRestingPoseDialog::calcRandomCfg(std::vector<RigidBody*> &bodies, rw::kinematics::State& state){
    const double lowR = Deg2Rad * ( _lowRollSpin->value() );
    const double highR = Deg2Rad * ( _highRollSpin->value() );
    const double lowP = Deg2Rad * ( _lowPitchSpin->value() );
    const double highP = Deg2Rad * ( _highPitchSpin->value() );
    const double lowY = Deg2Rad * ( _lowYawSpin->value() );
    const double highY = Deg2Rad * ( _highYawSpin->value() );
    double roll = Math::ran(lowR, highR);
    double pitch = Math::ran(lowP, highP);
    double yaw = Math::ran(lowY, highY);

    double x = _xSpinBox->value();
    double y = _ySpinBox->value();
    double z = _zSpinBox->value();
    Vector3D<> rpos(Math::ran(-x,x), Math::ran(-y,y),Math::ran(-z,z));

    // choose a preshape
    /*
    _hand->getModel().setQ(_preshapes[0], state);
     */

    // so we want to place the hand in a random configuration from the side of the object
    // r and center c
    //Transform3D<> bTo = Kinematics::frameTframe(_handBase, _object, state);
    Transform3D<> wTb = Kinematics::worldTframe(_handBase, state);
    Transform3D<> wTo = Kinematics::worldTframe(_object, state);
    double r = MetricUtil::norm2( (inverse(wTb)*wTo).P() );
    //Vector3D<> bPo = bTo.P();
    //rrot*bTo.R();
    // and we find a rotation for the hand base referenced in world and apply it
    Rotation3D<> hR = RPY<>(roll,pitch,yaw).toRotation3D();
    wTb.R() = hR*wTb.R();
    // set the transform origin to that of the objects center of mass
    wTb.P() = wTo.P() + wTo.R()*_body->getInfo().masscenter;

    // translate along the negative z-axis
    wTb.P() -= wTb.R()*Vector3D<>(0,0,1) * r;

    Transform3D<> bTo = inverse(wTb)*wTo;
    bTo.P() += rpos;
    wTb = wTo*inverse(bTo);
    //double r = MetricUtil::norm2(bTo.P());
    // we calculate the center of the object
    //bTo.P() = bTo.P() + bTo.R()*_body->getInfo().masscenter;

    //Transform3D<> oTb_n = /*Transform3D<>(Vector3D<>(0,0,0),hR) * */ inverse(bTo);
   // Transform3D<> wTb_n = wTo*oTb_n;
    // add some deviation in the position and also remember the masscenter offset
    //wTb_n.P() += wTb_n.R() * rpos + wTo.R()*_body->getInfo().masscenter;

    //Transform3D<> hT = _handBase->getTransform(state);
    //Rotation3D<> hR = RPY<>(Math::ran(0,Pi),0,0).toRotation3D() * hT.R();

    _handBase->setTransform(wTb, state);


}
#else
void GraspRestingPoseDialog::calcRandomCfg(std::vector<RigidBody*> &bodies, rw::kinematics::State& state){
    // TODO: pick a stable object pose.
    const double lowR = Deg2Rad * ( _lowRollSpin->value() );
    const double highR = Deg2Rad * ( _highRollSpin->value() );
    const double lowP = Deg2Rad * ( _lowPitchSpin->value() );
    const double highP = Deg2Rad * ( _highPitchSpin->value() );
    const double lowY = Deg2Rad * ( _lowYawSpin->value() );
    const double highY = Deg2Rad * ( _highYawSpin->value() );
    double roll = Math::ran(lowR, highR);
    double pitch = Math::ran(lowP, highP);
    double yaw = Math::ran(lowY, highY);
    Rotation3D<> rrot = RPY<>(roll,pitch,yaw).toRotation3D();

    double x = _xSpinBox->value();
    double y = _ySpinBox->value();
    double z = _zSpinBox->value();
    Vector3D<> rpos(Math::ran(-x,x), Math::ran(-y,y),Math::ran(-z,z));

    // choose a preshape
 /*   _hand->getModel().setQ(_preshapes[0], state);
*/
    // so we want to place the hand in a random configuration on the sphere with radius
    // r and center c
    Transform3D<> bTo = Kinematics::frameTframe(_handBase, _object, state);
    Transform3D<> wTo = Kinematics::worldTframe(_object, state);
    double r = MetricUtil::norm2(bTo.P());
    //Vector3D<> bPo = bTo.P();
    //rrot*bTo.R();
    // we calculate the center of the object
    bTo.P() = bTo.P() + bTo.R()*_body->getInfo().masscenter;

    // and we find a random rotation and apply it
    Rotation3D<> hR = RPY<>(Math::ran(-Pi/2,Pi/2),
                            Math::ran(-Pi/2,Pi/2),
                            Math::ran(-Pi/2,Pi/2)).toRotation3D()/* * bTo.R()*/;


    Transform3D<> oTb_n = Transform3D<>(Vector3D<>(0,0,0),hR) * inverse(bTo);
    Transform3D<> wTb_n = wTo*oTb_n;
    // add some deviation in the position and also remember the masscenter offset
    wTb_n.P() += wTb_n.R() * rpos + wTo.R()*_body->getInfo().masscenter;

    //Transform3D<> hT = _handBase->getTransform(state);
    //Rotation3D<> hR = RPY<>(Math::ran(0,Pi),0,0).toRotation3D() * hT.R();

    _handBase->setTransform(wTb_n, state);



/*


    BOOST_FOREACH(RigidBody *rbody, bodies){
        double roll = Math::ran(lowR, highR);
        double pitch = Math::ran(lowP, highP);
        double yaw = Math::ran(lowY, highY);
        Transform3D<> t3d = Kinematics::worldTframe(&(rbody->getMovableFrame()), _defstate);
        Transform3D<> nt3d = t3d;
        nt3d.R() = t3d.R()*Rotation3D<>( RPY<>(roll,pitch,yaw).toRotation3D() );
        rbody->getMovableFrame().setTransform(nt3d,state);
    }
    */
}
#endif
/*
void GraspRestingPoseDialog::updateController(){
    if()
}
*/
void GraspRestingPoseDialog::calcRandomCfg(rw::kinematics::State& state){
    if( _colFreeStart->isChecked() ){
        calcColFreeRandomCfg(state);
    } else {
        calcRandomCfg(_bodies, state);
    }
}
