#include "GraspRestingPoseDialog.hpp"

#include <boost/foreach.hpp>

#include <rw/rw.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/simulator/PhysicsEngineFactory.hpp>
#include <rwsim/loaders/ScapePoseFormat.hpp>
#include <rwsim/sensor/TactileArraySensor.hpp>

#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <fstream>
#include <string>

#include "ui_GraspRestingPoseDialog.h"

using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsim::util;
using namespace rwsim::control;
using namespace rwsim::sensor;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::loaders;
using namespace rw::sensor;
using namespace rw::trajectory;
using namespace rwlibs::simulation;
using namespace rw::models;
using namespace rw::geometry;
using namespace rw::graspplanning;

namespace bfs=boost::filesystem;

//#define RW_DEBUGS( str ) std::cout << str  << std::endl;
#define RW_DEBUGS( str )

namespace {


}

namespace {

	class TimeLabel: public QLabel {
		TimeLabel(){}

		void setTimeInMs();
		void setTimeInSec();
		void setTimeInMin();


	};

    void saveRestingPoseSimple(std::string& desc,
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

        // save stability
        if(isStable) file << "1\t"; else file << "0\t";

        // Grasp Qualities
        for(size_t i=0;i<qualities.size(); i++)
            file << qualities[i] << "\t";

        // preshape ID
        file << preshapeId << "\t";

        // preshape of hand
        for(size_t i=0;i<preq.size(); i++)
                file << preq[i]*Rad2Deg << "\t";

        // resting configuration of hand
        for(size_t i=0;i<handq.back().size(); i++)
                file << handq.back()[i]*Rad2Deg << "\t";

        // approach vector
        file << approach(0) << "\t" << approach[1] << "\t" << approach[2] << "\t";
        file << imgfile << "\n";
/*
        file << "#\n number of tactile sensor data readings\n";
        file << datas.size();

        file << "\n\n# tactile sensor data \n";

        // save tactile data
        int idx=0;
        BOOST_FOREACH(GraspRestingPoseDialog::TactileSensorData& tdata, datas){

            file << "# Q " << idx << "/" << datas.size() << "\n";
            for(size_t i=0;i<handq.back().size(); i++)
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
        */
    }


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
                file << preq[i]*Rad2Deg << " ";
        file << "\n\n";

        // resting configuration of hand
        file << "# grasp contact configuration \n";
        for(size_t i=0;i<handq.back().size(); i++)
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
                    file << handq[idx][i]*Rad2Deg << " ";
            file << "\n\n";

            file << "# Tdata " << idx << "/" << datas.size() << "\n";
			BOOST_FOREACH(Eigen::MatrixXf& data, tdata){
				for(Eigen::DenseIndex x=0;x<data.rows(); x++){
					for(Eigen::DenseIndex y=0;y<data.cols(); y++){
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
                                    DynamicWorkCell *dwc,
                                    rw::proximity::CollisionDetector *detector,
                                    QWidget *parent):
    QDialog(parent),
    _defstate(state),
    _state(state),
    _nrOfTests(0),
    _totalSimTime(0),
    _startTime(0),
    _id("0"),
    _dwc(dwc),
    _colDect(detector),
    _lastTime(0),
    _lastBelowThresUpdate(0),
    _avgSimTime(4),
    _avgTime(4),
    _handBase(NULL),
    _object(NULL),
    _exitHard(false),
    _graspNotStable(false),
    _nrOfTestsOld(0),
    _gtable("",""),
    _nrOfGraspsInGroup(0),
    _lastTableBackupCnt(0),
    _tactileDataOnAllCnt(0)
{
	RW_ASSERT( _dwc );
	_ui = new Ui::GraspRestingPoseDialog();
	_ui->setupUi(this);

    Math::seed( TimerUtil::currentTimeMs() );

    RW_DEBUGS("- Setting devices ");
    std::vector<DynamicDevice::Ptr> devices = _dwc->getDynamicDevices();
    BOOST_FOREACH(DynamicDevice::Ptr device, devices){
        if( device.cast<RigidDevice>() ){
            rw::models::Device *dev = &device->getModel();
            RW_ASSERT(dev);
            RW_DEBUGS("-- Dev name: " << dev->getName() );
            _ui->_deviceBox->addItem(dev->getName().c_str());
        }
    }

    RW_DEBUGS("- Setting objects ");
    BOOST_FOREACH(Body::Ptr body, _dwc->getBodies() ){
        Frame *obj = body->getBodyFrame();
        if(obj==NULL)
            continue;
        if( dynamic_cast<const MovableFrame*>(obj) ){
            _ui->_objectBox->addItem(obj->getName().c_str());
        }
    }

    _ui->_preshapeStratBox->addItem("Parallel");
    _ui->_preshapeStratBox->addItem("Spherical");
    _ui->_preshapeStratBox->addItem("Multiple10");
    _ui->_preshapeStratBox->addItem("CylStanding");


    _ui->_graspPolicyBox->addItem("SimpleClose");

    RW_DEBUGS("- Setting connections ");
    connect(_ui->_saveBtn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_startBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_resetBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_stopBtn     ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_simulatorBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_scapeBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_ui->_updateRateSpin, SIGNAL(valueChanged(int)), this, SLOT(changedEvent()) );
    connect(_ui->_fixedGroupBox, SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );

    PropertyMap map;
/*
    map.set<int>("R_low", -180);
    map.set<int>("R_high", 180);
    map.set<int>("P_low", -180);
    map.set<int>("P_high", 180);
    map.set<int>("Y_low", -180);
    map.set<int>("Y_high", 180);

    map.set<int>("x_delta", 3);
    map.set<int>("y_delta", 3);
    map.set<int>("z_delta", 3);

    XMLPropertySaver::save(map, "GraspTableConfig.xml");
*/
    map = XMLPropertyLoader::load( "GraspTableConfig.xml" );
    _ui->_lowRollSpin->setValue(map.get<int>("R_low",-180));
    _ui->_highRollSpin->setValue(map.get<int>("R_high",180));
    _ui->_lowPitchSpin->setValue(map.get<int>("P_low",-180));
    _ui->_highPitchSpin->setValue(map.get<int>("P_high",180));
    _ui->_lowYawSpin->setValue(map.get<int>("Y_low",-180));
    _ui->_highYawSpin->setValue(map.get<int>("Y_high",180));

    _ui->_xSpinBox->setValue(map.get<int>("x_delta",3)/100.0);
    _ui->_ySpinBox->setValue(map.get<int>("y_delta",3)/100.0);
    _ui->_zSpinBox->setValue(map.get<int>("z_delta",3)/100.0);



    RW_DEBUGS("- Setting timer ");
    _timer = new QTimer( NULL );
    _timer->setInterval( _ui->_updateRateSpin->value() );
    connect( _timer, SIGNAL(timeout()), this, SLOT(changedEvent()) );



}

void GraspRestingPoseDialog::setSaveDir(const std::string& str){
    _ui->_savePath->setText(str.c_str());
}



void GraspRestingPoseDialog::setPreshapeStrategy(const std::string& str){
    int idx = _ui->_preshapeStratBox->findText(str.c_str());
    if( idx>=0 )
        _ui->_preshapeStratBox->setCurrentIndex(idx);
}



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
    _lastTableBackupCnt = 0;
    _tactileDataOnAllCnt=0;

    // if the designated directory for saving the output does not exist, then create it
    bfs::create_directory( bfs::path(_ui->_savePath->text().toStdString()) );

    // write configuration to configfile
    QString str = _ui->_savePath->text();
    std::string pathPre( str.toStdString() );
    std::stringstream filename, pmap_filename;
    filename << pathPre << "/config_file.txt";

    std::ofstream file( filename.str().c_str() );
    if(!file.is_open())
        RW_THROW("CANNOT OPEN FILE! "<< filename);

    pmap_filename << pathPre << "/progress.xml";


    PropertyMap pmap;
    try {
    	pmap = XMLPropertyLoader::load( pmap_filename.str() );
    } catch(...){
    	std::cout << "File not found!" << std::endl;
    }
    int progressid = pmap.get<int>("PROGRESS_COUNT",0);
    progressid++;
    pmap.set<int>("PROGRESS_COUNT",progressid);
    XMLPropertySaver::save( pmap, pmap_filename.str() );

    std::stringstream sstemp;
    sstemp << progressid;
    _id = sstemp.str();

    State state = _defstate;
    int threads = _ui->_nrOfThreadsSpin->value();
    _simStartTimes.resize(threads, 0);
    RW_DEBUGS("threads: " << threads);
    file << "Nr of threads used: " << threads << std::endl;

    RW_DEBUGS("- Getting object!");
    std::string objName = _ui->_objectBox->currentText().toStdString();
    BOOST_FOREACH(Body::Ptr body, _dwc->getBodies()){
        if(RigidBody::Ptr rbody = body.cast<RigidBody>() ){
            if(rbody->getBodyFrame()->getName()==objName){
                _bodies.push_back(rbody);
                _body = rbody;
                _frameToBody[*rbody->getMovableFrame()] = rbody;
                _object = rbody->getMovableFrame();
                _body->getInfo().print(file);
                break;
            }
        }
    }
    if(_bodies.size()==0){
        RW_THROW("You must choose a valid object to grasp!");
    }

    RW_DEBUGS("- Getting device!");
    std::string devName = _ui->_deviceBox->currentText().toStdString();
    DynamicDevice::Ptr dev = _dwc->findDevice(devName);
    _hand = dev.cast<RigidDevice>();
    file << "Hand device name: " << devName << std::endl;
    RW_ASSERT(_hand);
    _handForceLimitsDefault = _hand->getMotorForceLimits();
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
    if( _ui->_preshapeStratBox->currentText()=="Parallel"){
        preQ(0) = -1.5; preQ(1) = -1.5;
        preQ(2) = 1.571;
        preQ(3) = -1.5; preQ(4) = 0;
        preQ(5) = -1.5; preQ(6) = 0;

        tQ(0) = -1.5; tQ(1) = -1.5;
        tQ(2) = 1.571;
        tQ(3) = 1; tQ(4) = 0.6;
        tQ(5) = 1; tQ(6) = 0.6;
        _preshapes.push_back( preQ );
        _targetQ.push_back( tQ );
    } else if(_ui->_preshapeStratBox->currentText()=="Spherical"){
        preQ(0) = -1.5; preQ(1) = 0;
        preQ(2) = 0.785;
        preQ(3) = -1.5; preQ(4) = 0;
        preQ(5) = -1.5; preQ(6) = 0;

        tQ(0) = 1; tQ(1) = 0.6;
        tQ(2) = 0.785;
        tQ(3) = 1; tQ(4) = 0.6;
        tQ(5) = 1; tQ(6) = 0.6;
        _preshapes.push_back( preQ );
        _targetQ.push_back( tQ );
    }else if(_ui->_preshapeStratBox->currentText()=="CylStanding"){
        preQ(0) = -1.5; preQ(1) = 0;
        preQ(2) = 0;
        preQ(3) = -1.5; preQ(4) = 0;
        preQ(5) = -1.5; preQ(6) = 0;

        tQ(0) = 1; tQ(1) = 0.6;
        tQ(2) = 0;
        tQ(3) = 1; tQ(4) = 0.6;
        tQ(5) = 1; tQ(6) = 0.6;
        _preshapes.push_back( preQ );
        _targetQ.push_back( tQ );
    } else if(_ui->_preshapeStratBox->currentText()=="Multiple10"){
        for(int i=0;i<10;i++){
            preQ(0) = -1.5; preQ(1) = 0;
            preQ(2) = Pi/20*i;
            preQ(3) = -1.5; preQ(4) = 0;
            preQ(5) = -1.5; preQ(6) = 0;

            tQ(0) = 1; tQ(1) = 0.6;
            tQ(2) = Pi/20*i;
            tQ(3) = 1; tQ(4) = 0.6;
            tQ(5) = 1; tQ(6) = 0.6;

            _preshapes.push_back( preQ );
            _targetQ.push_back( tQ );
        }
    }

    _hand->getModel().setQ(preQ, _defstate);
    file << "Preshape configuration type: " << _ui->_preshapeStratBox->currentText().toStdString() << "\n";
    file << "preshapes:  \n";
    BOOST_FOREACH(Q pq, _preshapes){
    	file << "- " << pq << "\n";
    }
    file << "\nTargets: \n";
    BOOST_FOREACH(Q pq, _targetQ){
    	file << "- " << pq << "\n";
    }

    file << "\nTime between tactile readings: " << _ui->_updateRateSpin->value() << "ms\n";

    _gtable = GraspTable(_hand->getModel().getName(), _bodies[0]->getMovableFrame()->getName());

    RW_DEBUGS("- Creating simulators: ");
    for(int i=0;i<threads;i++){
        state = _defstate;
        // create simulator
        RW_DEBUGS("-- sim nr " << i);
        PhysicsEngine::Ptr pengine = PhysicsEngineFactory::makePhysicsEngine("ODE",_dwc);
        DynamicSimulator::Ptr sim = ownedPtr(new DynamicSimulator(_dwc, pengine));
        RW_DEBUGS("-- Initialize simulator " << i);
        sim->init(state);

        // create a controller
        //PDControllerPtr pdctrl =
        //		ownedPtr(new PDController(_hand, PDController::POSITION, PDParam(10,0.03), 0.1));
        //pdctrl->reset( state );
        //sim->addController(pdctrl);
        //_controllers.push_back(pdctrl);
        //pdctrl->setTargetPos(_targetQ[0] );
        //_controllers = _dwc->getControllers();

        // add body sensor
        _bodySensor = ownedPtr( new BodyContactSensor("BodySensor", _object) );
        sim->addSensor( _bodySensor, state);

        ThreadSimulator *tsim = new ThreadSimulator(sim,state);
        CallBackFunctor *func = new CallBackFunctor(i, this);
        _functors.push_back( ownedPtr(func) );
        ThreadSimulator::StepCallback cb( boost::bind(&CallBackFunctor::stepCallBack, func, _1, _2) );

        tsim->setStepCallBack( cb );
        _simulators.push_back( ownedPtr( tsim ) );
        tsim->setRealTimeScale(0);
        tsim->setTimeStep(0.001);
        RW_DEBUGS("-- Calc random cfg " << i);

        state = _defstate;

		// generate a new random grasp
        _currentPreshapeIDX.push_back(i);
        _fingersInContact.push_back(false);

        _currentPreshapeIDX[i] = Math::ranI(0,(int)_preshapes.size());
        _preshape = _preshapes[_currentPreshapeIDX[i]];
        _target = _targetQ[_currentPreshapeIDX[i]];

        _hand->getModel().setQ(_preshape, state);
        calcRandomCfg(state);

        _initStates.push_back(state);
        _nextTimeUpdate.push_back( _ui->_updateRateSpin->value()/1000.0 );
        _tactiledatas.push_back(std::vector< TactileSensorData >());
        _handconfigs.push_back( std::vector< rw::math::Q >()) ;


        tsim->setState(state);
        tsim->start();
    }

    _startTime = rw::common::TimerUtil::currentTimeMs();
    _lastTime = _startTime;
    RW_DEBUGS("init finished");
    file.close();
}

void GraspRestingPoseDialog::startAuto(){
    _exitHard = true;
    _ui->_startBtn->click();

}

namespace {

	std::vector<Eigen::MatrixXf> getTactileData(const std::vector<SimulatedSensor::Ptr>& sensors, const State& state){
		std::vector<Eigen::MatrixXf> datas;
		BOOST_FOREACH(const SimulatedSensor::Ptr& sensor, sensors){
        	if( TactileArraySensor *tsensor = dynamic_cast<TactileArraySensor*>( sensor.get() ) ) {
                datas.push_back( tsensor->getTexelData(state) );
            }
        }
		return datas;
	}
}

void GraspRestingPoseDialog::stepCallBack(int i, const rw::kinematics::State& state){
	try{


	RW_DEBUGS("StepCallBack " << i);
	ThreadSimulator::Ptr tsim = _simulators[i];

    DynamicSimulator::Ptr sim = tsim->getSimulator();// if the simulation is not running then don't do anything
    bool isSimRunning = tsim->isRunning();
    if(!isSimRunning || _nrOfTests>=_ui->_nrOfTestsSpin->value()){
    	std::string str = _ui->_savePath->text().toStdString();
    	std::stringstream sstr;
    	sstr << str << "/" << "grasptablefile.txt";

    	_gtable.save(sstr.str());
    	RW_DEBUGS("StepCallBack - done (save)");
        return;
    }

    double time = sim->getTime();

    // check if any contact forces are too large...
    bool largeForces = false;
    std::vector<rw::sensor::Contact3D> contacts = _bodySensor->getContacts(state);
    BOOST_FOREACH(rw::sensor::Contact3D& con, contacts){
    	if( con.normalForce >1000 )
    		largeForces = true;
    }

    if( tsim->isInError() || largeForces){
		// recalc random start configurations and reset the simulator
		_nextTimeUpdate[i] = 0;
		State nstate = _defstate;
		_currentPreshapeIDX[i] = Math::ranI(0,(int)_preshapes.size());
		_hand->getModel().setQ(_preshapes[_currentPreshapeIDX[i]], nstate);
		_controllers[i]->setTargetPos(_targetQ[_currentPreshapeIDX[i]] );

		calcRandomCfg(nstate);

		_fingersInContact[i] = false;
		_initStates[i] = nstate;
		tsim->setState(nstate);
		std::cout << "**************** IS IN ERROR, NEW RANDOM CONFIG! *************" << std::endl;
		return;
	}

    //std::cout << "Curtime: " << time << " nextTime: " << _nextTimeUpdate[i] << std::endl;
    // we only want to check up on the simulation once every fixed timestep
    // (not necesarily the timestep of the simulator
    if( time<_nextTimeUpdate[i] ){
    	RW_DEBUGS("StepCallBack - done (next time)");
        return;
    }
    _nextTimeUpdate[i] += _ui->_logIntervalSpin->value()/1000.0;

    // record the tactile stuff if enabled
    int fingersWithData = 0;
    std::vector<TactileArrayModel::ValueMatrix> datas;


    // save tactile data
    std::vector<SimulatedSensor::Ptr> sensors = sim->getSensors();
    std::vector<std::vector<Contact3D> > tactileContacts;
    BOOST_FOREACH(SimulatedSensor::Ptr& sensor, sensors){
        if( TactileArraySensor *tsensor = dynamic_cast<TactileArraySensor*>( sensor.get() ) ) {
            datas.push_back( tsensor->getTexelData(state) );

            float sum = 0;
            // we require samples on at least two of the fingers
            for(Eigen::DenseIndex i=0; i<datas.back().rows(); i++)
                for(Eigen::DenseIndex j=0; j<datas.back().cols(); j++)
                	if(datas.back()(i,j)>0.5)
                		sum += datas.back()(i,j);
            if( sum>1 ){
            	tactileContacts.push_back(tsensor->getActualContacts(state));
                fingersWithData++;
            }

        }
    }

    if( fingersWithData>0 ){
    	_tactileDataOnAllCnt++;
    } else {
    	_tactileDataOnAllCnt =0;
    }

    if( _ui->_continuesLogging->isChecked() ){

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

    }

    bool isSimFinished = isSimulationFinished(sim, state);

    // get dt and make a simulation step

    // if requestet add the state to the state trajectory
    //if( _recordStatePath->checked() )
    //    _statePath.push_back( Timed<State>(time+_simStartTime, state) );

    if(!isSimFinished || time<_ui->_minTimeValidSpin->value())
        _lastBelowThresUpdate = time;

    if(i==0)
        _state = state;
    //bool dataValid = false;
    if( time-_lastBelowThresUpdate >  _ui->_minRestTimeSpin->value() || _tactileDataOnAllCnt>50){
        // one simulation has finished...
    	_tactileDataOnAllCnt = 0;
        {
            int simidx = i;
            // calculate grasp quality
            rw::math::Q qualities( Q::zero(3) );
            Grasp3D g3d( _bodySensor->getContacts(state) );

            //Transform3D<> wTf = Kinematics::worldTframe(_handBase, state);
            RW_DEBUGS("***** NR OF CONTACTS IN GRASP: " << g3d.contacts.size());
            std::cout << "***** NR OF CONTACTS IN GRASP: " << g3d.contacts.size() << std::endl;

            Vector3D<> cm = _body->getInfo().masscenter;
            //std::cout << cm << std::endl;

            if(g3d.contacts.size()>1){
            	RW_DEBUGS("Wrench calc");
            	try {
            	    WrenchMeasure3D wmeasure(8);
            	    wmeasure.setLambda( 0.3 );
					wmeasure.setObjectCenter(cm);
					wmeasure.quality(g3d);

					RW_DEBUGS("Wrench calc done!");

					qualities(0) = wmeasure.getMinWrench();
					qualities(1) = 0;

					std::cout << "*** Center of object: " << cm << std::endl;
					BOOST_FOREACH(Contact3D& c, g3d.contacts){
						std::cout << c.p << std::endl;
						std::cout << "\t" << c.p-cm << std::endl;
					}
					std::cout << "*****************************" << std::endl;

					CMDistCCPMeasure3D CMCPP( cm, 0.3);
					qualities(2) = CMCPP.quality( g3d );
            	} catch(...){
            		RW_WARN("CAUGHT GIFT WRAP BUG!");
            		qualities(0) = 0;
            		qualities(1) = 0;
            		qualities(2) = 0;
            	}
            }

            //Frame *world = _dwc->getWorkcell()->getWorldFrame();
            Transform3D<> wThb = Kinematics::worldTframe(_handBase, state);
            Transform3D<> hbTw = inverse(wThb);
            Transform3D<> wTo = Kinematics::worldTframe(_object, state);

            Transform3D<> hpTo = inverse(wThb)*wTo;

            Vector3D<> approach = hpTo * Vector3D<>(0,0,1);
            GraspTable::GraspData data;

            // all contacts are in world coordinates. Transform them...
            BOOST_FOREACH(std::vector<Contact3D>& convec, tactileContacts){
            	for(size_t i=0;i<convec.size();i++){
            		Contact3D &con = convec[i];
            		//_wTf*point,_wTf.R()*snormal,_wTf.R()*force
            		con.p = hbTw*con.p;
            		con.n = hbTw.R()*con.n;
            		con.f = hbTw.R()*con.f;
            	}
            }
            data.tactileContacts = tactileContacts;

            data.hp = Pose6D<>( hpTo );
            data.op = Pose6D<>( wTo );
            data.approach = approach;
            data.cq = _hand->getModel().getQ(state);
            data.pq = _preshapes[_currentPreshapeIDX[simidx]];

            data._tactiledata = getTactileData( sim->getSensors(), state );

            data.quality = qualities;
            data.grasp = g3d;


            if( _ui->_inGroupsCheck->isChecked()){
				if( _nrOfGraspsInGroup==1 && ( qualities(0)<0.1 || qualities(1)<=0 || qualities(2)<=0.1 || fingersWithData<3) ){
					_nrOfGraspsInGroup = 0;
					//dataValid = false;
				} else if(_nrOfGraspsInGroup>0){
					//dataValid = true;
					_gtable.addGrasp(data);
					_nrOfTests++;

				}
            }else {
            	_gtable.addGrasp(data);
            	_nrOfTests++;
            }

            if(_nrOfTests-_lastTableBackupCnt>100){
				std::string str = _ui->_savePath->text().toStdString();
				std::stringstream sstr;
				sstr << str << "/" << "grasptablefile" << _nrOfTests << ".txt";
				_gtable.save(sstr.str());
				_lastTableBackupCnt = _nrOfTests;
			}

        	_totalSimTime += time;
        }

        saveRestingState(i, sim, state);


        _tactiledatas[i].clear();
        _handconfigs[i].clear();
        _resultPoses.push_back(state);
        _startPoses.push_back(_initStates[i]);
        // recalc random start configurations and reset the simulator
        State nstate;
        _nextTimeUpdate[i] = 0;
        Q target = _target;
        Q preshape = _preshape;
    	if( _ui->_inGroupsCheck->isChecked()){
    		if(_nrOfGraspsInGroup == 0 || _nrOfGraspsInGroup==_ui->_groupSizeSpin->value()){
    			// generate a new random grasp
        		nstate = _defstate;
                _currentPreshapeIDX[i] = Math::ranI(0,(int)_preshapes.size());
                _preshape = _preshapes[_currentPreshapeIDX[i]];
                _target = _targetQ[_currentPreshapeIDX[i]];
                target = _target;
                preshape = _preshape;

                // set the allowed torques of the hand
                Q handForceLimits = _handForceLimitsDefault;
                double alpha = preshape(2);
                if(handForceLimits(2)<=Pi/2){
                	handForceLimits(3) = handForceLimits(0)/(2*cos(alpha));
                	handForceLimits(5) = handForceLimits(0)/(2*cos(alpha));
                } else {
                	handForceLimits(0) = 2*cos(alpha)*handForceLimits(3);
                }
                _hand->setMotorForceLimits(handForceLimits);
                _hand->getModel().setQ(preshape, nstate);

                calcRandomCfg(nstate);
    			// and make sure to save
                _nrOfGraspsInGroup = 1;
    			_initStates[i] = nstate;
    		} else {
    			// calculate a pose of the hand that is very close to the previous
    			if(_nrOfGraspsInGroup==1){
    				_objTransform = _object->getTransform(state);
    				_preshape = _hand->getQ(state);
    				_target = _hand->getQ(state);
    			}
    			nstate = _initStates[i];
    			Vector3D<> pos(0,0,0);
    			RPY<> rpy(0,0,0);
    			Q qtmp = Q::zero(6);
    			if( _ui->_fixedGroupBox->isChecked() ){
        			double devi = 0.005; //
        			if(_nrOfGraspsInGroup-1>5)
        				devi = Pi/90.0;
        			if((_nrOfGraspsInGroup-1)&0x1)
        				devi = -devi;


        			RW_DEBUGS( (int)floor(_nrOfGraspsInGroup/2.0-0.1) );
        			qtmp( (int)floor(_nrOfGraspsInGroup/2.0-0.1) ) = devi;

    			} else {
        			Q q_p(3,0.005); // 10 mm, pos
        			Q q_r(3, Pi/90.0); //
        			Q q_pr = concat(q_p,q_r);
        			Q qd = Math::ranQ(-q_pr, q_pr);
        			qtmp = qd;
    			}
    			_object->setTransform(_objTransform,nstate);
    			pos = Vector3D<>(qtmp(0),qtmp(1),qtmp(2));
    			rpy = RPY<>(qtmp(3),qtmp(4),qtmp(5));
    	        target = _target;
    	        preshape = _preshape;

    			// we set the target and preshape of the hand such that
    			// the fingers close approximately at the same point again
    			//preshape = _hand->getQ(state);
    			//target = _hand->getQ(state);

    			preshape(0) -= 30*Deg2Rad;
    			//preshape(1) -= 30*Deg2Rad;
    			//preshape(2) -= 30*Deg2Rad;
    			preshape(3) -= 30*Deg2Rad;
    			//preshape(4) -= 30*Deg2Rad;
    			//preshape(5) -= 30*Deg2Rad;
    			preshape(6) -= 30*Deg2Rad;
    			//preshape(7) -= 30*Deg2Rad;

    			target(0) += 2*Deg2Rad;
    			//target(1) += 5*Deg2Rad;
    			//preshape(2) += 5*Deg2Rad;
    			target(3) += 2*Deg2Rad;
    			//target(4) += 5*Deg2Rad;
    			//preshape(5) -= 30*Deg2Rad;
    			target(6) += 2*Deg2Rad;
    			//target(7) += 5*Deg2Rad;


    			Transform3D<> wThb = _handBase->getTransform(nstate);
    			_handBase->setTransform( wThb*Transform3D<>(pos,rpy.toRotation3D()) , nstate);

    			_nrOfGraspsInGroup++;
    			if(_nrOfGraspsInGroup>_ui->_groupSizeSpin->value())
    				_nrOfGraspsInGroup = 0;

    		}
    	} else {

    		nstate = _defstate;
            _currentPreshapeIDX[i] = Math::ranI(0,(int)_preshapes.size());
            preshape = _preshapes[_currentPreshapeIDX[i]];
            target = _targetQ[_currentPreshapeIDX[i]];

            // set the allowed torques of the hand
            Q handForceLimits = _handForceLimitsDefault;
            double alpha = preshape(2);
            if(handForceLimits(2)<=Pi/2){
            	handForceLimits(3) = handForceLimits(0)/(2*cos(alpha));
            	handForceLimits(5) = handForceLimits(0)/(2*cos(alpha));
            } else {
            	handForceLimits(0) = 2*cos(alpha)*handForceLimits(3);
            }
            _hand->setMotorForceLimits(handForceLimits);

            _hand->getModel().setQ(preshape, nstate);
            calcRandomCfg(nstate);
            _initStates[i] = nstate;
    	}

        _hand->getModel().setQ(preshape, nstate);
        _controllers[i]->setTargetPos( target );
        _fingersInContact[i] = false;

        tsim->setState(nstate);
        _simStartTimes[i] = time;
    } else if( time>_ui->_maxRunningTimeSpin->value() || tsim->isInError() ){
        // recalc random start configurations and reset the simulator
        _nextTimeUpdate[i] = 0;
        State nstate = _defstate;
        _currentPreshapeIDX[i] = Math::ranI(0,(int)_preshapes.size());
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
    //RW_DEBUGS("StepCallBack - done");

	} catch (...){
		std::cout << "EXCEPTION CAUGHT AT LAST!" <<std::endl;

	}
}

void GraspRestingPoseDialog::btnPressed(){
    QObject *obj = sender();
    if( obj == _ui->_saveBtn1 ){
    	QString str = _ui->_savePath->text();
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

        /*Log::infoLog() << "Resting pose calculation started: " << std::endl
					<< "- Time         : " << TimerUtil::currentTime() << std::endl
					<< "- Nr of tests  : " << _nrOfTestsSpin->value() << std::endl
					<< "- Nr of threads: " << _nrOfThreadsSpin->value() << std::endl;
*/
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
        /*
        Log::infoLog() << "Resting pose calculation stopped: " << std::endl
					<< "- Time         : " << TimerUtil::currentTime() << std::endl
					<< "- # tests done : " << _nrOfTests << std::endl;
*/
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
    			std::sprintf(cstr,"%.6u",(unsigned int)i);
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
    } else if( obj == _ui->_updateRateSpin ){
        _timer->setInterval( _ui->_updateRateSpin->value() );
    } else if( obj == _ui->_forceUpdateCheck ) {
        if( !_ui->_forceUpdateCheck->isChecked() ){
            _timer->setInterval( 100 );
        } else {
            _timer->setInterval( _ui->_updateRateSpin->value() );
        }
    } else if(obj == _ui->_fixedGroupBox){
    	if( _ui->_fixedGroupBox->isChecked()){
    	    _ui->_groupSizeSpin->setValue(13);
    	    _ui->_groupSizeSpin->setEnabled(false);
    	} else {
    	    _ui->_groupSizeSpin->setEnabled(true);
    	}
    }
}

bool GraspRestingPoseDialog::isSimulationFinished( DynamicSimulator::Ptr sim , const State& state){
    // test if the hand has stopped moving
    double lVelThres = _ui->_linVelSpin->value();

    Q vel = _hand->getJointVelocities(state);
    //std::cout << vel << std::endl;
    if( MetricUtil::normInf( vel ) < lVelThres ){
        return true;
    }
    return false;
}

bool GraspRestingPoseDialog::saveRestingState(int simidx, DynamicSimulator::Ptr sim , const rw::kinematics::State& state ){
    // test if the hand has stopped moving
    std::stringstream sstr;
    sstr << "# Object( " << _object->getName()
         << " ), Device( " << _hand->getModel().getName()
         << " ), " << _ui->_descBox->text().toStdString();


    // get the data
    int fingersWithData = 0;
    std::vector<SimulatedSensor::Ptr> sensors = sim->getSensors();
    std::vector<Eigen::MatrixXf> datas;
    BOOST_FOREACH(SimulatedSensor::Ptr& sensor, sensors){
        if( TactileArraySensor *tsensor = dynamic_cast<TactileArraySensor*>( sensor.get() ) ) {
            datas.push_back( tsensor->getTexelData(state) );

            float sum = 0;
            // we require samples on at least two of the fingers
            for(Eigen::DenseIndex i=0; i<datas.back().rows(); i++)
                for(Eigen::DenseIndex j=0; j<datas.back().cols(); j++)
                    sum += datas.back()(i,j);
            if( sum>1 ){
                fingersWithData++;
            }
        }
    }
    if(fingersWithData<1){
        _tactiledatas[simidx].clear();
        _handconfigs[simidx].clear();
        return false;
    }
    // calculate grasp quality
    rw::math::Q qualities( Q::zero(3) );
    Grasp3D g3d( _bodySensor->getContacts(state) );
    if(g3d.contacts.size()<1){
        _tactiledatas[simidx].clear();
        _handconfigs[simidx].clear();
        return false;
    }
    //Transform3D<> wTf = Kinematics::worldTframe(_handBase, state);
    BOOST_FOREACH(Contact3D &c, g3d.contacts){
        sstr << c.p << "  " << c.n << " ";
    }

    std::string desc = sstr.str();

    RW_DEBUGS("***** NR OF CONTACTS IN GRASP: " << g3d.contacts.size() );
    Vector3D<> cm = _body->getInfo().masscenter;
    RW_DEBUGS( cm );
    RW_DEBUGS("Wrench calc");
    try {
		GWSMeasure3D wmeasure( 8 );
		wmeasure.setLambda( 0.3 );
		wmeasure.setObjectCenter(cm);
		wmeasure.quality(g3d);
		RW_DEBUGS("Wrench calc done!");

		qualities(0) = wmeasure.getMinWrench();
		qualities(1) = 0;

		CMDistCCPMeasure3D CMCPP( cm, 0.3);
		qualities(2) = CMCPP.quality( g3d );

    } catch (...){
    	std::cout << "CAUGHT Wrench BUG!" << std::endl;
        _tactiledatas[simidx].clear();
        _handconfigs[simidx].clear();

    	return false;
    }

    // is it stable
    bool isStable = qualities(0)>2.5 /*&& qualities(1)>0.2*/;

    if(!isStable){
    //    _graspNotStable = !isStable;
    	std::cout << "UNSTABLE: \n";
    } else {
    	std::cout << "STABLE  : \n";
    }
    BOOST_FOREACH(Contact3D &c, g3d.contacts){
        std::cout << "\t" << c.p << "  " << c.n << " \n";
    }

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

    QString str = _ui->_savePath->text();
    std::string pathPre( str.toStdString() );
    world->getPropertyMap().set<std::string>("PathPre",pathPre);


    std::stringstream filename;
    if(isStable){
        filename << pathPre << "/S_OutDataTest_" << (_ui->_preshapeStratBox->currentText().toStdString()) << "_" << _id << "_" << _nrOfTests << ".txt";
    }else {
        filename << pathPre << "/U_OutDataTest_" << (_ui->_preshapeStratBox->currentText().toStdString()) << "_" << _id << "_" << _nrOfTests << ".txt";
    }

    RW_DEBUGS("Push restcfg");
    _restingConfigs.push( RestingConfig(state, filename.str().c_str() ) );
    RW_DEBUGS("save stuff");
    if( _ui->_continuesLogging->isChecked() ){
        //saveRestingPoseSimple(desc, _tactiledatas[simidx], qualities, isStable, preshapeId, preq, _handconfigs[simidx], approach, _file, squal.str());
        saveRestingPose(desc,_tactiledatas[simidx],qualities,isStable,preshapeId,preq,_handconfigs[simidx],approach, filename.str(), squal.str() );
    } else {
		std::vector<TactileSensorData> tdatas;
		tdatas.push_back(datas);
		saveRestingPose(desc,tdatas,qualities,isStable,preshapeId,preq,handq,approach, filename.str(), squal.str() );
    }
	_tactiledatas[simidx].clear();
	_handconfigs[simidx].clear();
	RW_DEBUGS("done!");
	return true;
}

void GraspRestingPoseDialog::updateStatus(){

	RW_DEBUGS("updateStatus!");
    if( _simulators.size()<1 )
        return;

    RestingConfig restcfg;
    if( _restingConfigs.try_pop(&restcfg) ){
    	//RW_DEBUGS("CALLL RESTING POSE EVENT!!!!!");
        restingPoseEvent( restcfg );
        //std::cout << "MYMY IS FINISH" << std::endl;
    }
    //RW_DEBUGS("---------------- NR TEST: " << _nrOfTestsOld);
    //RW_DEBUGS("---------------- NR TEST: " << _nrOfTests);
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

        int progress = (_nrOfTests*100)/_ui->_nrOfTestsSpin->value();
        _ui->_simProgress->setValue( progress );

        //avgSimTimePerTest = _totalSimTime/(double)_nrOfTests;
        avgSimTimePerTest = 0;//_avgSimTime.getAverage();

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
    if(_ui->_forceUpdateCheck->isChecked()){
        stateChanged(_state);
    }

    if( _nrOfTests>=_ui->_nrOfTestsSpin->value() ){
        if(_exitHard)
            exit(0);

        _ui->_stopBtn->click();
        //RW_DEBUGS("updateStatus! done (exit btn)");
        return;
    }
    //RW_DEBUGS("updateStatus! done");
}

void GraspRestingPoseDialog::calcColFreeRandomCfg(rw::kinematics::State& state){
    //std::cout << "-------- Col free collision: " << std::endl;
    // first calculate a random state
    CollisionDetector::QueryResult colresult;
    State istate;
    do {
        istate=state;
        //std::cout << ".";
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
//#define CYLENDRICAL_APPROACH

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


void GraspRestingPoseDialog::calcRandomCfg(std::vector<RigidBody::Ptr> &bodies, rw::kinematics::State& state){
	//if( _ui->_inGroupsCheck->)

		// TODO: pick a stable object pose.
		const double lowR = Deg2Rad * ( _ui->_lowRollSpin->value() );
		const double highR = Deg2Rad * ( _ui->_highRollSpin->value() );
		const double lowP = Deg2Rad * ( _ui->_lowPitchSpin->value() );
		const double highP = Deg2Rad * ( _ui->_highPitchSpin->value() );
		const double lowY = Deg2Rad * ( _ui->_lowYawSpin->value() );
		const double highY = Deg2Rad * ( _ui->_highYawSpin->value() );
		double roll = Math::ran(lowR, highR);
		double pitch = Math::ran(lowP, highP);
		double yaw = Math::ran(lowY, highY);
		Rotation3D<> rrot = RPY<>(roll,pitch,yaw).toRotation3D();

		double x = _ui->_xSpinBox->value();
		double y = _ui->_ySpinBox->value();
		double z = _ui->_zSpinBox->value();
		Vector3D<> rpos(Math::ran(-x,x), Math::ran(-y,y),Math::ran(-z,z));

		// choose a preshape
	 /*   _hand->getModel().setQ(_preshapes[0], state);
	*/
		// so we want to place the hand in a random configuration on the sphere with radius
		// r and center c
		Transform3D<> bTo = Kinematics::frameTframe(_handBase, _object, state);
		Transform3D<> wTo = Kinematics::worldTframe(_object, state);
		//double r = MetricUtil::norm2(bTo.P());


		// we calculate the center of the object
		bTo.P() = bTo.P() + bTo.R()*_body->getInfo().masscenter;

		// and we find a random rotation and apply it
		Rotation3D<> hR = RPY<>(Math::ran(-Pi/2,Pi/2),
								Math::ran(-Pi/2,Pi/2),
								Math::ran(-Pi/2,Pi/2)).toRotation3D()/* * bTo.R()*/;
		hR = rrot;

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

		if( _ui->_colFreeStart->isChecked() ){
			calcColFreeRandomCfg(state);
		} else {
			calcColFreeRandomCfg(state);
			//calcRandomCfg(_bodies, state);
		}
}
