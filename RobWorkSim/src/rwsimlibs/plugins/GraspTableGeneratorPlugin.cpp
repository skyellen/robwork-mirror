#include "GraspTableGeneratorPlugin.hpp"

#include <sstream>
#include <string>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>

#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>

#include <rws/RobWorkStudio.hpp>

#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>

#include <rwsim/util/GraspPolicyFactory.hpp>
#include <rwsim/util/GraspStrategyFactory.hpp>

#include "ui_GraspTableGeneratorPlugin.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>

using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsim::sensor;
using namespace rwsim::control;
using namespace rwsim::util;

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::loaders;
using namespace rw::sensor;
using namespace rw::trajectory;
using namespace rw::models;
using namespace rw::geometry;
using namespace rw::graspplanning;
using namespace rwlibs::simulation;

using namespace boost::numeric::ublas;

#define RW_DEBUGS( str ) std::cout << str  << std::endl;

namespace {


}

namespace {

	class TimeLabel {
		TimeLabel(){}

		void setTimeInMs(long val){ ms=val; }
		void setTimeInSec(long val){ ms=val*1000;}
		void setTimeInMin(long val){ ms=val*1000*60;}

		std::string toString(){
			//long sec;
			long min, hours, days;
			days = ms%1000*60*24;
			hours = (ms-days*1000*60*24)%1000*60;
			min = (ms-days*(hours*1000*60)*24)%60;
			//sec = (ms-days*24*(hours*60*(min*60)))%1000;

			std::stringstream sstr;
			if(days>0)
				sstr << days << "days ";
			if(days>0 || hours>0 )
				sstr << hours << ":";
			if(days>0 || hours>0 || min>0)
				sstr << min << ":";
			return sstr.str();
		}

		long ms;
	};

}


GraspTableGeneratorPlugin::GraspTableGeneratorPlugin():
    RobWorkStudioPlugin("GTableGen", QIcon(":/SimulationIcon.png")),
    _dwc(NULL),
    _avgSimTime(10),
    _avgTime(10)
{
    _ui = new Ui::GraspTableGeneratorPlugin();
    _ui->setupUi( this );

    Math::seed( TimerUtil::currentTimeMs() );

    std::vector<std::string> policies = GraspPolicyFactory::getAvailablePolicies();
    BOOST_FOREACH(const std::string& id, policies){
        _ui->_gPolicyBox->addItem(id.c_str());
    }


    std::vector<std::string> strategies = GraspStrategyFactory::getAvailableStrategies();
    BOOST_FOREACH(const std::string& id, strategies){
        _ui->_gStrategyBox->addItem(id.c_str());
    }

    //_gStrategyBox->addItem("Preshape");
    //_gStrategyBox->addItem("Preshape - Random");

    //_gPolicyBox->addItem("Position");// close to predefined position
    //_gPolicyBox->addItem("Fixed Velocity");// close with constant velocity
    //_gPolicyBox->addItem("Fixed Force");// close with constant velocity

    _ui->_qAvailMetricsBox->addItem("CM-CCP");
    _ui->_qMetricsBox->addItem("Force closure (LEB)");

    RW_DEBUGS("- Setting connections ");
    connect(_ui->_saveBtn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_genTableBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_resetBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_stopBtn     ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_simulatorBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_scapeBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_ui->_loadGTableConfig    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_saveGTableConfig    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_addBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_removeBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_browseOutputBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_genTableBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_stopBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    //connect(_updateRateSpin,SIGNAL(valueChanged(int)), this, SLOT(changedEvent()) );
    //connect(_fixedGroupBox, SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );

    RW_DEBUGS("- Setting timer ");
    _timer = new QTimer( NULL );
    _timer->setInterval( _ui->_updateRateSpin->value() );
    connect( _timer, SIGNAL(timeout()), this, SLOT(changedEvent()) );
}

GraspTableGeneratorPlugin::~GraspTableGeneratorPlugin(){

}

void GraspTableGeneratorPlugin::cleanup(){
    _ui->_deviceBox->clear();
    _ui->_objectBox->clear();
}

void GraspTableGeneratorPlugin::open(rw::models::WorkCell* workcell){

	if( workcell==NULL || _dwc==NULL )
		return;

	if( workcell!=_dwc->getWorkcell()){
		// a completely new workcell has been loaded, discard dwc and exit
		_dwc = NULL;
		cleanup();
		return;
	}

	RW_DEBUGS("- Setting devices ");
	std::vector<DynamicDevice::Ptr> devices = _dwc->getDynamicDevices();
	BOOST_FOREACH(DynamicDevice::Ptr device, devices){
		if(dynamic_cast<RigidDevice*>(device.get())){
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
}


void GraspTableGeneratorPlugin::loadConfiguration(const std::string& file){
	std::string configFile;
	if( file.empty() ){
	    QString selectedFilter;
	    std::string dirname =
	    		getRobWorkStudio()->getPropertyMap().get<std::string>("PreviousDirectory","");
	    const QString dir(dirname.c_str());

	    QString filename = QFileDialog::getOpenFileName(
	        this,
	        "Open GTable config file", // Title
	        dir, // Directory
	        "All supported ( *.xml )"
	        " \nRW XML files ( *.xml )"
	        " \n All ( *.* )",
	        &selectedFilter);

	    configFile = filename.toStdString();
	} else {
		configFile = file;
	}

	if (configFile.empty()){
		return;
	}

	getRobWorkStudio()->getPropertyMap().set<std::string>(
			"PreviousDirectory",
			rw::common::StringUtil::getDirectoryName(configFile));

    PropertyMap map;
    try {
        map = XMLPropertyLoader::load(configFile);
    } catch (const Exception& exp) {
        QMessageBox::information(
            NULL,
            "Error loading propertymap!",
            exp.getMessage().getText().c_str(),
            QMessageBox::Ok);
        return;
    }

    _config = map;

    // apply configuration
    applyConfiguration();
}

void GraspTableGeneratorPlugin::btnPressed(){
    QObject *obj = sender();

    if( obj == _ui->_loadGTableConfig ){loadConfiguration("");}
    else if(obj == _ui->_saveGTableConfig){saveConfiguration("");}
    //else if(){ }
    else if(obj == _ui->_genTableBtn){startTableGeneration();}
    else if(obj == _ui->_stopBtn){
    	if(_ui->_stopBtn->text()=="Pause"){
    	    _ui->_stopBtn->setText("Continue");
    		// todo stop execution
    	} else {
    	    _ui->_stopBtn->setText("Pause");
    	}
    }
    else if(obj == _ui->_addBtn){
    	QString txt = _ui->_qAvailMetricsBox->currentText();
    	if(txt.isEmpty()) return;
    	_ui->_qMetricsBox->addItem(txt);
    	_ui->_qAvailMetricsBox->removeItem(_ui->_qAvailMetricsBox->currentIndex());
    } else if(obj == _ui->_removeBtn){
    	QString txt = _ui->_qMetricsBox->currentText();
    	if(txt.isEmpty()) return;
    	_ui->_qAvailMetricsBox->addItem(txt);
    	_ui->_qMetricsBox->removeItem(_ui->_qMetricsBox->currentIndex());
    } else if(obj == _ui->_browseOutputBtn){
    	QString selectedFilter;
        std::string dirname =
        		getRobWorkStudio()->getPropertyMap().get<std::string>("PreviousDirectory","");
        const QString dir(dirname.c_str());
        QString filename = QFileDialog::getSaveFileName(
			this,
			"Name of Grasp Table data file", // Title
			dir, // Directory
			"All supported ( *.gdata )"
			" \nRW Grasp Data files ( *.gdata )"
			" \n All ( *.* )",
			&selectedFilter);

        _ui->_outputFileEdit->setText(filename);
    }
}

void GraspTableGeneratorPlugin::changedEvent(){

}

void GraspTableGeneratorPlugin::stepCallBack(int i, const rw::kinematics::State& state){

}

namespace {
	void setComboBox(const std::string& name, QComboBox *box, LogWriter& log, const std::string& errmsg){
		if(!name.empty()){
			int hidx = box->findText(name.c_str());
			if(hidx<0){
				log << errmsg << "\"" << name << "\" not found!";
			} else {
				box->setCurrentIndex(hidx);
			}
		}
	}
}

void GraspTableGeneratorPlugin::applyConfiguration(){
	std::string nameHand = _config.get<std::string>("HandName", "");
	std::string nameObject = _config.get<std::string>("ObjectName", "");

	std::string nameGraspPolicy = _config.get<std::string>("GraspPolicy", "");
	std::string nameGraspStrategy = _config.get<std::string>("GraspStrategy", "");

	std::string nameQMetric = _config.get<std::string>("QualityMetric", "");
	std::string nameQMetric1 = _config.get<std::string>("QualityMetric1", "");
	std::string nameQMetric2 = _config.get<std::string>("QualityMetric2", "");
	std::string nameQMetric3 = _config.get<std::string>("QualityMetric3", "");
	std::string nameQMetric4 = _config.get<std::string>("QualityMetric4", "");

	int table_size = _config.get<int>("TableSize", 1000);
	int group_size = _config.get<int>("GroupSize", 1);
	bool use_fixed_step = _config.get<bool>("FixedStep", false);
	bool use_continues_logging = _config.get<bool>("ContinuesTactileLogging", false);
	bool use_dynamic_simulation = _config.get<bool>("DynamicSimulation", true);
	bool use_colfree_initstate = _config.get<bool>("CollisionFreeInitState", true);

	// now set all values in the gui

	std::string output_file = _config.get<std::string>("OutputFile", "grasp_table_output.gdata");

	setComboBox(nameHand, _ui->_deviceBox, log().error(), "Hand ");
	setComboBox(nameObject, _ui->_objectBox, log().error(), "Object ");
	setComboBox(nameGraspPolicy, _ui->_gPolicyBox, log().error(), "Grasp Policy ");
	setComboBox(nameGraspStrategy, _ui->_gStrategyBox, log().error(), "Grasp Strategy ");

	_ui->_collisionFreeInitBox->setChecked(use_colfree_initstate);
	_ui->_dynamicSimulationBox->setChecked(use_dynamic_simulation);
	_ui->_continuesLoggingBox->setChecked(use_continues_logging);
	_ui->_fixedGroupBox->setChecked(use_fixed_step);

	_ui->_groupSizeSpin->setValue( group_size );
	_ui->_tableSizeSpin->setValue( table_size );

	_ui->_outputFileEdit->setText(output_file.c_str());
}

void GraspTableGeneratorPlugin::readConfiguration(){

	_config.set<std::string>("HandName", _ui->_deviceBox->currentText().toStdString());
	_config.set<std::string>("ObjectName", _ui->_objectBox->currentText().toStdString());

	_config.set<std::string>("GraspPolicy", _ui->_gPolicyBox->currentText().toStdString());
	_config.set<std::string>("GraspStrategy", _ui->_gStrategyBox->currentText().toStdString());


	std::string nameQMetric = _config.get<std::string>("QualityMetric", "");
	std::string nameQMetric1 = _config.get<std::string>("QualityMetric1", "");
	std::string nameQMetric2 = _config.get<std::string>("QualityMetric2", "");
	std::string nameQMetric3 = _config.get<std::string>("QualityMetric3", "");
	std::string nameQMetric4 = _config.get<std::string>("QualityMetric4", "");

	_config.set<int>("TableSize", _ui->_tableSizeSpin->value());
	_config.set<int>("GroupSize", _ui->_groupSizeSpin->value());
	_config.set<bool>("FixedStep", _ui->_fixedGroupBox->isChecked());
	_config.set<bool>("ContinuesTactileLogging", _ui->_continuesLoggingBox->isChecked());
	_config.set<bool>("DynamicSimulation", _ui->_dynamicSimulationBox->isChecked());
	_config.set<bool>("CollisionFreeInitState", _ui->_collisionFreeInitBox->isChecked());

	// now set all values in the gui
	_config.set<std::string>("OutputFile", _ui->_outputFileEdit->text().toStdString());
}


void GraspTableGeneratorPlugin::saveConfiguration(const std::string& file){
	readConfiguration();
	QString selectedFilter;
    std::string dirname =
    		getRobWorkStudio()->getPropertyMap().get<std::string>("PreviousDirectory","");
    const QString dir(dirname.c_str());
    QString filename(file.c_str());
    if(file.empty()){
		filename = QFileDialog::getSaveFileName(
			this,
			"Name of GTable config file", // Title
			dir, // Directory
			"All supported ( *.xml )"
			" \nRW XML files ( *.xml )"
			" \n All ( *.* )",
			&selectedFilter);
    }

    if(!filename.isEmpty()){
    	XMLPropertySaver::save(_config, filename.toStdString());
    }
}


void GraspTableGeneratorPlugin::close(){

}


void GraspTableGeneratorPlugin::initialize(){
    getRobWorkStudio()->stateChangedEvent().add(
    		boost::bind(&GraspTableGeneratorPlugin::stateChangedListener, this, _1), this);

    _configFile = getRobWorkStudio()->getPropertyMap().get<std::string>("GraspTableConfigFile","");
}


void GraspTableGeneratorPlugin::stateChangedListener(const rw::kinematics::State& state){

}


void GraspTableGeneratorPlugin::startTableGeneration(){
	std::cout << "Start table generation! " << std::endl;
	//
	//JointDevice *hand = _dwc->getWorkcell()->findDevice<JointDevice>( _deviceBox->currentText().toStdString() );
	MovableFrame *object = _dwc->getWorkcell()->findFrame<MovableFrame>( _ui->_objectBox->currentText().toStdString() );

	DynamicDevice *hand = _dwc->findDevice(_ui->_deviceBox->currentText().toStdString()).get();

	if( (hand==NULL) | (object==NULL) ){
		log().error() << "hand or object not found!" << std::endl;
		return;
	}

	std::string policyId = _ui->_gPolicyBox->currentText().toStdString();
	std::string strategyId = _ui->_gStrategyBox->currentText().toStdString();

	if(_gstrategy==NULL){
		_gstrategy = GraspStrategyFactory::makeStrategy(strategyId);
	}

	if(_gpolicy==NULL){
		_gpolicy = GraspPolicyFactory::makePolicy(policyId, _dwc.get(), hand);
	}

	// now create the simulations
	if(_ui->_dynamicSimulationBox->isChecked()){
	    StateSampler::Ptr sampler = _gstrategy->getSampler();
	    SimulatedController::Ptr controller = _gpolicy->getController();

	    _simulator->addController( controller );





	}

}





















#if 0
#include <rwsim/loaders/ScapePoseFormat.hpp>
#include <rwsim/sensor/TactileArraySensor.hpp>
namespace {

	std::vector<matrix<float> > getTactileData(const std::vector<SimulatedSensorPtr>& sensors){
		std::vector<matrix<float> > datas;
		BOOST_FOREACH(const SimulatedSensorPtr& sensor, sensors){
        	if( TactileArraySensor *tsensor = dynamic_cast<TactileArraySensor*>( sensor.get() ) ) {
                datas.push_back( tsensor->getTexelData() );
            }
        }
		return datas;
	}
}

void GraspTableGeneratorPlugin::stepCallBack(int i, const rw::kinematics::State& state){
    ThreadSimulatorPtr tsim = _simulators[i];

    SimulatorPtr sim = tsim->getSimulator();

    // if the simulation is not running then don't do anything
    bool isSimRunning = tsim->isRunning();
    if(!isSimRunning || _nrOfTests>=_nrOfTestsSpin->value()){
    	std::string str = _savePath->text().toStdString();
    	std::stringstream sstr;
    	sstr << str << "/" << "grasptablefile.txt";

    	_gtable.save(sstr.str());
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


    // save tactile data
    std::vector<SimulatedSensorPtr> sensors = sim->getSensors();
    std::vector<std::vector<Contact3D> > tactileContacts;
    BOOST_FOREACH(SimulatedSensorPtr& sensor, sensors){
        if( TactileArraySensor *tsensor = dynamic_cast<TactileArraySensor*>( sensor.get() ) ) {
            datas.push_back( tsensor->getTexelData() );

            float sum = 0;
            // we require samples on at least two of the fingers
            for(int i=0; i<datas.back().size1(); i++)
                for(int j=0; j<datas.back().size2(); j++)
                	if(datas.back()(i,j)>500)
                		sum += datas.back()(i,j);
            if( sum>1 ){
            	tactileContacts.push_back(tsensor->getActualContacts());
                fingersWithData++;
            }

        }
    }

    if( fingersWithData>0 ){
    	_tactileDataOnAllCnt++;
    } else {
    	_tactileDataOnAllCnt =0;
    }

    if( _continuesLogging->isChecked() ){

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

    if(!isSimFinished || time<_minTimeValidSpin->value())
        _lastBelowThresUpdate = time;

    if(i==0)
        _state = state;
    bool dataValid = false;
    if( time-_lastBelowThresUpdate >  _minRestTimeSpin->value() || _tactileDataOnAllCnt>50){
        // one simulation has finished...
    	_tactileDataOnAllCnt = 0;
        {
            int simidx = i;
            // calculate grasp quality
            rw::math::Q qualities( Q::zero(3) );
            Grasp3D g3d( _bodySensor->getContacts() );

            Transform3D<> wTf = Kinematics::worldTframe(_handBase, state);
            //std::cout << "***** NR OF CONTACTS IN GRASP: " << g3d.contacts.size() << std::endl;
            Vector3D<> cm = _body->getInfo().masscenter;
            std::cout << cm;
            //std::cout << "Wrench calc" << std::endl;
            if(g3d.contacts.size()>1){
                WrenchMeasure3D wmeasure(new QHull3D(), 6 );
                wmeasure.setObjectCenter(cm);
                wmeasure.quality(g3d);
                //std::cout << "Wrench calc done!" << std::endl;

                qualities(0) = wmeasure.getMinForce();
                qualities(1) = wmeasure.getMinTorque();

                CMDistCCPMeasure3D CMCPP( cm, 0.3);
                qualities(2) = CMCPP.quality( g3d );
            }

            Frame *world = _dwc->getWorkcell()->getWorldFrame();
            Transform3D<> wThb = Kinematics::worldTframe(_handBase, state);
            Transform3D<> hbTw = inverse(wThb);
            Transform3D<> wTo = Kinematics::worldTframe(_object, state);

            Transform3D<> hpTo = inverse(wThb)*wTo;

            Vector3D<> approach = hpTo * Vector3D<>(0,0,1);
            GraspTable::GraspData data;

            // all contacts are in world coordinates. Transform them...
            BOOST_FOREACH(std::vector<Contact3D>& convec, tactileContacts){
            	for(int i=0;i<convec.size();i++){
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

            data._tactiledata = getTactileData( sim->getSensors() );

            data.quality = qualities;
            data.grasp = g3d;
            if( _nrOfGraspsInGroup==1 && ( qualities(0)<0.1 || qualities(1)<=0 || qualities(2)<=0.1 || fingersWithData<3) ){
            	_nrOfGraspsInGroup = 0;
            	dataValid = false;
            } else if(_nrOfGraspsInGroup>0){
            	dataValid = true;
            	_gtable.addGrasp(data);
            	_nrOfTests++;

            	if(_nrOfTests-_lastTableBackupCnt>100){
                	std::string str = _savePath->text().toStdString();
                	std::stringstream sstr;
                	sstr << str << "/" << "grasptablefile" << _nrOfTests << ".txt";
                	_gtable.save(sstr.str());
                	_lastTableBackupCnt = _nrOfTests;
            	}

            }
        	_totalSimTime += time;
        }
        /*
        else if(saveRestingState(i, sim, state)){
        	_totalSimTime += time;
            _nrOfTests++;
        }
        */

        _tactiledatas[i].clear();
        _handconfigs[i].clear();
        _resultPoses.push_back(state);
        _startPoses.push_back(_initStates[i]);
        // recalc random start configurations and reset the simulator
        State nstate;
        _nextTimeUpdate[i] = 0;
        Q target = _target;
        Q preshape = _preshape;
    	if( _inGroupsCheck->isChecked()){
    		if(_nrOfGraspsInGroup == 0 || _nrOfGraspsInGroup==_groupSizeSpin->value()){
    			// generate a new random grasp
        		nstate = _defstate;
                _currentPreshapeIDX[i] = Math::ranI(0,_preshapes.size());
                _preshape = _preshapes[_currentPreshapeIDX[i]];
                _target = _targetQ[_currentPreshapeIDX[i]];
                target = _target;
                preshape = _preshape;

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
    			if( _fixedGroupBox->isChecked() ){
        			double devi = 0.005; //
        			if(_nrOfGraspsInGroup-1>5)
        				devi = Pi/90.0;
        			if(_nrOfGraspsInGroup-1&0x1)
        				devi = -devi;


        			std::cout << (int)floor(_nrOfGraspsInGroup/2.0-0.1) << std::endl;
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
    			if(_nrOfGraspsInGroup>_groupSizeSpin->value())
    				_nrOfGraspsInGroup = 0;

    		}
    	} else {

    		nstate = _defstate;
            _currentPreshapeIDX[i] = Math::ranI(0,_preshapes.size());
            preshape = _preshapes[_currentPreshapeIDX[i]];
            target = _targetQ[_currentPreshapeIDX[i]];
            calcRandomCfg(nstate);
            _initStates[i] = nstate;
    	}

        _hand->getModel().setQ(preshape, nstate);
        _controllers[i]->setTargetPos( target );
        _fingersInContact[i] = false;

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

void GraspTableGeneratorPlugin::btnPressed(){
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

void GraspTableGeneratorPlugin::changedEvent(){
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
    } else if(obj == _fixedGroupBox){
    	if( _fixedGroupBox->isChecked()){
    		_groupSizeSpin->setValue(13);
    		_groupSizeSpin->setEnabled(false);
    	} else {
    		_groupSizeSpin->setEnabled(true);
    	}
    }
}

bool GraspTableGeneratorPlugin::saveRestingState(int simidx, SimulatorPtr sim , const rw::kinematics::State& state ){
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
    if(fingersWithData<2){
        _tactiledatas[simidx].clear();
        _handconfigs[simidx].clear();

        return false;
    }
    // calculate grasp quality
    rw::math::Q qualities( Q::zero(3) );
    Grasp3D g3d( _bodySensor->getContacts() );
    if(g3d.contacts.size()<2){
        _tactiledatas[simidx].clear();
        _handconfigs[simidx].clear();
        return false;
    }
    Transform3D<> wTf = Kinematics::worldTframe(_handBase, state);
    BOOST_FOREACH(Contact3D &c, g3d.contacts){
        sstr << c.p << "  " << c.n << " ";
    }

    std::string desc = sstr.str();

    //std::cout << "***** NR OF CONTACTS IN GRASP: " << g3d.contacts.size() << std::endl;
    Vector3D<> cm = _body->getInfo().masscenter;
    std::cout << cm;
    //std::cout << "Wrench calc" << std::endl;
    WrenchMeasure3D wmeasure(new QHull3D(), 6 );
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
        //saveRestingPoseSimple(desc, _tactiledatas[simidx], qualities, isStable, preshapeId, preq, _handconfigs[simidx], approach, _file, squal.str());
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

void GraspTableGeneratorPlugin::updateStatus(){

    if( _generators.size()==0 )
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

void GraspTableGeneratorPlugin::calcColFreeRandomCfg(rw::kinematics::State& state){
    //std::cout << "-------- Col free collision: " << std::endl;
    // first calculate a random state
    CollisionResult colresult;
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
//#define CYLENDRICAL_APPROACH

void GraspTableGeneratorPlugin::genericEventListener(const std::string& event){
    //std::cout << "Generic event: " << event << std::endl;
    if( event=="DynamicWorkCellLoaded" ){
        // get the dynamic workcell from the propertymap
        RW_DEBUG("Getting dynamic workcell from propertymap!");

        Ptr<DynamicWorkCell> dwc =
			getRobWorkStudio()->getPropertyMap().get<Ptr<DynamicWorkCell> >("DynamicWorkcell",NULL);


        if( dwc==NULL){
        	log().error() << "Could not load dynamic workcell from propertymap!!" << std::endl;
        	return;
        }

        _dwc = dwc;
    }
}

#endif

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(GraspTableGeneratorPlugin)
#endif
