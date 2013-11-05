#include "FalconControl.hpp"
#include <bitset>
void FalconControl::step(ThreadSimulator* sim, const rw::kinematics::State& state){
	static int firstTime;      		//first time we get an error
	static bool graspingChange; 	//grasping change or nots
	static double lastTime = 0;
	//add new transform to total template transform
	Vector3D<> pos = falcon.getPosition().P() * 70;
	
	if( sim->getTime() <lastTime+0.1 )
	    return;

	lastTime = sim->getTime();


	//record the paces of scene
	/*
	if(!sim->isInError() ) {
		FKTable fk(state);
		vector<ContactPoint> contacts =  _engine->getContacts();
 		map<pair<string,string>,vector<ContactPoint> > cbodies =  _engine->getContactingBodies();
 		cout << "cbodies:  " << cbodies.size()<< " contacts:  " << contacts.size() << endl;
 		for (int i = 0; i < contacts.size(); ++i)
 			cout << contacts[i].p << endl;
 		
 		cout << "--------" << endl;
 		for (map<pair<string,string>,vector<ContactPoint> >::iterator it=cbodies.begin(); it!=cbodies.end(); ++it)
 		{
 			if (it->second.size() > 0)
			    cout << it->first.first << " - " << it->first.second << " => " << it->second[0].p << endl;
		}
    	cout << "end" << endl;
		vector< pair< int, Transform3D<> > > objectPoses;
		
		for(int i = 0 ; i < allBodies.size() ; ++i)
			objectPoses.push_back( make_pair( allBodies[i]->getBodyFrame()->getID(), fk.get(allBodies[i]->getBodyFrame()) ) );

		Pace * p = new Pace(sim->getTime()+totalTime, controller->getQ(), controllerPG->getQ(), objectPoses);
		rec.addPace(p);
	}
    */


	//If robot cannot do this step of action then stop the simulator and set the force.
	if (sim->isInError()) {
	    std::cout << "Simulator in error... what to do!" << std::endl;
		falcon.setStrongForce(true);
		totalTime += sim->getTime();
		sim->reset(state);
		return;
	}

	std::cout << "Pos: " << pos << std::endl;
    //get coodinates of TCP
    Transform3D<> baseToTCP = dev->baseTframe(_tcpFrame, state);
    //cancel the old command
    controller->stop();
    //give the new command

    Transform3D<> target = baseToTCP;

    // tool coordinates
    // the position should be in tool frame BUT with an orientation such that the
    // y-axis is always pointing down in the negative world z-axis

    // we project the direction of world z-axis onto the tool xy-plane
    Vector3D<> Wzaxis = inverse( baseToTCP.R() ) * Vector3D<>::z();
    double d = dot(Vector3D<>::z(),Wzaxis);
    Vector3D<> projPoint = Wzaxis + d*Vector3D<>::z();
    // now find a rotation that will rotate y-axis of tool into projPoint direction
    EAA<> yaxisToWzaxis( Vector3D<>::y(), normalize(projPoint) );
    Rotation3D<> yToz_tool = yaxisToWzaxis.toRotation3D();

    target.P() += baseToTCP.R() * inverse(yToz_tool) * pos;
    //target.R() = RPY<>(-180*Deg2Rad,0,-90*Deg2Rad).toRotation3D();
    target.R() = RPY<>(90*Deg2Rad,-90*Deg2Rad,0).toRotation3D() *
                 RPY<>(0,0,_rotateYaw*2*Deg2Rad).toRotation3D() *
                 RPY<>(0,_rotatePitch*2*Deg2Rad,0).toRotation3D();




    // base coordinates
    //target.P() += pos;

    Metric<Transform3D<> >::Ptr t3d_metric = MetricFactory::makeTransform3DMetric<double>(1.0, 1.0);
    //std::cout << "distance: " << t3d_metric->distance(baseToTCP, target) << std::endl;
    if(t3d_metric->distance(baseToTCP, target)>0.00001){
        std::cout << "Move: " << target.P() << std::endl;
        controller->movePTP_T( target , 70);
    }
	

	//grasping
    std::cout << "States: " << std::bitset<32>(falcon.getButtonStates() ) << "   " << falcon.getButtonStates()  << std::endl;

    bool grasp = falcon.getGrasping();
    if( grasp != graspingChange) {
	    std::cout << "Grasping" << std::endl;
		graspingChange = grasp; // falcon.getGrasping();
		std::cout << "Setting target : " << Q(1 , graspingChange*MAX_GRASPING_VALUE) << std::endl;
		controllerPG->setTargetPos( Q(1 , graspingChange*MAX_GRASPING_VALUE) );
	}
}


FalconControl::FalconControl():
    RobWorkStudioPlugin("FalconControl", QIcon(":/pa_icon.png")),
    recordCount(0),
    totalTime(0.0),
    _rotateYaw(0),
    _rotatePitch(0)
{
    setupUi(this);
    
    // now connect stuff from the ui component
    connect(_startBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_stopBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    // this is the timer used to make events where we poll the simulator for information
    _timer = new QTimer( NULL );
    _timer->setInterval( 100 );

    connect( _timer, SIGNAL(timeout()), this, SLOT(btnPressed()) );

    _startBtn->setEnabled(false);
    _stopBtn->setEnabled(false);
}

FalconControl::~FalconControl()
{
    delete _timer;
}

void FalconControl::keyEventListener(int key, Qt::KeyboardModifiers modifier){
    std::cout << "Key pressed:  " << key << std::endl;
    if( key==65 ){ // A
        _rotateYaw++;
    } else if( key==68) { //D
        _rotateYaw--;
    } else if( key==87) {
        _rotatePitch++;

    } else if (key==83) {
        _rotatePitch--;
    }

}

void FalconControl::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(
            boost::bind(&FalconControl::stateChangedListener, this, _1), this);

    getRobWorkStudio()->genericEvent().add(
          boost::bind(&FalconControl::genericEventListener, this, _1), this);

    getRobWorkStudio()->keyEvent().add(
          boost::bind(&FalconControl::keyEventListener, this, _1, _2), this);

    Log::setLog( _log );
}

void FalconControl::startSimulation() {

    // create simulator and stuff here
    
    dev = _dwc->getWorkcell()->findDevice( "UR-6-85-5-A" );
	controller = _dwc->findController<SerialDeviceController>( "URController" );
	controllerPG = _dwc->findController<PDController>( "PG70GraspController" );
    _tcpFrame = _dwc->getWorkcell()->findFrame( "UR-6-85-5-A.TCP" );
    allBodies = _dwc->getBodies();
    
        
    State state = getRobWorkStudio()->getState();
    
    if(_sim == NULL) {
			
        _engine = ownedPtr( new ODESimulator(_dwc));
    	_engine->setContactLoggingEnabled( true );
        _sim = ownedPtr( new DynamicSimulator(_dwc, _engine ));
  		
        try 
        {
            _sim->init(state);
        } 
        catch(...)
        {
        	
        }
        
        _tsim = ownedPtr( new ThreadSimulator(_sim, state) );
        ThreadSimulator::StepCallback cb( boost::bind(&FalconControl::step, this, _1, _2) );
        _tsim->setStepCallBack( cb );
        _tsim->setRealTimeScale(1.0);
        _tsim->setTimeStep(0.01);

    } else
        _tsim->reset(state);


	// start the falcon device	
	falcon.start();
    // start poll timer
    _timer->start();
    // start simulator
    _tsim->start();
}

void FalconControl::open(WorkCell* workcell)
{
    RW_WARN("open");
    if(workcell==NULL || _dwc==NULL)
        return;
    RW_WARN("open");
    _startBtn->setEnabled(true);
    RW_WARN("open");
    _wc = workcell;
    RW_WARN("open");
}

void FalconControl::close() {
    // destroy simulation and stuff
    _wc = NULL;
}

void FalconControl::saveFile(){
	stringstream ss;
	ss<<"path_"<<recordCount++;
	string filename = ss.str();
	rec.save(filename);
	totalTime = 0.0;
}

void FalconControl::btnPressed() {
	QObject *obj = sender();
	if(obj==_startBtn)
	{
		startSimulation();
		_startBtn->setEnabled(false);
		_stopBtn->setEnabled(true);
	} 
	else if(obj==_stopBtn)
	{
		_startBtn->setEnabled(true);
		_stopBtn->setEnabled(false);
		falcon.isThreadStopped = true;
		_tsim->stop();
		_timer->stop();
		saveFile();
	} 
	else if(obj==_timer)
	{
		// update the RobWorkStudio state
		// poll the simulator state and update the visualization state
		getRobWorkStudio()->setState( _tsim->getState() );
	}
}

void FalconControl::stateChangedListener(const State& state) {
	_state = state;
}


void FalconControl::genericEventListener(const std::string& event){
    if( event=="DynamicWorkCellLoaded" ){
        // get the dynamic workcell from the propertymap
        RW_DEBUG("Getting dynamic workcell from propertymap!");
        DynamicWorkCell::Ptr dwc =
            getRobWorkStudio()->getPropertyMap().get<DynamicWorkCell::Ptr>("DynamicWorkcell",NULL);

        if( dwc==NULL){
            log().error() << "Could not load dynamic workcell from propertymap!!" << std::endl;
            return;
        }
        _dwc = dwc;
    }
}

Q_EXPORT_PLUGIN(FalconControl);
