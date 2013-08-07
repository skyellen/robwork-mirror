#include "SerialDeviceController.hpp"

#include <rw/trajectory/CubicSplineFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rwsim/util/RecursiveNewtonEuler.hpp>

#include <rw/common/macros.hpp>
#include <rw/math/Wrench6D.hpp>

using namespace rwsim::control;
using namespace rwsim::dynamics;

using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;
using namespace rw::kinematics;

SerialDeviceController::SerialDeviceController(
		const std::string& name, dynamics::DynamicDevice::Ptr ddev):
	_ddev(ddev),
	_currentQ(Q::zero(ddev->getModel().getDOF())),
	_currentQd(Q::zero(ddev->getModel().getDOF())),
	_time(0.0),
	_name(name),
	_q_error_last(ddev->getModel().getDOF()),
	_q_error(ddev->getModel().getDOF()),
	_targetAdded(false),
	_linVelMax(1), // default 1 m/s
	_angVelMax(Pi) // default Pi rad/s
{
	RW_WARN("creating solver");
	_solver = ownedPtr( new rw::invkin::JacobianIKSolver(ddev->getKinematicModel(), ddev->getStateStructure()->getDefaultState()) );
	_rdev = ddev.cast<rwsim::dynamics::RigidDevice>();
	for(int i=0;i<ddev->getModel().getDOF();i++)
		_q_error_last[i] = 0;
	_q_error = _q_error_last;
	_out.open("serialdev-out.txt");
}


SerialDeviceController::SerialDeviceController(
		const std::string& name, dynamics::RigidDevice::Ptr ddev):
	_ddev(ddev),
	_rdev(ddev),
	_currentQ(Q::zero(ddev->getModel().getDOF())),
	_currentQd(Q::zero(ddev->getModel().getDOF())),
	_time(0.0),
	_name(name),
	_q_error_last(ddev->getModel().getDOF()),
	_q_error(ddev->getModel().getDOF()),
	_targetAdded(false),
	_linVelMax(1), // default 1 m/s
	_angVelMax(Pi) // default Pi rad/s
{
	RW_WARN("creating solver");
	_solver = ownedPtr( new rw::invkin::JacobianIKSolver(ddev->getKinematicModel(), ddev->getStateStructure()->getDefaultState()) );
	for(int i=0;i<ddev->getModel().getDOF();i++)
		_q_error_last[i] = 0;
	_q_error = _q_error_last;
	_out.open("serialdev-out.txt");

}

SerialDeviceController::~SerialDeviceController(){

}

bool SerialDeviceController::moveLin(const rw::math::Transform3D<>& target, float speed, float blend){
	Target cmd_target;
	cmd_target.type = Lin;
	cmd_target.lin_target = target;
	cmd_target.speed = speed * 1.0/100.0;
	cmd_target.blend = blend;
	addTarget(cmd_target);
	return true;
}

bool SerialDeviceController::movePTP(const rw::math::Q& target, float speed, float blend)
{
	RW_WARN("Setting PTP target!");
	// set a joint target
	Target cmd_target;
	cmd_target.type = PTP;
	cmd_target.q_target = target;
	cmd_target.speed = speed * 1.0/100.0;
	cmd_target.blend = blend;
	addTarget(cmd_target);
	return true;
}

bool SerialDeviceController::movePTP_T(const rw::math::Transform3D<>& target, float speed, float blend)
{
	// set a joint target
	Target cmd_target;
	cmd_target.type = PTP_T;
	cmd_target.lin_target = target;
	cmd_target.speed = speed * 1.0/100.0;
	cmd_target.blend = blend;
	addTarget(cmd_target);
	return true;

}

bool SerialDeviceController::moveVelQ(const rw::math::Q& target){
	// set a joint target
	Target cmd_target;
	cmd_target.type = VelQ;
	cmd_target.q_vel_target = target;
	addTarget(cmd_target);
	return true;
}

bool SerialDeviceController::moveVelT(const rw::math::VelocityScrew6D<>& target){
	// set a joint target
	Target cmd_target;
	cmd_target.type = VelT;
	cmd_target.lin_vel_target = target;
	addTarget(cmd_target);
	return true;
}


bool SerialDeviceController::moveLinFC(const rw::math::Transform3D<>& target,
						  const rw::math::Wrench6D<>& wtarget,
						  float selection[6],
						  std::string refframe,
						  rw::math::Transform3D<> offset,
						  float speed,
						  float blend)
{

	RW_WARN("1");
	Target cmd_target;
	cmd_target.type = LinFC;
	_bXd = target;
	_bFd = wtarget;
	for(int i=0;i<6;i++)
		_S[i]=selection[i];
	// todo: set the refframe and offset
	_taskFrame = _ddev->getKinematicModel()->getEnd();


	cmd_target.lin_target = target;
	cmd_target.speed = speed * 1.0/100.0;
	cmd_target.blend = blend;
	addTarget(cmd_target);
	return true;
}

void SerialDeviceController::flushQueue(){

}

bool SerialDeviceController::stop(){
	_stop = true;
	return true;
}


bool SerialDeviceController::pause(){
	_pause = true;
	return true;
}


bool SerialDeviceController::setSafeModeEnabled(bool enable){
	RW_THROW("Not implemented yet!");
	return false;
}

rw::math::Q SerialDeviceController::getQ(){
	return _currentQ;
}

rw::math::Q SerialDeviceController::getQd(){
	return _currentQd;
}

bool SerialDeviceController::isMoving(){
	return _currentQd.normInf()>0.001 || _targetQueue.size()>0;
}

void SerialDeviceController::addTarget(const Target& target){
	{
		boost::mutex::scoped_lock lock(_targetMutex);
		_targetAdded = true;
		_targetQueue.push_back(target);
		_targetQueue.back().id = _idCnt;
		_idCnt++;
	}
}

namespace  {

}

//! create trajectory
SerialDeviceController::CompiledTarget SerialDeviceController::makeTrajectory(const std::vector<Target>& targets, rw::kinematics::State& initstate)
{
	RW_WARN("Make trajectory!");

	Q velLimits = _ddev->getKinematicModel()->getVelocityLimits();

	// we use 1.2 because we do not include velocity ramps, hence this reduces the speed a bit
	WeightedInfinityMetric<Q>::Ptr metric = MetricFactory::makeWeightedInfinity<Q>( 1.2/velLimits );
	std::cout << "weight: " << (1.2/velLimits) << std::endl;
	Metric<Transform3D<> >::Ptr t3d_metric = MetricFactory::makeTransform3DMetric<double>(1.2/_linVelMax, 1.2/_angVelMax  );
	rw::kinematics::State state = initstate;
	std::vector< Target > sequence;

	std::vector< rw::trajectory::QTrajectory::Ptr > trajectories;
	// first create the sequence of targets that fit together
	Q lastQ = _ddev->getKinematicModel()->getQ(initstate);
	Q lastQd = _currentQd;
	Transform3D<> lastT = _ddev->getKinematicModel()->baseTend(initstate);

	if(targets.back().type==LinFC){
		CompiledTarget ctarget;
		ctarget.ftcontrol = true;

		// todo: here we need to use cubic splines or something like it. Currently the "start" velocity will not be taken into account

		Metric<Transform3D<> >::Ptr t3d_metric = MetricFactory::makeTransform3DMetric<double>(1.2/_linVelMax, 1.2/_angVelMax  );
		double timeGuess = t3d_metric->distance(lastT, targets.back().lin_target);
		std::cout << "TIMEGUESS: " << timeGuess << std::endl;
		LinearInterpolator<Transform3D<> >::Ptr ramp =
	            rw::common::ownedPtr( new LinearInterpolator<Transform3D<> >( lastT , targets.back().lin_target,timeGuess /* *(100.0/targets.back().speed )*/ ));
	    InterpolatorTrajectory<Transform3D<> >::Ptr  ttraj = rw::common::ownedPtr( new InterpolatorTrajectory<Transform3D<> >() );
	    ttraj->add(ramp);
	    ctarget.t3dtraj = ttraj;

	    return ctarget;
	}


	for(int i=0; i<targets.size(); i++){
		sequence.clear();
		Target target = targets[i];
		target.q_start = lastQ;

		Transform3D<> lastT = _ddev->getKinematicModel()->baseTend(initstate);

		/////////////////////// THIS IS FOR POINT TO POINT SEGMENTS
		if(target.type == PTP_T){
			// use inverse kinematics to calculate q target
			target.type = PTP;
			_ddev->getKinematicModel()->setQ(lastQ, state);
			// calculate configuration close to the current configuration
			std::vector<Q> res = _solver->solve( target.lin_target, state );
			if(res.size()==0){
				RW_THROW("Could not calculate InvKin:" << lastQ << " --> "
													  << target.lin_target );
			}
			target.q_target = res[0];
		}

		// if PTP target then find all following PTP or PTP_T targets and make it a sequence
		if(target.type == PTP ){
			sequence.push_back( target );
			lastQ = sequence.back().q_target;

			// If there are more PTP or PTP_T in a row bundle them
			for(i+=1;i<targets.size();i++){
				Target ntarget = targets[i];
				ntarget.q_start = lastQ;
				if(ntarget.type==PTP_T){
					// use inverse kinematics to calculate q target
					ntarget.type = PTP;
					_ddev->getKinematicModel()->setQ(lastQ, state);
					// calculate configuration close to the current configuration
					std::vector<Q> res = _solver->solve( ntarget.lin_target, state );
					if(res.size()==0){
						RW_THROW("Could not calculate InvKin:" << lastQ << " --> "
															  << target.lin_target );
					}
					ntarget.q_target = res[0];
				}

				if(ntarget.type==PTP){
					sequence.push_back( ntarget );
					lastQ = sequence.back().q_target;
				} else {
					// only PTP and PTP_T targets can be in this sequence. So start a new if this is not compatible.
					break;
				}
			}


			// make PTP trajectory
			TimedQPath::Ptr path = ownedPtr( new rw::trajectory::TimedQPath() );
			double time = 0.0;
			path->push_back( TimedQ( 0.0, sequence[0].q_start ) );
			for(size_t k=0;k<sequence.size();k++){
				// make a guess on the time it will take to reach the point
				double timeGuess = metric->distance(sequence[k].q_start, sequence[k].q_target );
				std::cout << "timeguess: " << timeGuess << std::endl;

				time+=timeGuess/std::min(target.speed, 1.0f); // scale time such that speed is taken into account
				if (timeGuess < 1e-9)
					continue;
				path->push_back( TimedQ( time, sequence[k].q_target ) );
			}

			CompiledTarget ttarget;
			ttarget.toId = sequence.back().id;

			std::cout << "path size: " << path->size() << std::endl;
			std::cout << "Make trajectory from-to: "<< path->at(0).getValue() << " to " << path->at(1).getValue() << " time " << path->at(0).getTime() << " to " << path->at(1).getTime() << std::endl;
			std::cout << "last: "<< path->at(0).getValue() << " to " << path->back().getValue() << " time " << path->at(0).getTime() << " to " << path->back().getTime() << std::endl;
			ttarget.qtraj = CubicSplineFactory::makeClampedSpline(path, lastQd, Q::zero( lastQd.size() ));
			// finally reset the velocity in the end point
			lastQ = path->back().getValue();
			lastQd = Q::zero( lastQd.size() );
			return ttarget;
		}


		/////////////////////// THIS IS FOR LINEAR SEGMENTS
		if(target.type == Lin ){
			target.t_start = lastT;
			target.q_start = lastQ;
			sequence.push_back( target );
			std::vector<LinearInterpolator<Transform3D<> >::Ptr> linsequence;
			// find all Lin targets following this
			for(i+=1;i<targets.size();i++){
				Target ntarget = targets[i];
				ntarget.t_start = targets[i-1].lin_target;

				if(ntarget.type==Lin){
					sequence.push_back( ntarget );
					double timeGuess = t3d_metric->distance(ntarget.t_start, ntarget.lin_target);
					linsequence.push_back(
							ownedPtr(new LinearInterpolator<Transform3D<> >(ntarget.t_start, ntarget.lin_target, timeGuess*(100.0/ntarget.speed)) ) );

				} else if( ntarget.type==LinFC ) {
					sequence.push_back( ntarget );
					double timeGuess = t3d_metric->distance(ntarget.t_start, ntarget.lin_target);
					linsequence.push_back(
							ownedPtr(new LinearInterpolator<Transform3D<> >(ntarget.t_start, ntarget.lin_target, timeGuess*(100.0/ntarget.speed)) ) );
				} else {
					// only PTP and PTP_T targets can be in this sequence. So start a new if this is not compatible.
					break;
				}
			}

			// now we have a sequence of linear targets, now calculate blended trajectory

		}

	}
	CompiledTarget ttarget1;
	return ttarget1;

/*


	rw::trajectory::QTrajectory::Ptr traj;
	Q velLimits = _ddev->getKinematicModel()->getVelocityLimits();
	// we weigh with the inverse velLimits, such that distance (m) times 1/vel (s/m)
	// results in the fastest time taken to traverse the distance
	WeightedInfinityMetric<Q>::Ptr metric = MetricFactory::makeWeightedInfinity<Q>( 1.2/velLimits );

	//if(to-from == 0){
		// only one new target has been added.
		if( _targetQueue[to].type == PTP ){

			QPath::Ptr path = ownedPtr( new rw::trajectory::QPath() );
			path->push_back( _ddev->getKinematicModel()->getQ(state) );
			path->push_back( _targetQueue[to].q_target );

			std::cout << "Make trajectory from-to: "<< (*path)[0] << " to " << (*path)[1] << std::endl;
			// make a guess on the time it will take to reach the point
			double timeGuess = metric->distance(_ddev->getKinematicModel()->getQ(state), _targetQueue[to].q_target );
			traj = CubicSplineFactory::makeClampedSpline(path, _currentQd, Q::zero( _currentQd.size() ), timeGuess);

		} else if( _targetQueue[to].type == PTP_T ){
			Q next = _ddev->getKinematicModel()->getQ(state);
			// calculate configuration close to the current configuration
			std::vector<Q> res = _solver->solve( _targetQueue[to].lin_target, state );
			if(res.size()==0){
				RW_WARN("Could not calculate InvKin:" << next << " --> "
													  << _targetQueue[to].lin_target );
			} else {
				next = res[0];
			}
			QPath::Ptr path = ownedPtr( new rw::trajectory::QPath() );
			path->push_back( _ddev->getKinematicModel()->getQ(state) );
			path->push_back( next );

			double timeGuess = metric->distance(_ddev->getKinematicModel()->getQ(state), next );
			traj = CubicSplineFactory::makeClampedSpline(path, _currentQd, Q::zero( _currentQd.size() ), timeGuess);

		}

	//}

*/
	// optimize the trajectory such that velocity and acceleration limits are within bounds

/*
	if(trajectories.size()==1)
		return trajectories[0];

	for(int i=0;i<trajectories.size();i++)

	return traj;
	*/
}

namespace {
/*
	Eigen::MatrixXd calcJointSpaceInertiaMatrix( rwsim::dynamics::RigidDevice::Ptr ddev, const rw::kinematics::State& state){
	    Eigen::MatrixXd Mq(ddev->getKinematicModel()->getDOF(), ddev->getKinematicModel()->getDOF() );

		// compute torques imposed by gravity
		std::vector<Body::Ptr> links = ddev->getLinks();
		std::vector<rw::models::Joint*> joints = ddev->getJointDevice()->getJoints();
		//std::cout << links.size() << ">" << joints.size() << std::endl;
		rw::math::Q tauCompensate(joints.size());
		rw::kinematics::Frame* base = ddev->getKinematicModel()->getBase();
		// go through all links and compute their contribution to the torque
		for(int i=0;i<joints.size();i++){
			// go through all links and compute their contribution to the torque
			Transform3D<> bTj = rw::kinematics::Kinematics::frameTframe(base,joints[i], state);
			double jT = 0;
			for(int j=i;j<links.size();j++){
				const double& mass = links[j]->getInfo().mass;
				const double& cm = links[j]->getInfo().masscenter;

				Transform3D<> bTl = rw::kinematics::Kinematics::frameTframe(base, links[j]->getBodyFrame(), state);
				Transform3D<> jTl_cm = rw::kinematics::Kinematics::frameTframe(joints[i],links[j]->getBodyFrame(), state);
				jTl_cm.P() += jTl_cm.R() * cm;

				Mq(i,i) += inertia_tensor*dir + mass*Math::sqr(jTl_cm.P().norm2());

				Vector3D<> jF = inverse(bTj) * (links[j]->getInfo().mass*Vector3D<>(0,0,-9.82) );
				jT += cross( jTl.P()+ jTl.R()*links[j]->getInfo().masscenter, jF)[2]; // only take the z component
				//std::cout << cross( jTl.P()+ jTl.R()*links[j]->getInfo().masscenter, jF) << std::endl;
			}
			tauCompensate[i] = jT;
		}




		return Mq;
	}
*/
}

void SerialDeviceController::updateFTcontrolWrist(
		const rwlibs::simulation::Simulator::UpdateInfo& info,
			 rw::kinematics::State& state)
{
	// This is the hybrid force torque controller with Wrist based FT feedback
    if(!info.rollback){
    	_q_error_last = _q_error;
    }

    // define gain matrixes
	// selection matrix for position and for force, if 0 then force control is used.
    // Else S will be the positional gain. S is  selection matrix for position, Sf for force
	Eigen::Matrix<double,6,6> Kp = Eigen::Matrix<double,6,6>::Identity(); // positional error gain
	Eigen::Matrix<double,6,6> Kv = Eigen::Matrix<double,6,6>::Identity(); // velocity error gain
	Eigen::Matrix<double,6,6> Sf = Eigen::Matrix<double,6,6>::Identity();

	for(int i=0;i<6;i++){
		Sf(i,i) = 0;
		Kp(i,i) = _S[i];
		Kv(i,i) = 0.0; // TODO: select good damping parameter
		if(_S[i]==0){
			Kp(i,i) = 0;
			Sf(i,i) = 1.0;
		}
	}




	// todo: specify task frame, current position target pose and force target
	rw::kinematics::Frame *taskFrame = _taskFrame;

    // compute the jacobian
	Jacobian J = _ddev->getKinematicModel()->baseJframe( taskFrame , state );
	Q q = _ddev->getKinematicModel()->getQ(state);
	Q dq = _ddev->getJointVelocities(state);

	// targets
	const Transform3D<> bX_t = _executingTarget.t3dtraj->x( _currentTrajTime );
	const Transform3D<> bXd_t = _executingTarget.t3dtraj->dx( _currentTrajTime );
	const VelocityScrew6D<> bXdd_t = VelocityScrew6D<>(_executingTarget.t3dtraj->ddx( _currentTrajTime ) );

	const Wrench6D<> bF_t = _executingTarget._wrenchTarget;

	// current configuration
	const Transform3D<> bX_e = rw::kinematics::Kinematics::frameTframe(_ddev->getKinematicModel()->getBase(), taskFrame, state );
	const VelocityScrew6D<> bXd_e = J*dq;
	// todo: get wrist force/torque sensor data
	const Wrench6D<> bF_e;

	// calculate the pose error
    VelocityScrew6D<> bXe_err = bX_e.R() * VelocityScrew6D<>( inverse(bX_e) * bX_t );
    VelocityScrew6D<> bXde_err = bXd_e - _bXde_last;

    // update the state variables
    if(!info.rollback){
    	_bXe_last = bX_e;
    	_bXde_last = bXd_e;
    }

    // we add the gain to the cartesean velocity and pose error
    Eigen::Matrix<double, 6, 1> E = bXe_err.e();
    Eigen::Matrix<double, 6, 1> Ed = bXde_err.e();
    // Kp is the positional error gain, and Kv is the volocity error gain matrix
    //Eigen::Matrix<double, 6, 1> Edd = bXdd_t.e() + Kp*E + Kv*Ed;


	Wrench6D<> env_w;
	env_w(0) = E[0] * 10/0.01;// 10N per 0.01m
	env_w(1) = E[1] * 10/0.01; ;// 10N per 0.01m
	env_w(2) = E[2] * 10/0.01; ;// 10N per 0.01m
	env_w(3) = E[3] * 1/(50.0*Deg2Rad); ;// 1Nm per 10 grader
	env_w(4) = E[4] * 1/(50.0*Deg2Rad); ;// 1Nm per 10 grader
	env_w(5) = E[5] * 1/(50.0*Deg2Rad); ;// 1Nm per 10 grader


	//Wrench6D<> comb_w = Sf*bF_t + Sfinv*env_w;


    // calculate compensation of robot

	rwsim::util::RecursiveNewtonEuler dsolver(_rdev);

    dsolver.setGravity( inverse(Kinematics::worldTframe(_rdev->getKinematicModel()->getBase(),state).R())*Vector3D<>(0,0,-9.82) );

    rw::math::Q ddq = rw::math::Q::zero(q.size());

    // compute a wrench to apply to the end effector
    // this is spring based
    dsolver.setEnvironment( env_w );

    // solve for the torques of the robot
    rw::math::Q tauCompensate = dsolver.solveMotorTorques(state, ddq, ddq);

    std::cout << "COMPENSATE: " << tauCompensate << std::endl;
	_rdev->setMotorForceTargets(tauCompensate, state);

//#define STIFFNESS_CONTROL
#ifdef STIFFNESS_CONTROL



	// position and velocity is transformed into acceleration error





//#else


	const double KP = 60.0;
	const double KD = 0;


	// position part is controlled using simple PD controller
	double dt_prev_inv=0;
	if(info.dt_prev>0.00000001)
		dt_prev_inv = 1.0/info.dt_prev;


	// scale with konstant, the smaller the link the smaller the konstant
	/*
	for(int i=0;i<q_error.size();i++){
		q_error*KP*(q_error.size()-i)/(1.0*q_error.size());
	}
*/
	//Q tau( q_error /*+ ( (_q_error_last-q_error)*KD )*dt_prev_inv*/ );
	/*
	_out << info.time
			<< "\t" << bXe[0] << "\t" << bXe[1] << "\t" << bXe[2] << "\t" << bXe[3] << "\t" << bXe[4] << "\t" << bXe[5]
			<< "\t" << tau[0] << "\t" << tau[1] << "\t" << tau[2] << "\t" << tau[3] << "\t" << tau[4]<< "\t" << tau[5] <<std::endl;
	*/
	_q_error = q_error;

	//_q_error_last = q_error;

	// force part is controlled using simple P controller
	//const double KPtau = 0.8;
	//tau += Q( tau_error*KPtau );



	// something else....

	// add gains to bXe
	//bXe
	_q_error = bXe.e();


	Eigen::VectorXd ft_error_d = -( (_q_error_last-_q_error)*KD )*dt_prev_inv;

	Eigen::VectorXd ft_error = _q_error;
	ft_error[0] *= 10/0.01; ;// 10N per 0.01m
	ft_error[1] *= 10/0.01; ;// 10N per 0.01m
	ft_error[2] *= 10/0.01; ;// 10N per 0.01m
	ft_error[3] *= 1/(50.0*Deg2Rad); ;// 1Nm per 10 grader
	ft_error[4] *= 1/(50.0*Deg2Rad); ;// 1Nm per 10 grader
	ft_error[5] *= 1/(50.0*Deg2Rad); ;// 1Nm per 10 grader


    Eigen::VectorXd tau_error_trans =J.e().transpose()*ft_error;
    // TODO: scale control output with regard to time step

	_rdev->setMotorForceTargets(Q(tau_error_trans)+Q(ft_error_d)-tauCompensate, state);
	//_rdev->setMotorForceTargets(Q(6,0,0,0,0,0,0), state);
#endif

	// now calculate position control law and force control law
	// for now we use





}



void SerialDeviceController::update(const rwlibs::simulation::Simulator::UpdateInfo& info,
			 rw::kinematics::State& state)
{
	//std::cout << "supdate" << std::endl;
	// we make the decision that any 3 future targets may be locked for further optimisation.
	// that is if current stack has 3 targets and the robot is executing these then only the blend path between
	// target 3 and target 4 can be modified. The blends from 1 to 2 or 2 to 3 will not be further optimized.


	// first we check if new targets have been added
	if(_targetAdded){
		RW_WARN("Update: Target added!");
		std::vector<Target> targets;
		{
			boost::mutex::scoped_lock lock(_targetMutex);
			_targetAdded = false;
			targets = _targetQueue;
			_targetQueue.clear();
		}


		// now do something intelligent with targets from _currentTargetIdx to lastTarget
		CompiledTarget traj = makeTrajectory(targets, state);
		if( traj.qtraj!=NULL ){
			std::cout << "Qtraj: " << traj.qtraj->startTime() << " --> " << traj.qtraj->endTime() << std::endl;
			std::cout << "Currtime: " << _currentTrajTime << std::endl;
			std::cout << "Start: " << traj.qtraj->x(traj.qtraj->startTime()) << std::endl;
			std::cout << "End: " << traj.qtraj->x(traj.qtraj->endTime())<< std::endl;
			/*for(int i=0;i<100;i++){

				std::cout << traj.qtraj->x( i*traj.qtraj->endTime()/100 ) << std::endl;
			}
			std::cout << std::endl;
			for(int i=0;i<100;i++){
				std::cout << traj.qtraj->dx( i*traj.qtraj->endTime()/100 ) << std::endl;
			}*/

		} else if(traj.t3dtraj!=NULL ){
			std::cout << "t3dtraj: " << traj.t3dtraj->startTime() << " --> " << traj.t3dtraj->endTime() << std::endl;
			std::cout << "Currtime: " << _currentTrajTime << std::endl;
			std::cout << "Start: " << traj.t3dtraj->x(traj.t3dtraj->startTime()) << std::endl;
			std::cout << "End: " << traj.t3dtraj->x(traj.t3dtraj->endTime())<< std::endl;
			for(int i=0;i<100;i++){

				std::cout << traj.t3dtraj->x( i*traj.t3dtraj->endTime()/100 ) << std::endl;
			}
			std::cout << std::endl;
			for(int i=0;i<100;i++){
				std::cout << traj.t3dtraj->dx( i*traj.t3dtraj->endTime()/100 ) << std::endl;
			}
		}
		_compiledTargets.push_back( traj );
	}


	// in the following we need to send velocity and force commands to the robot
	if( !info.rollback ){
		// if its not a roll back then we increase the current time, because a step was succesfully applied
		_currentTrajTime += info.dt_prev;
	}

	if( _executingTarget.isFinished(_currentTrajTime) ){

		// the trajectory is finished.
		// check if we should start another compiled target
		if(_compiledTargets.empty()){
			//RW_WARN("Finished and no compiled in queue! " << _currentTrajTime << "s");
			// if no targets are ready then Keep setting the velocity to zero.
			// TODO: we might need to make sure that the robot does not drift...
			_currentQ = _ddev->getQ(state);
			_currentQd = _ddev->getJointVelocities(state);
			_ddev->setMotorVelocityTargets(Q::zero(_ddev->getKinematicModel()->getDOF()), state);
			return;
		} else {
			//RW_WARN("Target avail!");
			_executingTarget = _compiledTargets.front();
			_currentTrajTime = 0;
			_compiledTargets.pop_front();
		}
	}
	//RW_WARN("goto exe");
	if(_executingTarget.qtraj != NULL ){
		//RW_WARN("exe target running!");
		// else we need to follow the trajectory
		_currentQ = _ddev->getQ( state );
		_currentQd = _ddev->getJointVelocities( state );

		Q current_target_q = _executingTarget.qtraj->x( _currentTrajTime ); // the target that we should be in
		Q current_target_vel = _executingTarget.qtraj->dx( _currentTrajTime );

		Q next_target_q = _executingTarget.qtraj->x( _currentTrajTime + info.dt );
		Q next_target_vel = _executingTarget.qtraj->dx( _currentTrajTime + info.dt );

		// now calculate the velocity of the robot such that we reach next_target_q in the next timestep
		Q target_vel = (next_target_q - _currentQ)/info.dt - _currentQd;
		//std::cout << _currentTrajTime << ", "<< next_target_vel << std::endl;
		//_ddev->setMotorVelocityTargets( target_vel, state);
		_ddev->setMotorVelocityTargets( next_target_vel, state);
	} else if(_executingTarget.ftcontrol ){
		//
		std::cout << " ftcontrol " << std::endl;
		updateFTcontrolWrist(info, state);
	}
}

void SerialDeviceController::setMaxLinearVelocity( double maxVel ){
	_linVelMax = maxVel;
}

void SerialDeviceController::setMaxAngularVelocity( double maxVel ){
	_angVelMax = maxVel;
}

