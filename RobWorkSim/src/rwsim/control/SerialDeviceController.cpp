#include "SerialDeviceController.hpp"

#include <rw/trajectory/CubicSplineFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/math/MetricFactory.hpp>

#include <rw/common/macros.hpp>

using namespace rwsim::control;

using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;

SerialDeviceController::SerialDeviceController(
		const std::string& name, dynamics::DynamicDevice::Ptr ddev):
	_ddev(ddev),
	_currentQ(Q::zero(ddev->getModel().getDOF())),
	_currentQd(Q::zero(ddev->getModel().getDOF())),
	_time(0.0),
	_name(name),
	_targetAdded(false),
	_linVelMax(1), // default 1 m/s
	_angVelMax(Pi) // default Pi rad/s
{
	RW_WARN("creating solver");
	_solver = ownedPtr( new rw::invkin::JacobianIKSolver(ddev->getKinematicModel(), ddev->getStateStructure()->getDefaultState()) );
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
						  rw::math::Wrench6D<>& wtarget,
						  float selection[6],
						  std::string refframe,
						  rw::math::Transform3D<> offset,
						  float speed,
						  float blend)
{
	Target cmd_target;
	cmd_target.type = PTP_T;
	cmd_target.lin_target = target;
	cmd_target.speed = speed * 1.0/100.0;
	cmd_target.blend = blend;
	addTarget(cmd_target);
	return true;
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
	return _currentQd.normInf()<0.001 && _targetQueue.size()==0;
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
SerialDeviceController::CompiledTarget SerialDeviceController::makeTrajectory(int from, rw::kinematics::State& initstate)
{
	int to = _targetQueue.size();
	Q velLimits = _ddev->getKinematicModel()->getVelocityLimits();
	// we use 1.2 because we do not include velocity ramps, hence this reduces the speed a bit
	WeightedInfinityMetric<Q>::Ptr metric = MetricFactory::makeWeightedInfinity<Q>( 1.2/velLimits );
	Metric<Transform3D<> >::Ptr t3d_metric = MetricFactory::makeTransform3DMetric<double>(1.2/_linVelMax, 1.2/_angVelMax  );
	rw::kinematics::State state = initstate;
	std::vector< Target > sequence;

	std::vector< rw::trajectory::QTrajectory::Ptr > trajectories;
	// first create the sequence of targets that fit together
	Q lastQ = _ddev->getKinematicModel()->getQ(initstate);
	Q lastQd = _currentQd;
	Transform3D<> lastT = _ddev->getKinematicModel()->baseTend(initstate);

	for(int i=from; i<to; i++){
		sequence.clear();
		Target target = _targetQueue[i];
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
			for(i+=1;i<to;i++){
				Target ntarget = _targetQueue[i];
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
				time+=timeGuess*(100.0/target.speed); // scale time such that speed is taken into account
				path->push_back( TimedQ( time, sequence[k].q_target ) );
			}

			CompiledTarget ttarget;
			ttarget.toId = sequence.back().id;

			//std::cout << "Make trajectory from-to: "<< (*path)[0] << " to " << (*path)[1] << std::endl;
			ttarget.qtraj = CubicSplineFactory::makeClampedSpline(path, lastQd, Q::zero( lastQd.size() ));
			// finally reset the velocity in the end point
			lastQ = path->back().getValue();
			lastQd = Q::zero( lastQd.size() );
			continue;
		}


		/////////////////////// THIS IS FOR LINEAR SEGMENTS
		if(target.type == Lin ){
			target.t_start = lastT;
			target.q_start = lastQ;
			sequence.push_back( target );
			std::vector<LinearInterpolator<Transform3D<> >::Ptr> linsequence;
			// find all Lin targets following this
			for(i+=1;i<to;i++){
				Target ntarget = _targetQueue[i];
				ntarget.t_start = _targetQueue[i-1].lin_target;

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


	// optimize the trajectory such that velocity and acceleration limits are within bounds
*/
/*
	if(trajectories.size()==1)
		return trajectories[0];

	for(int i=0;i<trajectories.size();i++)

	return traj;
	*/
}


void SerialDeviceController::update(const rwlibs::simulation::Simulator::UpdateInfo& info,
			 rw::kinematics::State& state)
{
	// we make the decision that any 3 future targets may be locked for further optimisation.
	// that is if current stack has 3 targets and the robot is executing these then only the blend path between
	// target 3 and target 4 can be modified. The blends from 1 to 2 or 2 to 3 will not be further optimized.


	// first we check if new targets have been added
	if(_targetAdded){
		RW_WARN("Target added!");
		int lastTarget;
		{
			boost::mutex::scoped_lock lock(_targetMutex);
			_targetAdded = false;
			lastTarget  = _targetQueue.size()-1;
		}
		// now do something intelligent with targets from _currentTargetIdx to lastTarget

		rw::trajectory::QTrajectory::Ptr traj; // = makeTrajectory(_currentTargetIdx, lastTarget, state);
		RW_ASSERT(true);
		// replace old trajectory with the new, and reset current time
		_currentTraj = traj;
		_currentTrajTime = traj->startTime();
	}

	// in the following we need to send velocity and force commands to the robot
	if( !info.rollback ){
		// if its not a roll back then we increase the current time, because a step was succesfully applied
		_currentTrajTime += info.dt_prev;
	}

	if(_currentTraj==NULL || _currentTraj->endTime()<=_currentTrajTime){
		// the trajectory is finished. Keep setting the velocity to zero.
		// TODO: we might need to make sure that the robot does not drift...
		_currentQ = _ddev->getQ(state);
		_currentQd = _ddev->getJointVelocities(state);
		_ddev->setMotorVelocityTargets(Q::zero(_ddev->getKinematicModel()->getDOF()), state);
		return;
	}

	// else we need to follow the trajectory
	_currentQ = _ddev->getQ( state );
	_currentQd = _ddev->getJointVelocities( state );

	Q current_target_q = _currentTraj->x( _currentTrajTime ); // the target that we should be in
	Q current_target_vel = _currentTraj->dx( _currentTrajTime );

	Q next_target_q = _currentTraj->x( _currentTrajTime + info.dt );

	// now calculate the velocity of the robot such that we reach next_target_q in the next timestep
	Q target_vel = (next_target_q - _currentQ)/info.dt - _currentQd;
	//std::cout << _currentTrajTime << ", "<< current_target_q << target_vel << std::endl;
	_ddev->setMotorVelocityTargets( target_vel, state);
}

void SerialDeviceController::setMaxLinearVelocity( double maxVel ){
	_linVelMax = maxVel;
}

void SerialDeviceController::setMaxAngularVelocity( double maxVel ){
	_angVelMax = maxVel;
}

