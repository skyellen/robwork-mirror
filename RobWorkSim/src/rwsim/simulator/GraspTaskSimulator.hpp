/*
 * GraspTaskSimulator.hpp
 *
 *  Created on: Jun 28, 2011
 *      Author: jimali
 */

#ifndef GRASPTASKSIMULATOR_HPP_
#define GRASPTASKSIMULATOR_HPP_

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/control/BodyController.hpp>
#include <rwlibs/control/JointController.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rw/sensor/Contact3D.hpp>
#include "DynamicSimulator.hpp"
#include "ThreadSimulator.hpp"
#include <stack>
#include <rwsim/dynamics/KinematicBody.hpp>

namespace rwsim {
namespace simulator {
/**
 * @brief A class for simulating multiple grasping tasks.
 *
 * A grasping task can be considered as a sequence of the following states
 * - initialize gripper joint values and pose
 * - approach the target "object"
 * - grasp target by closing fingers
 * - stop grasp when either fingers are still or there is a timeout
 * - calculate grasp success
 * - retract gripper
 * - calculate grasp success
 *
 * The home and target poses are controlled through a task description file. Which is
 * all so used to write back all the results of the simulation.
 *
 * The configuration of the simulation is setup through properties. These can be set from the
 * command prompt, loaded by file, or edited in the gui. These properties include:
 *
 * - Simulator
 * - TimeStepSize
 * - HandOpenConfig
 * - HandCloseConfig
 * - MinRestingTime
 *
 *
 */
class GraspTaskSimulator {
public:
	typedef rw::common::Ptr<GraspTaskSimulator> Ptr;

	//! the possible discrete outcomes of a single task simulation
	typedef enum Status {
		UnInitialized = 0,
        Success, CollisionInitially,
        ObjectMissed, ObjectDropped,
        ObjectSlipped, TimeOut,
        SimulationFailure,
        InvKinFailure,
        PoseEstimateFailure
	 } TestStatus;

public:

	GraspTaskSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dwc);

	virtual ~GraspTaskSimulator();

	/**
	 * @brief load tasks
	 */
	void load(const std::string& filename);

	/**
	 * @brief
	 * @param graspTasks
	 */
	void load(rwlibs::task::CartesianTask::Ptr graspTasks);

	// these are basically the same since the result is added to the loaded tasks
	rwlibs::task::CartesianTask::Ptr getTasks(){ return _roottask; };
	rwlibs::task::CartesianTask::Ptr getResult(){ return _roottask; }

	//! @brief get the number of targets
	size_t getNrTargets();
	ThreadSimulator::Ptr getSimulator();
	std::vector<ThreadSimulator::Ptr> getSimulators();

	//----- simulation control and query function api
	void init(rwsim::dynamics::DynamicWorkCell::Ptr dwc, const rw::kinematics::State& initState);
	void startSimulation(const rw::kinematics::State& initState);
	void pauseSimulation();
	void resumeSimulation();

	bool isRunning();
	bool isFinished();

	/**
	 * @brief get the statistics of the current simulations
	 * @param status [in]
	 * @return
	 */
	int getStat(TestStatus status){
	    if(status>=0 && status<=SimulationFailure)
	        return _stat[status];
	    RW_THROW("Unknown TestStatus!");
	}
	std::vector<int> getStat(){ return _stat; }
	int getNrTargetsDone();

	/**
	 * @brief add a delay each time the simulation has stepped.
	 * usefull for debuging or visualization.
	 * @param delay
	 */
	void setStepDelay(int delay){ _stepDelayMs=delay;};

	// events
	// simulation target performed event
	typedef enum {TaskFormat, CommaSeperatedFormat} ExportFormat;

	static void save(const std::string& filename, rwlibs::task::CartesianTask::Ptr tasks,  ExportFormat format=TaskFormat);
	static void save(std::ostream& ostr, rwlibs::task::CartesianTask::Ptr tasks, ExportFormat format=TaskFormat);

	struct GraspedObject {
        GraspedObject():object(NULL){}
        rwsim::dynamics::RigidBody* object;
        std::vector<rw::sensor::Contact3D> contacts;
        std::vector<rwsim::dynamics::Body*> bodies;
    };

    typedef enum{GRASPING, LIFTING, NEW_GRASP, APPROACH} StepState;
    struct SimState {
        SimState():_restingTime(0),
                _simTime(0),
                _graspTime(0),
                _approachedTime(0),
                _currentState(NEW_GRASP)
        {}

        rw::kinematics::State _state;
        rw::common::Timer _wallTimer;
        rwlibs::task::CartesianTask::Ptr _task;
        rwlibs::task::CartesianTarget::Ptr _target;
        StepState _currentState;

        rw::kinematics::State _postLiftObjState;

        std::vector< rwsim::sensor::BodyContactSensor::Ptr > _bsensors;

        double  _restingTime,
                _simTime,
                _graspTime,
                _approachedTime; // the simulation time when the approach has finished

        // the explicit values from _task
        rw::kinematics::Frame* _taskRefFrame;
        rw::math::Transform3D<> _taskOffset;
        rw::math::Transform3D<> _approach;
        rw::math::Transform3D<> _retract;
        rw::math::Q _openQ, _closeQ;

        // explicit values of _target
        rw::math::Transform3D<> _wTtcp_initTarget,
                                _wTmbase_initTarget, // approach to this config from _initTarget
                                _wTmbase_approachTarget, // approach to this config from _initTarget
                                _wTmbase_retractTarget; // retract to this config from _approachTarget


        bool _stopped;

    };
private:

    //std::vector<rw::sensor::Contact3D> getObjectContacts(const rw::kinematics::State& state);


    GraspedObject getObjectContacts(const rw::kinematics::State& state, SimState &sstate);
    std::vector<rw::sensor::Contact3D> getObjectContacts(const rw::kinematics::State& state,
                                                         rwsim::dynamics::RigidBody *object,
                                                         rwsim::sensor::BodyContactSensor::Ptr sensor,
                                                         std::vector<rwsim::dynamics::Body*>& bodies);


    rw::math::Q calcGraspQuality(const rw::kinematics::State& state, SimState &sstate);

	/**
	 * @brief the step callback function
	 * @param state
	 */
	void stepCB(rwsim::simulator::ThreadSimulator* sim, const rw::kinematics::State& state);

	bool getNextTarget(SimState & sstate);

private:
	rwsim::dynamics::DynamicWorkCell::Ptr _dwc;

	int _gripperDim;
	int _stepDelayMs;
	int _nrOfThreads;
	bool _requestSimulationStop;
	bool _initialized;

	std::vector<int> _stat;
	int _failed, _success, _slipped, _collision, _timeout, _simfailed, _skipped,
	    _nrOfExperiments, _lastSaveTaskIndex;
	int _totalNrOfExperiments;

	int _autoSaveInterval;

	// if any object exceeds this threshold the simulation is considered faulty
	double _maxObjectGripperDistanceThreshold;

	std::vector<rwsim::dynamics::RigidBody*> _objects;
	rwsim::dynamics::DynamicDevice *_dhand;
	rwsim::dynamics::RigidDevice *_rhand;
    rw::models::Device* _hand;
    rwsim::dynamics::KinematicBody *_hbase;
    rw::kinematics::MovableFrame *_mbase;

    rwlibs::control::JointController *_graspController;

    rw::kinematics::Frame *_tcp;
    rw::kinematics::State _homeState;

	std::map<rwsim::simulator::ThreadSimulator::Ptr, SimState> _simStates;
	std::vector<rwsim::simulator::ThreadSimulator::Ptr> _simulators;

	rwlibs::task::CartesianTask::Ptr _roottask, _currentTask;
	std::stack<rwlibs::task::CartesianTask::Ptr> _taskQueue;
	int _currentTargetIndex;
	rw::proximity::CollisionDetector::Ptr _collisionDetector;
};

}
}

#endif /* GRASPTASKSIMULATOR_HPP_ */
