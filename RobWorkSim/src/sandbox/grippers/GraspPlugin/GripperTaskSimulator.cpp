#include "GripperTaskSimulator.hpp"

#include <rw/math/MetricFactory.hpp>
#include <rwlibs/algorithms/StablePose1DModel.hpp>
#include <rwlibs/algorithms/StablePose0DModel.hpp>
#include <algorithm>
#include <vector>
#include "TaskGenerator.hpp"

#define DEBUG cout


using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;
using namespace rwsim::simulator;
using namespace rw::common;
using namespace rwlibs::algorithms;



GripperTaskSimulator::GripperTaskSimulator(rw::models::Gripper::Ptr gripper, rwlibs::task::GraspTask::Ptr tasks,
		rwlibs::task::GraspTask::Ptr samples, TaskDescription::Ptr td) :
	GraspTaskSimulator(td->getDynamicWorkCell(), 1),
	_gripper(gripper),
	_samples(samples),
	_td(td)
{
	setWallTimeLimit(10.0);
	setSimTimeLimit(0.0);
	load(tasks);
}



void GripperTaskSimulator::graspFinished(SimState& sstate)
{
	/* After each grasp is finished, its quality should be asessed.
	 * First, grasp has to pass interference measurement. When the sum
	 * of individual object interferences exceed limit specified in task description,
	 * grasp status is changed from Success to Interference.
	 * Then, if wrench space measurement result is below specified minimum wrench,
	 * grasp status is changed from Success to ObjectDropped.
	 */
	if (!_td) {
		RW_THROW("NULL task description!");
	}
	//RW_WARN("");
	if ((sstate._target->getResult()->testStatus == GraspTask::Success ||
		sstate._target->getResult()->testStatus == GraspTask::ObjectSlipped) &&
		calculateInterference(sstate, _td->getInitState()) > _td->getInterferenceLimit()) {
			
		DEBUG << "Grasp above interference limit." << endl;
		sstate._target->getResult()->testStatus = GraspTask::Interference;
	}
	//RW_WARN("");
	if ((sstate._target->getResult()->testStatus == GraspTask::Success ||
		sstate._target->getResult()->testStatus == GraspTask::ObjectSlipped) &&
		calculateWrench(sstate) < _td->getWrenchLimit()) {
			
		DEBUG << "Grasp below wrench limit." << endl;
		sstate._target->getResult()->testStatus = GraspTask::ObjectDropped;
	}
	//RW_WARN("");
	printGraspResult(sstate);
	//RW_WARN("");
}



double GripperTaskSimulator::calculateInterference(SimState& sstate, const rw::kinematics::State& state0)
{
	State state1 = getSimulators()[0]->getState();
	
	double interference = 0.0;
	vector<double> interferenceDistances;
	vector<double> interferenceAngles;
	vector<double> interferences;
	
	BOOST_FOREACH (Object::Ptr obj, _td->getInterferenceObjects()) {
		
		Transform3D<> Tbefore = obj->getBase()->getTransform(state0);
		Transform3D<> Tafter = obj->getBase()->getTransform(state1);
		
		double d = MetricUtil::dist2(Tafter.P(), Tbefore.P());
		interferenceDistances.push_back(d);
		
		double a = angle(Tbefore*Vector3D<>::z(), Tafter*Vector3D<>::z());
		interferenceAngles.push_back(a);
		
		Metric<Transform3D<> >::Ptr metric = MetricFactory::makeTransform3DMetric<double>(1.0, 1.0);
		double objInt = metric->distance(Tbefore, Tafter);
		
		interference += objInt;
		interferences.push_back(objInt);
	}
	
	sstate._target->getResult()->interferenceDistances = interferenceDistances;
	sstate._target->getResult()->interferenceAngles = interferenceAngles;
	sstate._target->getResult()->interferences = interferences;
	sstate._target->getResult()->interference = interference;
	
	return interference;
}



double GripperTaskSimulator::calculateWrench(SimState& sstate) const
{
	Q& qual = sstate._target->getResult()->qualityAfterLifting;
	//qual = _gripper->getForce() * qual;
	
	if (qual.size() >= 1) {
		return qual(1);
	} else {
		//DEBUG << "No wrench measurement!" << endl;
		return 0.0;
	} //sstate._target->getResult()->qualityAfterLifting(1); // use average wrench from origin
}



double GripperTaskSimulator::calculateCoverage(double actualRatio)
{
	if (!_gtask || !_samples) {
		RW_WARN("NULL tasks or samples");
		return 0.0;
	}
	
	//DEBUG << "CALCULATING COVERAGE - " << endl;
	
	double coverage = 0.0;

	Q covDist = _td->getCoverageDistance();
	double R = 2.0 * sin(0.25 * covDist(1));
	Q diff(7, covDist(0), covDist(0), covDist(0), R, R, R, covDist(2));
	
	/* okTargets is the number of succesful targets after filtering +
	 * the number of slippages + the number of interferences */
	//DEBUG << "Filtering targets..." << endl;
	GraspTask::Ptr coverageTasks = TaskGenerator::filterTasks(_gtask, diff);
	int okTargets = TaskGenerator::countTasks(coverageTasks, GraspTask::Success); 	
	//DEBUG << "Successful tasks: " << okTargets;
	okTargets += TaskGenerator::countTasks(coverageTasks, GraspTask::ObjectSlipped);
	okTargets += TaskGenerator::countTasks(coverageTasks, GraspTask::Interference);
	//DEBUG << " + interference= " << okTargets << endl;
	
	//DEBUG << "Filtering samples..." << endl;
	int allTargets = TaskGenerator::countTasks(TaskGenerator::filterTasks(_samples, diff), GraspTask::UnInitialized);
	
	DEBUG << "Requested targets: " << getNrTargets() << " / Samples: " << _samples->getAllTargets().size() << endl;
	DEBUG << "Targets (S+I) after filtering: " << okTargets << " / Samples after filtering: " << allTargets << endl;
	coverage = 1.0 * okTargets / (allTargets * actualRatio);
	
	return coverage;
}



bool sortf(double a, double b) { return (a>b); }



rw::math::Q GripperTaskSimulator::calculateWrenchMeasurement() const
{
	/* this only takes into account succesful grasps */
	vector<double> wrenches; // used to find the top 10%
	Q wrench(3, 0, 0, 0);
	
	int successes = 0;
	typedef std::pair<class GraspSubTask*, class GraspTarget*> TaskTarget;
	//DEBUG << "WRENCHES!" << endl;
	BOOST_FOREACH (TaskTarget p, _gtask->getAllTargets()) {
		//DEBUG << "??? " << p.second->getResult()->testStatus << endl;
		if (p.second->getResult()->testStatus == GraspTask::Success || p.second->getResult()->testStatus == GraspTask::ObjectSlipped) {
			successes++;
			
			Q result = p.second->getResult()->qualityAfterLifting;
			//DEBUG << result << endl;
			
			wrench(0) += result(0);
			wrench(1) += result(1);
			
			wrenches.push_back(result(1));
		}
	}
	
	// find top 20%
	sort(wrenches.begin(), wrenches.end(), sortf);
	
	int num = 0.2*successes < 1 ? 1 : 0.2*successes;
	
	if (wrenches.size() > 0) {
		for (int i = 0; i < num; ++i) {
			wrench(2) += wrenches[i];
			
			//cout << wrench(2) << endl;
		}
	}
	
	// calculate averages
	if (successes == 0) successes = 1;
	wrench(0) /= successes;
	wrench(1) /= successes;
	wrench(2) /= num;
	
	return wrench;
}



double GripperTaskSimulator::calculateAlignment() const {
	double alignment = 0.0;
	
	// store rotations
	int successes = 0;
	vector<Rotation3D<> > rot_before, rot_after;
	typedef std::pair<class GraspSubTask*, class GraspTarget*> TaskTarget;
	BOOST_FOREACH (TaskTarget p, _gtask->getAllTargets()) {
		
		// we only take succesful grasps
		if (p.second->getResult()->testStatus == GraspTask::Success) {
			rw::math::Transform3D<> poseApproach = inverse(p.second->getResult()->objectTtcpApproach);
			rw::math::Transform3D<> poseLift = inverse(p.second->getResult()->objectTtcpLift);
		
			rot_before.push_back(poseApproach.R());
			rot_after.push_back(poseLift.R());
			
			++successes;
		}
	}
	
	// use RANSAC to find the most likely stable pose
	Q params = _td->getRANSACParameters();
	vector<StablePose0DModel> models = StablePose0DModel::findModels(rot_after, (int)params[0], (int)params[1], params[2], params[3]);
	
	if (models.size() == 0) return 0.0;
	sort(models.begin(), models.end());
	reverse(models.begin(), models.end());
	
	int inliers = 0;
	Rotation3DAngleMetric<double> metric;
	DEBUG << "Models found (" << models.size() << "):" << endl;
	BOOST_FOREACH (const StablePose0DModel& m, models) {
		DEBUG << " - StablePose: " << m << ", QUALITY: " << m.getQuality() << ", INLIERS: " << m.getNumberOfInliers() << endl;
		inliers += m.getNumberOfInliers();
		
		// calculate model mean and variance
		int n = m.getNumberOfInliers();		
		vector<size_t> indices = m.getInlierIndices();
		if (indices.size() == 0) continue;
		
		// mean
		vector<double> diffs;
		double avg_diff = 0.0;
		BOOST_FOREACH (size_t idx, indices) {
			double diff = metric.distance(rot_before[idx], rot_after[idx]);
			diffs.push_back(diff);
			avg_diff += diff;
		}
		avg_diff /= n;
		DEBUG << "Average difference= " << avg_diff << endl;
		
		// variance
		double variance = 0.0;
		BOOST_FOREACH (double diff, diffs) {
			double dvar = diff - avg_diff;
			//DEBUG << "Dvar= " << dvar << endl;
			variance += dvar * dvar;
		}
		variance = sqrt(variance / n);
		DEBUG << "Variance= " << variance << endl;
		
		alignment += variance * n / successes;
		DEBUG << "Alignment so far= " << alignment << endl;
	}
	
	DEBUG << "Sum of inliers= " << inliers << endl;
	DEBUG << "Total successes= " << successes << endl;

	// this uses the number of inliers in all of the stable poses:
	//alignment = (successes > 0) ? 1.0 * inliers / successes : 0.0; //sqrt(alignment / diffs.size());
	
	DEBUG << "Alignment= " << alignment << endl;
	
	return 1.0 * alignment; // scaling factor
}



double GripperTaskSimulator::calculateShape()
{
	return 0.0;
}



double GripperTaskSimulator::calculateQuality()
{
	return 0.0;
}



void GripperTaskSimulator::printGraspResult(SimState& sstate)
{
	//int status = sstate._target->getResult()->testStatus;
	
	/*switch (status) {
		case GraspTask::Success:
			DEBUG << "Grasp result " << getNrTargetsDone() << ": success" << endl;
			break;
			
		case GraspTask::Interference:
			DEBUG << "Grasp result " << getNrTargetsDone() << ": INTERFERENCE" << endl;
			break;
			
		case GraspTask::ObjectSlipped:
			DEBUG << "Grasp result " << getNrTargetsDone() << ": SLIPPED or INSUFFICIENT WRENCH" << endl;
			break;
			
		case GraspTask::TimeOut:
			DEBUG << "Grasp result " << getNrTargetsDone() << ": TIMEOUT" << endl;
			break;
			
		default:
			DEBUG << "Grasp result " << getNrTargetsDone() << ": OTHER(" << status << ")" << endl;
	}*/
	
	DEBUG << "Grasp result " << getNrTargetsDone() << ": "
		<< GraspTask::toString((GraspTask::TestStatus)sstate._target->getResult()->testStatus)
		<< endl;
	
	DEBUG << " I: " << sstate._target->getResult()->interference;
	DEBUG << " W: " << sstate._target->getResult()->qualityAfterLifting << endl;
	
	if ((GraspTask::TestStatus)sstate._target->getResult()->testStatus == GraspTask::Success) {
		rw::math::Transform3D<> poseApproach = inverse(sstate._target->getResult()->objectTtcpApproach);
		DEBUG << " Pose after approach: " << poseApproach.P() << " " << RPY<>(poseApproach.R()) << endl;
		
		rw::math::Transform3D<> poseLift = inverse(sstate._target->getResult()->objectTtcpLift);
		DEBUG << " Pose after lift: " << poseLift.P() << " " << RPY<>(poseLift.R()) << endl;
	}
}



void GripperTaskSimulator::simulationFinished(SimState& sstate)
{
	evaluateGripper();
}



void GripperTaskSimulator::evaluateGripper()
{
	/* This function performs gripper quality evaluation.
	 * This includes:
	 * - success ratio calculation
	 * - wrench space measurement
	 * - coverage calculation
	 * 
	 * AT THIS POINT I'M NOT EVEN SURE WHY & HOW THIS WORKS:
	 * 
	 */
	TaskDescription::Qualities& b = _td->getBaseline();
	TaskDescription::Qualities& w = _td->getWeights();
	
	DEBUG << "EVALUATION - " << endl;
	DEBUG << _gripper->getName() << " - Evaluating..." << endl;
	
	int successes = TaskGenerator::countTasks(_gtask, GraspTask::Success);
	int interferences = TaskGenerator::countTasks(_gtask, GraspTask::Interference);
	int slippages = TaskGenerator::countTasks(_gtask, GraspTask::ObjectSlipped);
	int drops = TaskGenerator::countTasks(_gtask, GraspTask::ObjectDropped);
	int failures = TaskGenerator::countTasks(_gtask, GraspTask::SimulationFailure);
	int samples = TaskGenerator::countTasks(_samples, GraspTask::UnInitialized);
	int removed = TaskGenerator::countTasks(_gtask, GraspTask::Filtered);
	int filtered = getNrTargets() - removed;
	int actual = filtered - failures;
	
	successes += slippages;
	
	DEBUG << "CALCULATING SUCCESS RATIO - " << endl;
	DEBUG << " * Targets generated: " << getNrTargets() << endl;
	DEBUG << " * After preliminary filtering: " << filtered << endl;
	DEBUG << " * Actual simulated targets (without sim failure): " << actual << endl;
	
	//DEBUG << "* Actual number of tasks simulated is " << actual << " out of " << getNrTargets()
	//	<< " targets." << endl;
	DEBUG << "* Outcomes (success/interference/drop/fail): " << successes << "/" << interferences
		<< "/" << drops << "/" << failures << endl;
		
	/*typedef std::pair<class GraspSubTask*, class GraspTarget*> TaskTarget;
	BOOST_FOREACH (TaskTarget p, _gtask->getAllTargets()) {
		DEBUG << GraspTask::toString((GraspTask::TestStatus)p.second->getResult()->testStatus) << endl;
	}*/
	
	double successRatio = (1.0 * successes / actual) / b.success;
	
	/* alignment */
	DEBUG << "CALCULATING ALIGNMENT - " << endl;
	double alignment = calculateAlignment();
	DEBUG << " * Alignment: " << alignment << endl;
	
	/* wrench */
	DEBUG << "CALCULATING WRENCH - " << endl;
	Q wrenchMeasurement = calculateWrenchMeasurement();
	double wrench = wrenchMeasurement(1) / b.wrench;
	double topwrench = wrenchMeasurement(2) / b.wrench;
	
	/* coverage */
	DEBUG << "CALCULATING COVERAGE - " << endl;
	double coverage = calculateCoverage(1.0 * actual / filtered) / b.coverage;
	// task set is filtered at this point
	
	//double sumWeights = w.coverage + w.success + w.wrench + w.topwrench + w.robustness + w.stress;
	
	/* Calculate quality
	 * 
	 * quality is based on success, but with subtracted penalty for max stress:
	 * penalty = stress / stresslimit clamped to [0, 1]
	 * quality = success - penalty clamped to [0, 1]
	 */
	DEBUG << "CALCULATING STRESS - " << endl;
	double maxstress = _gripper->getMaxStress();
	double penaltyS = maxstress / _td->getStressLimit();
	//if (penaltyS > 1.0) penaltyS = 1.0;
	
	DEBUG << " * Max stress: " << maxstress << endl;
	DEBUG << " * Penalty (stress): " << penaltyS << endl;
	
	DEBUG << "CALCULATING VOLUME - " << endl;
	double volume = _gripper->getVolume();
	double penaltyV = volume;
	//if (penaltyV > 1.0) penaltyV = 1.0;
	
	DEBUG << " * Volume: " << volume << endl;
	DEBUG << " * Penalty (volume): " << penaltyV << endl;
	
	DEBUG << "CALCULATING PENALTY - " << endl;
	double penalty = w.stress * penaltyS + w.volume * penaltyV;
	//if (penalty > 1.0) {
	//	penalty = 1.0;
	//}
	DEBUG << " * Penalty (volume+stress): " << penalty << endl;
	
	DEBUG << "CALCULATING QUALITY - " << endl;
	DEBUG << " * Weights: " << endl;
	DEBUG << "  - Wsuccess= " << w.success << endl;
	DEBUG << "  - Wcoverage= " << w.coverage << endl;
	DEBUG << "  - Wwrench= " << w.wrench << endl;
	DEBUG << "  - Wtopwrench= " << w.topwrench << endl;
	DEBUG << "  - Wstress= " << w.stress << endl;
	DEBUG << "  - Wvolume= " << w.volume << endl;
	DEBUG << "  - Walignment= " << w.alignment << endl;
	
	double quality = w.success * successRatio + w.coverage * coverage + w.wrench * wrench + w.topwrench * topwrench + w.alignment * alignment - penalty;
	if (quality < 0.0) quality = 0.0;
	
	// save data to gripper result
	
	_quality.nOfExperiments = getNrTargets();
	_quality.nOfSuccesses = successes;
	_quality.nOfSamples = samples;
	_quality.coverage = coverage;
	_quality.success = successRatio;
	_quality.wrench = wrench;
	_quality.topwrench = topwrench;
	_quality.maxstress = maxstress;
	_quality.volume = volume;
	_quality.alignment = alignment;
	_quality.quality = quality;
	
	DEBUG << "DONE - " << endl;
	DEBUG << _quality << endl;
}
