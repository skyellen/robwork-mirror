#include "InverseKinematicsTestSuite.hpp"

#include <rw/iksolvers/ResolvedRateSolver.hpp>
#include <rw/iksolvers/SimpleSolver.hpp>
#include <rw/iksolvers/SimpleMultiSolver.hpp>
#include <rw/iksolvers/CCDSolver.hpp>
#include <rwlibs/algorithms/qpcontroller/IKQPSolver.hpp>
#include <rw/iksolvers/PieperSolver.hpp>
#include <rw/inversekinematics/IterativeMultiIK.hpp>

#include <rw/models/DeviceModel.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Math.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <rw/loaders/rwxml/XMLRWLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/common/TimerUtil.hpp>
#include <string>

using namespace boost::unit_test;

using namespace robwork;
using namespace rwlibs::algorithms;
using namespace rw::loaders;

typedef std::auto_ptr<IterativeIK> (* MakeIKSolver)(SerialDevice*, State&);
typedef std::auto_ptr<IterativeMultiIK> (* MakeMultiIKSolver)(TreeDevice*, State&);

void testIKSolver(
    const std::string& solverName,
    MakeIKSolver maker,
    double relativeDisplacement)
{
    // Load a serial device that has revolute joints only.
    std::auto_ptr<WorkCell> workcell = WorkCellLoader::load("testfiles/PA10/PA10.wu");
    DeviceModel* any_device = workcell->getDevices().at(0);
    SerialDevice* device = dynamic_cast<SerialDevice*>(any_device);
    BOOST_REQUIRE(device);

    // Take a configuration somewhere in the valid range.
    std::pair<Q, Q> pair = device->getBounds();
    const Q q_zero = 0.45 * (pair.first + pair.second);

    const int maxCnt = 10;

    // The maximum amounts with which each joint is displaced.
    const Q displacements =
        relativeDisplacement * (pair.second - pair.first);

    // Create a number of small modifications of this configurations.
    std::vector<Q> q_targets;
    for (int cnt = 0; cnt < maxCnt; cnt++) {

        Q q = q_zero;
        const int dof = (int)q_zero.size();
        for (int i = 0; i < dof; i++) {
            const double d = displacements(i);
            q(i) += Math::Ran(-d, d);
        }

        q_targets.push_back(q);
    }

    // Use these to create targets.
    std::vector<Transform3D<> > targets;
    for (int i = 0; i < maxCnt; i++) {
        State state = workcell->getDefaultState();
        device->setQ(q_targets.at(i), state);
        targets.push_back(device->baseTend(state));
    }

    State initial_state = workcell->getDefaultState();
    std::auto_ptr<IterativeIK> solver = maker(device, initial_state);

    // Check if IK can be solved for all of the targets for a starting
    // configuration of q_zero.

    device->setQ(q_zero, initial_state);
    unsigned int errcnt = 0;
    for (int i = 0; i < maxCnt; i++) {
        const bool ok = !solver->solve(targets.at(i), initial_state).empty();
        if (!ok) {
            std::cout << "Could not solve IK for solver " << solverName << "\n";
	    errcnt++;
	}

    }
    BOOST_CHECK(errcnt <= 2);
}

void testMultiIKSolver(
    const std::string& solverName,
    MakeMultiIKSolver maker,
    double relativeDisplacement)
{
    // Load a tree device that has revolute joints only.
    std::auto_ptr<WorkCell> workcell = WorkCellLoader::load(
        "testfiles/SchunkHand/SchunkHand.xml");

    DeviceModel* any_device = workcell->getDevices().at(0);
    TreeDevice* device = dynamic_cast<TreeDevice*>(any_device);
    BOOST_REQUIRE(device);
    std::cout << "Device loadet" << std::endl;
    // Take a configuration somewhere in the valid range.
    std::pair<Q, Q> pair = device->getBounds();
    const Q q_zero = 0.45 * (pair.first + pair.second);

    const int maxCnt = 10;

    // The maximum amounts with which each joint is displaced.
    const Q displacements =
        relativeDisplacement * (pair.second - pair.first);
    std::cout << "Calculate random configurations" << std::endl;
    
    // Create a number of small modifications of this configurations.
    std::vector<Q> q_targets;
    for (int cnt = 0; cnt < maxCnt; cnt++) {

        Q q = q_zero;
        const int dof = (int)q_zero.size();
        for (int i = 0; i < dof; i++) {
            const double d = displacements(i);
            q(i) += Math::Ran(-d, d);
        }

        q_targets.push_back(q);
    }
    std::cout << "Calculate random targets" << std::endl;
    // Use these to create targets.
    std::vector<std::vector<Transform3D<> > > targets;
    for (int i = 0; i < maxCnt; i++) {
        State state = workcell->getDefaultState();
        device->setQ(q_targets.at(i), state);
        std::vector<Transform3D<> > target;
        for(size_t j=0;j<device->getEnds().size();j++){
            target.push_back(device->baseTframe(device->getEnds()[j],state));
        }
        targets.push_back(target);
    }
    std::cout << "get intial state" << std::endl;
    State initial_state = workcell->getDefaultState();
    std::auto_ptr<IterativeMultiIK> solver = maker(device, initial_state);

    // Check if IK can be solved for all of the targets for a starting
    // configuration of q_zero.
    std::cout << "Solve IK" << std::endl;
    device->setQ(q_zero, initial_state);
    unsigned int errcnt = 0;
    for (int i = 0; i < maxCnt; i++) {
        const bool ok = !solver->solve(targets.at(i), initial_state).empty();
        if (!ok) {
            std::cout << "Could not solve IK for solver " << solverName << "\n";
            errcnt++;
        }

    }
    BOOST_CHECK(errcnt <= 2);
}

void testIKSolverPerform(
    const std::string& solverName,
    MakeIKSolver maker,
    int maxCnt)
{
    // Load a serial device that has revolute joints only.
    std::auto_ptr<WorkCell> workcell = WorkCellLoader::load("testfiles/PA10/PA10.wu");
    DeviceModel* any_device = workcell->getDevices().at(0);
    SerialDevice* device = dynamic_cast<SerialDevice*>(any_device);
    BOOST_REQUIRE(device);

    // Take a configuration somewhere in the valid range.
    std::pair<Q, Q> pair = device->getBounds();
    const Q q_zero = 0.45 * (pair.first + pair.second);

    //const int maxCnt = 1000;


    // Create a number of small modifications of this configurations.
    std::vector<Q> q_targets;
    for (int cnt = 0; cnt < maxCnt; cnt++) {
        const int dof = (int)q_zero.size();
        Q q(dof);
        for (int i = 0; i < dof; i++) {
            q(i) = Math::Ran(pair.first[i], pair.second[i]);
        }
        q_targets.push_back(q);
    }

    // Use these to create targets.
    std::vector<Transform3D<> > targets;
    for (int i = 0; i < maxCnt; i++) {
        State state = workcell->getDefaultState();
        device->setQ(q_targets.at(i), state);
        targets.push_back(device->baseTend(state));
    }

    State initial_state = workcell->getDefaultState();
    std::auto_ptr<IterativeIK> solver = maker(device, initial_state);

    // Check if IK can be solved for all of the targets for a starting
    // configuration of q_zero.
    long startTime = TimerUtil::CurrentTimeMs();
    device->setQ(q_zero, initial_state);
    unsigned int solveCnt=0;
    for (int i = 0; i < maxCnt; i++) {
        const bool ok = !solver->solve(targets.at(i), initial_state).empty();
        if( ok )
            solveCnt++;
    }
    long endTime = TimerUtil::CurrentTimeMs();
    double succesratio = ((double)solveCnt/(double)maxCnt)*100.0;
    std::cout << solverName << " had a succesratio of: " 
              << succesratio << "%" << std::endl; 
    std::cout << solverName << " took " << (endTime-startTime) 
              <<"ms to solve IK for " << maxCnt << " targets"<< std::endl; 
    
    //BOOST_CHECK(errcnt <= 2);
}

void testMultiIKSolverPerform(
    const std::string& solverName,
    MakeMultiIKSolver maker,
    int maxCnt)
{
    // Load a tree device that has revolute joints only.
    std::auto_ptr<WorkCell> workcell = 
        XMLRWLoader::LoadWorkcell("testfiles/SchunkHand/SchunkHand.xml");
    DeviceModel* any_device = workcell->getDevices().at(0);
    TreeDevice* device = dynamic_cast<TreeDevice*>(any_device);
    BOOST_REQUIRE(device);

    // Take a configuration somewhere in the valid range.
    std::pair<Q, Q> pair = device->getBounds();
    const Q q_zero = 0.45 * (pair.first + pair.second);

    //const int maxCnt = 1000;


    // Create a number of small modifications of this configurations.
    std::vector<Q> q_targets;
    for (int cnt = 0; cnt < maxCnt; cnt++) {
        const int dof = (int)q_zero.size();
        Q q(dof);
        for (int i = 0; i < dof; i++) {
            q(i) = Math::Ran(pair.first[i], pair.second[i]);
        }
        q_targets.push_back(q);
    }

    // Use these to create targets.
    std::vector<std::vector<Transform3D<> > > targets;
    for (int i = 0; i < maxCnt; i++) {
        State state = workcell->getDefaultState();
        device->setQ(q_targets.at(i), state);
        std::vector<Transform3D<> > target;
        for(size_t j=0;j<device->getEnds().size();j++){
            target.push_back(device->baseTframe(device->getEnds()[j],state));
        }
        targets.push_back(target);
    }
    
    State initial_state = workcell->getDefaultState();
    std::auto_ptr<IterativeMultiIK> solver = maker(device, initial_state);

    // Check if IK can be solved for all of the targets for a starting
    // configuration of q_zero.
    long startTime = TimerUtil::CurrentTimeMs();
    device->setQ(q_zero, initial_state);
    unsigned int solveCnt=0;
    for (int i = 0; i < maxCnt; i++) {
        const bool ok = !solver->solve(targets.at(i), initial_state).empty();
        if( ok )
            solveCnt++;
    }
    long endTime = TimerUtil::CurrentTimeMs();
    double succesratio = ((double)solveCnt/(double)maxCnt)*100.0;
    std::cout << solverName << " had a succesratio of: " 
              << succesratio << "%" << std::endl; 
    std::cout << solverName << " took " << (endTime-startTime) 
              <<"ms to solve IK for " << maxCnt << " targets"<< std::endl; 
    
    //BOOST_CHECK(errcnt <= 2);
}

std::auto_ptr<IterativeIK> makeCCD(SerialDevice* device, State& state)
{
    std::auto_ptr<IterativeIK> result(new CCDSolver(device));
    return result;
}

std::auto_ptr<IterativeIK> makeResolvedRateSolver(SerialDevice* device, State& state)
{
    std::auto_ptr<IterativeIK> result(new ResolvedRateSolver(device));
    return result;
}

std::auto_ptr<IterativeIK> makeSimpleSolver(SerialDevice* device, State& state)
{
    SimpleSolver *sol = new SimpleSolver(device); 
    std::auto_ptr<IterativeIK> result(sol);
    return result;
}

std::auto_ptr<IterativeIK> makeIKQPSolver(SerialDevice* device, State& state) {
    std::auto_ptr<IterativeIK> result(new IKQPSolver(device, state));
    return result;
}


std::auto_ptr<IterativeMultiIK> makeSimpleMultiSolver(TreeDevice* device, State& state)
{
    SimpleMultiSolver *sol = new SimpleMultiSolver(device, state); 
    //sol->setMaxLocalStep(0.4,5.0);
    std::auto_ptr<IterativeMultiIK> result(sol);
    return result;
}

void testIterativeInverseKinematics()
{
    // We seed the random number generator so that we get reproducible results.
    Math::Seed(0);

    // The IK solvers really aren't impressive right now. In particular the CCD
    // solver isn't very reliable. Some tuning of these is necessary, I think.
    // Also perhaps the testIKSolver() should just verify that a _reasonably_
    // large percentage of the IK calculations succeed.
    testIKSolver("CCD", makeCCD, 0.002);
    testIKSolver("IKQPSolver", makeIKQPSolver, 0.2);
    testIKSolver("ResolvedRateSolver", makeResolvedRateSolver, 0.2);
    testIKSolver("SimpleSolver", makeSimpleSolver, 0.2);
    testMultiIKSolver("SimpleMultiSolver",makeSimpleMultiSolver, 0.2);
    
    // some performance testing
    testIKSolverPerform("SimpleSolver", makeSimpleSolver, 200);
    testIKSolverPerform("ResolvedRateSolver", makeResolvedRateSolver, 200);
    testIKSolverPerform("IKQPSolver", makeIKQPSolver, 20);
    testMultiIKSolverPerform("SimpleMultiSolver",makeSimpleMultiSolver, 200);
    //testIKSolverPerform("CCD", makeCCD, 200);
}





int testClosedFormWithQ(const Q& q, std::vector<DHSet>& dhparams) { 
    //Transform from the three intersection axis to tool
    Transform3D<> T06(Transform3D<>::Identity());

    for (size_t i = 0; i<dhparams.size(); i++) {
        T06 = T06*Transform3D<>::CraigDH(dhparams[i]._alpha, dhparams[i]._a, dhparams[i]._d, q(i));
    }
    Transform3D<> T6tool(Vector3D<>(0.1,0.2,0.3), RPY<>(1,2,3));

    Transform3D<> baseTend = T06*T6tool;

    PieperSolver solver(dhparams, T6tool);
    std::vector<Q> solutions = solver.solve(baseTend);

    //    BOOST_CHECK(solutions.size() == 8);
    for (std::vector<Q>::iterator it = solutions.begin(); it != solutions.end(); ++it) {
	Q qres = *it;
	T06 = Transform3D<>::Identity();
	for (size_t i = 0; i<dhparams.size(); i++) {
	    T06 = T06*Transform3D<>::CraigDH(dhparams[i]._alpha, dhparams[i]._a, dhparams[i]._d, qres(i));
	}

	Transform3D<> T6tool(Vector3D<>(0.1,0.2,0.3), RPY<>(1,2,3));
	Transform3D<> baseTend2 = T06*T6tool;

	Transform3D<> diff = inverse(baseTend)*baseTend2;

	for (int i = 0; i<3; i++) {
	    for (int j = 0; j<4; j++) {
		if (i == j)
		    BOOST_CHECK(fabs(diff(i,j)-1)<1e-12);
		else
		    BOOST_CHECK(fabs(diff(i,j))<1e-12);
	    }
	}
    }
    return solutions.size();
}


void testClosedFormInverseKinematics() {
    std::cout<<"Testing PieperSolver"<<std::endl;
    Q q(boost::numeric::ublas::zero_vector<double>(6));

    const double DEG2RAD = M_PI/180.;
    std::vector<DHSet> dhparams;
    dhparams.push_back(DHSet(0,0,0,0));
    dhparams.push_back(DHSet(-90*DEG2RAD, 0.26, 0, 0));
    dhparams.push_back(DHSet(0,0.68,0,0));    
    dhparams.push_back(DHSet(-90*DEG2RAD,-0.035,0.67,0));
    dhparams.push_back(DHSet(-90*DEG2RAD,0,0,0));
    dhparams.push_back(DHSet(90*DEG2RAD,0,0,0));


    int cnt = testClosedFormWithQ(q, dhparams);
    BOOST_CHECK(cnt == 8);

    q(0) = 0.5;
    q(1) = 1;    
    q(2) = 1.5;
    q(3) = 2;
    q(4) = 2.5;
    q(5) = 3;
    cnt = testClosedFormWithQ(q, dhparams);
    BOOST_CHECK(cnt == 8);


    //Test special case with a1=0
    std::vector<DHSet> dhparams2;
    dhparams2.push_back(DHSet(0,0,0,0));
    dhparams2.push_back(DHSet(-90*DEG2RAD, 0, 0, 0));
    dhparams2.push_back(DHSet(0,0.68,0,0));    
    dhparams2.push_back(DHSet(-90*DEG2RAD,-0.035,0.67,0));
    dhparams2.push_back(DHSet(-90*DEG2RAD,0,0,0));
    dhparams2.push_back(DHSet(90*DEG2RAD,0,0,0));
    cnt = testClosedFormWithQ(q, dhparams2);
    BOOST_CHECK(cnt == 8);

    //Test special case with alpha1 = 0
    std::vector<DHSet> dhparams3;


    dhparams3.push_back(DHSet(0,0,0,0));
    dhparams3.push_back(DHSet(0, 0.26, 0, 0));
    dhparams3.push_back(DHSet(90*DEG2RAD,0.68,0,0));    
    dhparams3.push_back(DHSet(-90*DEG2RAD,-0.035,0.67,0));
    dhparams3.push_back(DHSet(-90*DEG2RAD,0,0,0));
    dhparams3.push_back(DHSet(90*DEG2RAD,0,0,0));

    q(0) = 1;
    q(1) = 1;
    q(2) = 1;
    q(3) = 1.4;
    q(4) = 1.5;
    q(5) = 1.6;

    cnt = testClosedFormWithQ(q, dhparams3);
    BOOST_CHECK(cnt == 4);
    std::cout<<"PieperSolver Tested"<<std::endl;
}


InverseKinematicsTestSuite::InverseKinematicsTestSuite()
{
    BOOST_MESSAGE("InverseKinematicsTestSuite");
    add(BOOST_TEST_CASE(&testIterativeInverseKinematics));
    add(BOOST_TEST_CASE(&testClosedFormInverseKinematics));
}                        

