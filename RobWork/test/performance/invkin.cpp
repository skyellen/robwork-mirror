/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "../TestSuiteConfig.hpp"

#include <rw/invkin.hpp>
#include <rw/models.hpp>
#include <rw/kinematics.hpp>
#include <rw/math.hpp>

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rwlibs/algorithms/qpcontroller/IKQPSolver.hpp>

#include <string>

#include <boost/test/unit_test.hpp>

USE_ROBWORK_NAMESPACE

using namespace rwlibs::algorithms;
using namespace boost::unit_test;
using namespace robwork;

typedef std::auto_ptr<IterativeIK> (* MakeIKSolver)(SerialDevice*, State&);
typedef std::auto_ptr<IterativeMultiIK> (* MakeMultiIKSolver)(TreeDevice*, State&);

void testIKSolverPerform(
    const std::string& solverName,
    MakeIKSolver maker,
    int maxCnt)
{
    BOOST_MESSAGE(" Performance testing " << solverName << " on 7 DOF robot arm (PA10)");
    // Load a serial device that has revolute joints only.
    WorkCellPtr workcell = WorkCellLoader::load(testFilePath() + "PA10/pa10.xml");
    Device* any_device = workcell->getDevices().at(0);
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
            q(i) = Math::ran(pair.first[i], pair.second[i]);
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
    long startTime = TimerUtil::currentTimeMs();
    device->setQ(q_zero, initial_state);
    unsigned int solveCnt=0;
    for (int i = 0; i < maxCnt; i++) {
        const bool ok = !solver->solve(targets.at(i), initial_state).empty();

        if( ok ){
            // TODO: check if it is infact the correct target...
            solveCnt++;
        }
    }
    long endTime = TimerUtil::currentTimeMs();
    double succesratio = (((double)solveCnt)/((double)maxCnt))*100.0;
    BOOST_MESSAGE("  *********************************************");
    BOOST_MESSAGE("  * " << solverName << " had a succesratio of: "
                  << succesratio << "%");
    BOOST_MESSAGE("  * " << solverName << " took " << (endTime-startTime)
                  <<"ms to solve IK for " << maxCnt << " targets");
    BOOST_MESSAGE("  *********************************************");
    //BOOST_CHECK(errcnt <= 2);
}

void testMultiIKSolverPerform(
    const std::string& solverName,
    MakeMultiIKSolver maker,
    int maxCnt)
{
    BOOST_MESSAGE("- Testing " << solverName);
    // Load a tree device that has revolute joints only.
    WorkCellPtr workcell = WorkCellLoader::load(testFilePath() + "SchunkHand/SchunkHand.xml");
    Device* any_device = workcell->getDevices().at(0);
    TreeDevice* device = dynamic_cast<TreeDevice*>(any_device);
    BOOST_REQUIRE(device);

    // Take a configuration somewhere in the valid range.
    std::pair<Q, Q> pair = device->getBounds();
    const Q q_zero = 0.45 * (pair.first + pair.second);

    // Create a number of small modifications of this configurations.
    std::vector<Q> q_targets;
    for (int cnt = 0; cnt < maxCnt; cnt++) {
        const int dof = (int)q_zero.size();
        Q q(dof);
        for (int i = 0; i < dof; i++) {
            q(i) = Math::ran(pair.first[i], pair.second[i]);
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
    long startTime = TimerUtil::currentTimeMs();
    device->setQ(q_zero, initial_state);
    unsigned int solveCnt=0;
    for (int i = 0; i < maxCnt; i++) {
        const bool ok = !solver->solve(targets.at(i), initial_state).empty();
        if( ok )
            solveCnt++;
    }
    long endTime = TimerUtil::currentTimeMs();
    double succesratio = ((double)solveCnt/(double)maxCnt)*100.0;
    BOOST_MESSAGE("  *********************************************");
    BOOST_MESSAGE("  * " << solverName << " had a succesratio of: "
                  << succesratio << "%");
    BOOST_MESSAGE("  * " << solverName << " took " << (endTime-startTime)
                  <<"ms to solve IK for " << maxCnt << " targets");
    BOOST_MESSAGE("  *********************************************");
    //BOOST_CHECK(errcnt <= 2);
}

std::auto_ptr<IterativeIK> makeCCD(SerialDevice* device, State& state)
{
    std::auto_ptr<IterativeIK> result(new CCDSolver(device,state));
    return result;
}

std::auto_ptr<IterativeIK> makeResolvedRateSolver(SerialDevice* device, State& state)
{
    std::auto_ptr<IterativeIK> result(new ResolvedRateSolver(device, state));
    return result;
}

std::auto_ptr<IterativeIK> makeSimpleSolver(SerialDevice* device, State& state)
{
    SimpleSolver *sol = new SimpleSolver(device,state);
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

BOOST_AUTO_TEST_CASE( testIterativeInverseKinematicsPerformance )
{
    BOOST_MESSAGE("InverseKinematicsPerformanceTestSuite");
    // We seed the random number generator so that we get reproducible results.
    Math::seed(0);

    // some performance testing
    testIKSolverPerform("SimpleSolver", makeSimpleSolver, 1000);
    testIKSolverPerform("ResolvedRateSolver", makeResolvedRateSolver, 1000);

    testIKSolverPerform("CCD", makeCCD, 100);

    testIKSolverPerform("IKQPSolver", makeIKQPSolver, 100);
    testMultiIKSolverPerform("SimpleMultiSolver",makeSimpleMultiSolver, 1000);

}
