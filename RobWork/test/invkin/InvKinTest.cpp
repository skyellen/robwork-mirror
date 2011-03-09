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

#include <rw/rw.hpp>

#include <string>
#include <boost/test/unit_test.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace boost::unit_test;
//using namespace rwlibs::algorithms;
using namespace rw::loaders;

typedef std::auto_ptr<IterativeIK> (* MakeIKSolver)(SerialDevice*, State&);
typedef std::auto_ptr<IterativeMultiIK> (* MakeMultiIKSolver)(TreeDevice*, State&);

void testIKSolver(
    const std::string& solverName,
    MakeIKSolver maker,
    double relativeDisplacement)
{
    BOOST_MESSAGE("- Testing " << solverName);
    // Load a serial device that has revolute joints only.
    WorkCell::Ptr workcell = WorkCellLoader::load(testFilePath() + "PA10/pa10.xml");
    Device* any_device = workcell->getDevices().at(0).get();
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
            q(i) += Math::ran(-d, d);
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
    BOOST_MESSAGE("- Testing " << solverName);
    // Load a tree device that has revolute joints only.
    WorkCell::Ptr workcell = WorkCellLoader::load(
        testFilePath() + "SchunkHand/SchunkHand.xml");

    Device* any_device = workcell->getDevices().at(0).get();

    TreeDevice* device = dynamic_cast<TreeDevice*>(any_device);
    BOOST_REQUIRE(device);
    //std::cout << "Device loadet" << std::endl;
    // Take a configuration somewhere in the valid range.
    std::pair<Q, Q> pair = device->getBounds();
    const Q q_zero = 0.45 * (pair.first + pair.second);

    const int maxCnt = 10;

    // The maximum amounts with which each joint is displaced.
    const Q displacements =
        relativeDisplacement * (pair.second - pair.first);
    //std::cout << "Calculate random configurations" << std::endl;

    // Create a number of small modifications of this configurations.
    std::vector<Q> q_targets;
    for (int cnt = 0; cnt < maxCnt; cnt++) {

        Q q = q_zero;
        const int dof = (int)q_zero.size();
        for (int i = 0; i < dof; i++) {
            const double d = displacements(i);
            q(i) += Math::ran(-d, d);
        }

        q_targets.push_back(q);
    }
    //std::cout << "Calculate random targets" << std::endl;
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
    //std::cout << "get intial state\n";
    State initial_state = workcell->getDefaultState();
    std::auto_ptr<IterativeMultiIK> solver = maker(device, initial_state);

    // Check if IK can be solved for all of the targets for a starting
    // configuration of q_zero.
    //std::cout << "Solve IK: " << solverName << "\n";
    device->setQ(q_zero, initial_state);
    unsigned int errcnt = 0;
    for (int i = 0; i < maxCnt; i++) {
        const bool ok = !solver->solve(targets.at(i), initial_state).empty();
        if (!ok) {
            //std::cout << "Could not solve IK for solver " << solverName << "\n";
            errcnt++;
        }
    }

    BOOST_CHECK(errcnt <= 2);
}


std::auto_ptr<IterativeIK> makeCCD(SerialDevice* device, State& state)
{
    std::auto_ptr<IterativeIK> result(new CCDSolver(device,state));
    return result;
}

std::auto_ptr<IterativeIK> makeSimpleSolver(SerialDevice* device, State& state)
{
    JacobianIKSolver *sol = new JacobianIKSolver(device,state);
    std::auto_ptr<IterativeIK> result(sol);
    return result;
}
/*
std::auto_ptr<IterativeIK> makeIKQPSolver(SerialDevice* device, State& state) {
  std::auto_ptr<IterativeIK> result(new IKQPSolver(device, state));
  return result;
}
*/

std::auto_ptr<IterativeMultiIK> makeSimpleMultiSolver(TreeDevice* device, State& state)
{
    JacobianIKSolverM *sol = new JacobianIKSolverM(device, state);
    //sol->setMaxLocalStep(0.4,5.0);
    std::auto_ptr<IterativeMultiIK> result(sol);
    return result;
}

BOOST_AUTO_TEST_CASE( testIterativeInverseKinematics )
{
    BOOST_MESSAGE("InverseKinematicsTestSuite");
    // We seed the random number generator so that we get reproducible results.
    Math::seed(0);

    // The IK solvers really aren't impressive right now. In particular the CCD
    // solver isn't very reliable. Some tuning of these is necessary, I think.
    // Also perhaps the testIKSolver() should just verify that a _reasonably_
    // large percentage of the IK calculations succeed.

    // Too slow to be considered correct.
    testIKSolver("CCD", makeCCD, 0.002);

    //testIKSolver("IKQPSolver", makeIKQPSolver, 0.2);
    testIKSolver("JacobianIKSolver", makeSimpleSolver, 0.2);
    testMultiIKSolver("JacobianIKSolver",makeSimpleMultiSolver, 0.2);
}

int testClosedFormWithQ(const Q& q, std::vector<DHParameterSet>& dhparams) {
    //Transform from the three intersection axis to tool
    Transform3D<> T06(Transform3D<>::identity());

    for (size_t i = 0; i<dhparams.size(); i++) {
        T06 = T06*Transform3D<>::craigDH(
            dhparams[i].alpha(),
            dhparams[i].a(),
            dhparams[i].d(),
            q(i));
    }
    Transform3D<> T6tool(Vector3D<>(0.1,0.2,0.3), RPY<>(1,2,3));

    Transform3D<> baseTend = T06*T6tool;

    PieperSolver solver(dhparams, T6tool);
    State state;
    std::vector<Q> solutions = solver.solve(baseTend, state);

    //    BOOST_CHECK(solutions.size() == 8);
    for (std::vector<Q>::iterator it = solutions.begin(); it != solutions.end(); ++it) {
        Q qres = *it;
        T06 = Transform3D<>::identity();
        for (size_t i = 0; i<dhparams.size(); i++) {
            T06 = T06*Transform3D<>::craigDH(
                dhparams[i].alpha(),
                dhparams[i].a(),
                dhparams[i].d(),
                qres(i));
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


BOOST_AUTO_TEST_CASE( testClosedFormInverseKinematics ) {
    //std::cout<<"- Testing PieperSolver"<<std::endl;
    Q q(boost::numeric::ublas::zero_vector<double>(6));

    std::vector<DHParameterSet> dhparams;
    dhparams.push_back(DHParameterSet(0,0,0,0));
    dhparams.push_back(DHParameterSet(-90*Deg2Rad, 0.26, 0, 0));
    dhparams.push_back(DHParameterSet(0,0.68,0,0));
    dhparams.push_back(DHParameterSet(-90*Deg2Rad,-0.035,0.67,0));
    dhparams.push_back(DHParameterSet(-90*Deg2Rad,0,0,0));
    dhparams.push_back(DHParameterSet(90*Deg2Rad,0,0,0));


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
    std::vector<DHParameterSet> dhparams2;
    dhparams2.push_back(DHParameterSet(0,0,0,0));
    dhparams2.push_back(DHParameterSet(-90*Deg2Rad, 0, 0, 0));
    dhparams2.push_back(DHParameterSet(0,0.68,0,0));
    dhparams2.push_back(DHParameterSet(-90*Deg2Rad,-0.035,0.67,0));
    dhparams2.push_back(DHParameterSet(-90*Deg2Rad,0,0,0));
    dhparams2.push_back(DHParameterSet(90*Deg2Rad,0,0,0));
    cnt = testClosedFormWithQ(q, dhparams2);
    BOOST_CHECK(cnt == 8);

    //Test special case with alpha1 = 0
    std::vector<DHParameterSet> dhparams3;


    dhparams3.push_back(DHParameterSet(0,0,0,0));
    dhparams3.push_back(DHParameterSet(0, 0.26, 0, 0));
    dhparams3.push_back(DHParameterSet(90*Deg2Rad,0.68,0,0));
    dhparams3.push_back(DHParameterSet(-90*Deg2Rad,-0.035,0.67,0));
    dhparams3.push_back(DHParameterSet(-90*Deg2Rad,0,0,0));
    dhparams3.push_back(DHParameterSet(90*Deg2Rad,0,0,0));

    q(0) = 1;
    q(1) = 1;
    q(2) = 1;
    q(3) = 1.4;
    q(4) = 1.5;
    q(5) = 1.6;

    cnt = testClosedFormWithQ(q, dhparams3);
    BOOST_CHECK(cnt == 4);
    // std::cout<<"PieperSolver Tested"<<std::endl;
}

