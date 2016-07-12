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
/*

#include <rw/trajectory/Path.hpp>
#include <rw/trajectory/TrajectoryFactory.cpp>
#include <rw/math/Q.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <sandbox/AsciiPathSaver.hpp>

#include <iostream>
#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::trajectory;
*/
BOOST_AUTO_TEST_CASE( PathTest ){
	/*
    BOOST_MESSAGE("- Testing Path");

    //Test the simple path
    {
        QPath path;
        Q q(7);
        q(0) = 1;
        path.push_back(q);
        q(0) = 2;
        path.push_back(q);
        q(0) = 3;
        path.push_back(q);
        int index = 1;
        BOOST_FOREACH(Q q, path) {
            BOOST_CHECK(q(0) == index);
            index++;
        }
		AsciiPathSaver::save("d:\\temp\\TestPath.save", path, AsciiPathSaver::Mathematica);
    }


    BOOST_MESSAGE("- Testing Path 2");

    //Test the simple path
    {
        Transform3DPath path;
        Transform3D<> t;
        t(0,0) = 1;
        path.push_back(t);
        t(0,0) = 2;
        path.push_back(t);
        t(0,0) = 3;
        path.push_back(t);
		AsciiPathSaver::save("d:\\temp\\TestPath2.save", path, AsciiPathSaver::Mathematica);
    }
*/
    /*
    StatePath statepath;
    WorkCellPtr workcell = WorkCellLoader::load(testFilePath() + "MultiRobotDemo/Scene.wu");
    BOOST_CHECK(workcell);
    Device* dev = workcell->getDevices().front();
    const State defstate = workcell->getDefaultState();

    //Test StatePath and the possibility of creating
    {
        StatePath statepath;
        State state = defstate;

        statepath.push_back(state);
        Q q = dev->getQ(state);
        for (size_t i = 0; i<q.size(); i++)
            q(i) = 0.5;
        dev->setQ(q, state);
        statepath.push_back(state);
        for (size_t i = 0; i<q.size(); i++)
            q(i) = -1;
        dev->setQ(q, state);
        statepath.push_back(state);

        StateTrajectoryPtr statetrajectory = TrajectoryFactory::makeLinearTrajectoryUnitStep(statepath);
        State s0 = statetrajectory->x(0);
        q = dev->getQ(s0);
        BOOST_CHECK(q(0) == 0);

        State s05 = statetrajectory->x(0.5);
        q = dev->getQ(s05);
        BOOST_CHECK(q(0) == 0.25);

        State s1 = statetrajectory->x(1);
        q = dev->getQ(s1);
        BOOST_CHECK(q(0) == 0.5);

        State s15 = statetrajectory->x(1.5);
        q = dev->getQ(s15);
        std::cout<<"q = "<<q<<std::endl;
        BOOST_CHECK(q(0) == -0.25);

        State s2 = statetrajectory->x(2);
        q = dev->getQ(s2);
        BOOST_CHECK(q(0) == -1);
    }

    //Test StatePath and the possibility of creating
    {
        TimedStatePath timedStatePath;
        WorkCellPtr workcell = WorkCellLoader::load(testFilePath() + "MultiRobotDemo/Scene.wu");
        BOOST_CHECK(workcell);

        Device* dev = workcell->getDevices().front();
        State state = workcell->getDefaultState();
        timedStatePath.push_back(Timed<State>(0, state));

        Q q = dev->getQ(state);
        for (size_t i = 0; i<q.size(); i++)
            q(i) = 0.5;
        dev->setQ(q, state);
        timedStatePath.push_back(Timed<State>(12, state));

        for (size_t i = 0; i<q.size(); i++)
            q(i) = -1;
        dev->setQ(q, state);
        timedStatePath.push_back(Timed<State>(24, state));

        StateTrajectoryPtr statetrajectory = TrajectoryFactory::makeLinearTrajectory(timedStatePath);
        State s0 = statetrajectory->x(0);
        q = dev->getQ(s0);
        std::cout<<"q(0) = "<<q<<std::endl;
        BOOST_CHECK(q(0) == 0);

        State s3 = statetrajectory->x(3);
        q = dev->getQ(s3);
        std::cout<<"q(3) = "<<q<<std::endl;
        BOOST_CHECK(q(0) == 0.125);

        State s1 = statetrajectory->x(12);
        q = dev->getQ(s1);
        BOOST_CHECK(q(0) == 0.5);

        State s15 = statetrajectory->x(18);
        q = dev->getQ(s15);
        std::cout<<"q(18) = "<<q<<std::endl;
        BOOST_CHECK(q(0) == -0.25);

        State s2 = statetrajectory->x(24);
        q = dev->getQ(s2);
        BOOST_CHECK(q(0) == -1);
    }
    */
}
