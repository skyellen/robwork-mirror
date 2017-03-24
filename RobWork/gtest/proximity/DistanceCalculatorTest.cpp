/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <gtest/gtest.h>
#include "../TestEnvironment.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/DistanceCalculator.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <cmath>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::proximity;
using namespace std;

class DistanceCalculatorTest : public ::testing::Test {
public:

    CollisionSetup collisionSetup;
    const string workcellFile = TestEnvironment::testfilesDir() + "/workcells/DistanceTest.wc.xml";
    rw::models::WorkCell::Ptr wc;
    rw::kinematics::Frame* initialFrame;
    rw::kinematics::State initialState;

    DistanceStrategy::Ptr strategyPQP;
    DistanceCalculator::Ptr distCalcPQP;


    virtual void SetUp(){

        wc = WorkCellLoader::Factory::load(workcellFile);
        initialFrame = wc->findFrame("Sphere1");
        initialState = wc->getDefaultState();
        collisionSetup = wc->getCollisionSetup();


        strategyPQP = DistanceStrategy::Factory::makeStrategy("PQP");
        distCalcPQP = new DistanceCalculator(wc, strategyPQP);
    }

};



TEST_F (DistanceCalculatorTest, SimpleGeometryPQP)
{
    vector<DistanceStrategy::Result> * distances = new vector<DistanceStrategy::Result>;

    // distances vector should be empty:
    EXPECT_EQ(distances->size(),0);

    distCalcPQP->distance(initialState, distances);
    // DEBUG: Gives the frame pair names and distance
    /*for (int i = 0; i < distances->size(); i++)
    {
    cout << distances->at(i).f1->getName() << " " << distances->at(i).f2->getName() << " " << distances->at(i).distance << endl;
    }
    */


    // Checking a serial chain of 4 frames, expect 3 distances:
    EXPECT_EQ(distances->size(),3);

    //Radius of spheres: 0.2, size of boxes: x=y=z=0.2;

    //Distance from sphere1 to sphere2:
    //  distance = sqrt(1²+2²+3²)-2radius
    EXPECT_NEAR(distances->at(0).distance,2.6,0.001);

    //Distance from sphere1 to box2:
    //  distance = 2-radius-0.1
    EXPECT_NEAR(distances->at(1).distance,1.7,0.001);

    //Distance from box1 to box2:
    //  distance = 1-2*0.1
    EXPECT_NEAR(distances->at(2).distance,0.8,0.001);


}

