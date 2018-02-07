/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/dom/DOMProximitySetupSaver.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/ProximitySetup.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <string>

using namespace rw::common;
using namespace rw::loaders;
using namespace rw::proximity;
using namespace std;

class DOMProximitySetupSaverTest : public ::testing::Test {
public:

    const string workcellFile = TestEnvironment::testfilesDir() + "/workcells/simple_wc/SimpleWorkcell.wc.xml";
    const string workcellFileUsingSerializedProx = TestEnvironment::testfilesDir() + "/workcells/simple_wc/SimpleWorkcellUsingSerializedProx.wc.xml";
    rw::models::WorkCell::Ptr wc;
    rw::models::WorkCell::Ptr wcUsingProx;
    rw::proximity::ProximitySetup proxSetup;
    rw::proximity::ProximitySetup serializedProxSetup;
    BasicFilterStrategy::Ptr broadphase;

    virtual void SetUp(){
        // load simple workcell for testing
        wc = WorkCellLoader::Factory::load(workcellFile);
        // obtain current proximity setup from workcell
        proxSetup = rw::proximity::ProximitySetup::get(*wc);
    }
};

TEST_F (DOMProximitySetupSaverTest, LoadWCSerializeProxAndReload)
{
    /*
     *  At this stage the test workcell and the associated proximity setup has been loaded. [see SetUp()]
     */

    // Get where to save the proximity setup
    const std::string serializationFilepath = TestEnvironment::testfilesDir() + "/workcells/simple_wc/SerializedProximitySetup.xml";

    // Save current proximity setup using the DOMProximitySetupSaver
    rw::loaders::DOMProximitySetupSaver::save(proxSetup, serializationFilepath);

    // Load the workcell that is using the serialized proximity setup
    wcUsingProx = WorkCellLoader::Factory::load(workcellFileUsingSerializedProx);
    serializedProxSetup = rw::proximity::ProximitySetup::get(*wcUsingProx);

    // Perform a collision detection on the default wc state
    CollisionDetector::Ptr cd = rw::common::ownedPtr(new CollisionDetector(
            wcUsingProx,
            rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    CollisionDetector::QueryResult res;
    cd->inCollision(wcUsingProx->getDefaultState(), &res);

    // We expect 0 collisions if the proximity setup has been serialized and loaded properly
    EXPECT_EQ(res.collidingFrames.size(), 0);

    // Compare number of rules in the original proximity setup with the serialized one.
    EXPECT_EQ(proxSetup.getProximitySetupRules().size(), serializedProxSetup.getProximitySetupRules().size());
}

