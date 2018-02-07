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

#include <rw/loaders/path/PathLoaderCSV.hpp>
#include <rw/models/Device.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <vector>

using namespace rw::loaders;
using namespace rw::trajectory;
using namespace rw::math;
using namespace rw::models;
using namespace std;

namespace {
class PathLoaderCSVTest : public ::testing::Test
{
public:
    const string workcellFile = TestEnvironment::testfilesDir() + "workcells/UniversalRobotScene.wc.xml";
    WorkCell::Ptr wc1;
    const string NoTimestampFile = TestEnvironment::testfilesDir() + "paths/6dof_no_timestamp.csv";
    const string timestampedFile = TestEnvironment::testfilesDir() + "paths/6dof_timestamp.csv";

    Device::Ptr robot;

    vector<double> expected_first;
    vector<double> expected_last;

    virtual void SetUp()
    {
        wc1 = WorkCellLoader::Factory::load(workcellFile);
        robot = wc1->findDevice("UR1");
        expected_first = {-1.5,0.0,-1.5,0.0,-1.5,0.0};
        expected_last = {0.0,-1.5,0.0,-1.5,0.0,-1.5};
    }

};

TEST_F(PathLoaderCSVTest, loadPath) {
    QPath path = PathLoaderCSV::loadPath(NoTimestampFile);
    EXPECT_EQ(path.size(), 8); // Test length of loaded path.

    vector<double> first_element = path.front().toStdVector();
    EXPECT_EQ(first_element.size(),6); // Test size of Qs

    vector<double> last_element = path.back().toStdVector();


    EXPECT_EQ(first_element,expected_first);
    EXPECT_EQ(last_element,expected_last);
}

TEST_F(PathLoaderCSVTest,loadStatePath) {
    ASSERT_FALSE(wc1.isNull());
    StatePath path = PathLoaderCSV::loadStatePath(*wc1, NoTimestampFile);

    EXPECT_EQ(path.size(),8); // test path length

    vector<double> first_element = robot->getQ(path.front()).toStdVector();
    vector<double> last_element = robot->getQ(path.back()).toStdVector();

    // Test size of Qs
    EXPECT_EQ(first_element.size(),6);
    EXPECT_EQ(last_element.size(),6);

    // Test elements
    EXPECT_EQ(first_element,expected_first);
    EXPECT_EQ(last_element,expected_last);
}

TEST_F(PathLoaderCSVTest,loadTimedStatePath) {
    ASSERT_FALSE(wc1.isNull());
    TimedStatePath path = PathLoaderCSV::loadTimedStatePath(*wc1, timestampedFile);

    EXPECT_EQ(path.size(),8); // test path length

    EXPECT_EQ(path.front().getTime(),0.5); // test time stamp at front and back
    EXPECT_EQ(path.back().getTime(),4.5);

    //Test if Qs are ok
    vector<double> first_element = robot->getQ(path.front().getValue()).toStdVector();
    vector<double> last_element = robot->getQ(path.back().getValue()).toStdVector();

    // Test size of Qs
    EXPECT_EQ(first_element.size(),6);
    EXPECT_EQ(last_element.size(),6);

    // Test elements
    EXPECT_EQ(first_element,expected_first);
    EXPECT_EQ(last_element,expected_last);
}
}

