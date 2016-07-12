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
//#include <rw/models/SerialDevice.hpp>
//#include <rw/kinematics/State.hpp>
//#include <rw/math/Vector3D.hpp>
//#include <rw/math/Rotation3D.hpp>
#include <rw/trajectory/Path.hpp>

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>

#include <cmath>

using rw::math::Q;
using rw::models::WorkCell;
using rw::trajectory::QPath;
using namespace rw::loaders;

/*
namespace
{
    double norm_inf(const Vector3D<>& v)
    {
        return norm_inf(v.m());
    }

    double norm_inf(const Q& v)
    {
        return norm_inf(v.m());
    }

    bool isZero(double x) { return fabs(x) < 1e-14; }
}

BOOST_AUTO_TEST_CASE( TULLoaderTest )
{
    BOOST_TEST_MESSAGE("TULTestTestSuite");
    BOOST_TEST_MESSAGE("- Loading workcell file");
    WorkCellPtr workcell = WorkCellLoader::load(testFilePath() + "PA10/PA10.wu");

    BOOST_REQUIRE(NULL != workcell.get());
    BOOST_REQUIRE(workcell->getDevices().size() == 1);

    BOOST_TEST_MESSAGE("- Testing nr of devices");
    SerialDevice* device = (SerialDevice*)workcell->getDevices()[0];
    State state = workcell->getDefaultState();

    BOOST_CHECK(
        norm_inf(
            device->baseTend(state).R().m() -
            Rotation3D<>(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0).m()) < 1e-6);

    BOOST_CHECK(norm_inf(device->baseTend(state).P() - Vector3D<>(0.195,0,1.581)) < 1e-6);

    std::pair<Q, Q> bounds = device->getBounds();
    Q newFirst = bounds.first*2;
    Q newSecond = bounds.second*3;
    std::pair<Q, Q> boundsTmp(newFirst, newSecond);

    device->setBounds(boundsTmp);
    std::pair<Q, Q> newbounds = device->getBounds();
    for (size_t i = 0; i<bounds.first.size(); i++) {
        BOOST_CHECK(isZero(2*bounds.first(i) - newbounds.first(i)));
        BOOST_CHECK(isZero(3*bounds.second(i) - newbounds.second(i)));
    }

    Q vellimits = device->getVelocityLimits();
    device->setVelocityLimits(vellimits*0.5);
    Q newvellimits = device->getVelocityLimits();
    for (size_t i = 0; i<vellimits.size(); i++) {
        BOOST_CHECK(0.5*vellimits(i) == newvellimits(i));
    }

    Q acclimits = device->getAccelerationLimits();
    device->setAccelerationLimits(acclimits*5);
    Q newacclimits = device->getAccelerationLimits();
    for (size_t i = 0; i<acclimits.size(); i++) {
        BOOST_CHECK(isZero(5*acclimits(i) - newacclimits(i)));
    }
}
*/ 

BOOST_AUTO_TEST_CASE( WorkCellLoaderTest ) 
{
    BOOST_TEST_MESSAGE("- Testing WorkCellLoader");
    // Load a tree device that has revolute joints only.
	std::string file = testFilePath() + "SchunkHand/SchunkHand.xml";
	rw::common::Ptr<WorkCell> workcell = WorkCellLoader::Factory::load(file);


	BOOST_CHECK(workcell != NULL);

}

BOOST_AUTO_TEST_CASE( PathLoaderTest )
{
    BOOST_TEST_MESSAGE("- Testing PathLoader");
    QPath path;
    Q q(3);
    for (int i = 0; i < 100; i++) {
        for (size_t j = 0; j < q.size(); j++)
            q(j) = rand();
        path.push_back(q);
    }
    PathLoader::storePath(path, "path.pth");

    QPath path2 = PathLoader::loadPath("path.pth");
    BOOST_CHECK(path.size() == path2.size());

    QPath::iterator it1 = path.begin();
    QPath::iterator it2 = path2.begin();
    for (; it1 != path.end(); ++it1, ++it2) {
        BOOST_CHECK(*it1 == *it2);
    }
}
