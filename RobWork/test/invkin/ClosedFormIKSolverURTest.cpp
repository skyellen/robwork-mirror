/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>

using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;
using namespace rw::invkin;

BOOST_AUTO_TEST_CASE( ClosedFormIKSolverURTest ){
	static const double EPS = 1e-14;

    BOOST_MESSAGE("- Testing ClosedFormIKSolverUR");
	const WorkCell::Ptr wc = WorkCellFactory::load(testFilePath() + "devices/UR6855A/UR6855A.wc.xml");
	BOOST_REQUIRE(wc != NULL);
	SerialDevice::Ptr device = wc->findDevice<SerialDevice>("UR-6-85-5-A");
	BOOST_REQUIRE(device != NULL);
	const ClosedFormIKSolverUR solver(device,wc->getDefaultState());

	{
		State state = wc->getDefaultState();
	    BOOST_MESSAGE("- - Testing configuration q={0.352,-2.408,-0.785,-1.78,2.199,0.785}");
		const Q qRef = Q(6,0.352,-2.408,-0.785,-1.78,2.199,0.785);
		device->setQ(qRef,state);
		const Transform3D<> T = device->baseTend(state);
		const std::vector<Q> solutions = solver.solve(T, state);
	    BOOST_MESSAGE("- - - Found " << solutions.size() << " solutions: ");
		BOOST_CHECK(solutions.size() > 0);
		bool found = false;
		BOOST_FOREACH(const Q& sol, solutions) {
		    BOOST_MESSAGE("- - - - " << sol);
			if ((sol-qRef).normInf() <= EPS) {
				found = true;
			}
			device->setQ(sol,state);
			const Transform3D<> Tfound = device->baseTend(state);
			BOOST_CHECK_SMALL((Tfound.P()-T.P()).normInf(),EPS);
			BOOST_CHECK(T.R().equal(Tfound.R(),EPS));
		}
		BOOST_CHECK(found);
	}
}
