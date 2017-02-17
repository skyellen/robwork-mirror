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
#include <rw/models/SerialDevice.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/loaders/rwxml/XMLRWLoader.hpp>
#include <rwlibs/calibration.hpp>

using namespace rwlibs::calibration;
using namespace rw::math;
using namespace rw::kinematics;
using rw::models::SerialDevice;


BOOST_AUTO_TEST_CASE( Calibration ) {
#ifdef _WIN32
	_CrtSetDbgFlag(0);
#endif
}
