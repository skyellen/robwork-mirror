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

#include <rw/common/StringUtil.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <rw/common/Timer.hpp>
#include <rw/common/Ptr.hpp>

#include <rw/common/INIArchive.hpp>
#include <rw/common/BINArchive.hpp>

using namespace rw::common;

BOOST_AUTO_TEST_CASE( SerializationTest )
{
	INIArchive iniarchive;
	iniarchive.open("testfile.ini");
	std::vector<double> dataarr,dataarrres;
	dataarr.push_back(0.3);
	dataarr.push_back(0.5);
	dataarr.push_back(0.6);
	dataarr.push_back(0.7);
	iniarchive.write(dataarr,"arr1");
	iniarchive.write(0.2,"d1");
	iniarchive.writeEnterScope("stuff");
	iniarchive.write(0.2,"data1");
	iniarchive.write(0.6,"data2");
	iniarchive.write(0.3,"data3");
	iniarchive.writeLeaveScope("stuff");

	iniarchive.close();

	iniarchive.open("testfile.ini");
	iniarchive.read(dataarrres,"arr1");
	BOOST_CHECK_CLOSE(iniarchive.readDouble("d1"), 0.2, 0.001);
	iniarchive.readEnterScope("stuff");
	BOOST_CHECK_CLOSE(iniarchive.readDouble("data1"), 0.2, 0.001);
	BOOST_CHECK_CLOSE(iniarchive.readDouble("data2"), 0.6, 0.001);
	BOOST_CHECK_CLOSE(iniarchive.readDouble("data3"), 0.3, 0.001);
	iniarchive.readLeaveScope("stuff");


}
