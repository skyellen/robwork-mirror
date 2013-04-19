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
using namespace rw::math;

struct SerializationData: public rw::common::Serializable {
	SerializationData(){
		data1.push_back(0.3);
		data1.push_back(0.5);
		data1.push_back(0.6);
		data1.push_back(0.7);
		data2 = Q(3,0.1,0.2,0.3);
	}

	void read(InputArchive& iarchive, const std::string& id){

	}
	void write(OutputArchive& oarchive, const std::string& id) const{

		oarchive.write(data1,"data1");
		oarchive.write(data2,"data2");

		oarchive.writeEnterScope("primitives");
		oarchive.write(data3,"data3");
		oarchive.write(data4,"data4");
		oarchive.write(data5,"data5");
		oarchive.write(data6,"data6");
		oarchive.writeLeaveScope("primitives");

	}


	std::vector<double> data1;
	rw::math::Q data2;

	double data3;
	uint32_t data4;
	uint16_t data5;
	uint8_t data6;


};


BOOST_AUTO_TEST_CASE( INIArchiveTest )
{
	SerializationData sdata;

	INIArchive iniarchive;
	iniarchive.open("testfile.ini");

	iniarchive.write( sdata, "sdata" );
	iniarchive.close();

}

BOOST_AUTO_TEST_CASE( BINArchiveTest )
{
	SerializationData sdata;

	BINArchive iniarchive;
	iniarchive.open("testfile.ini");

	iniarchive.write( sdata, "sdata" );
	iniarchive.close();

}
