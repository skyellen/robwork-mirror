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
#include <rw/math/Vector2D.hpp>

using namespace rw::common;
using namespace rw::math;

struct SerializationData: public rw::common::Serializable {
	SerializationData(){
		data1.push_back(0.3);
		data1.push_back(0.5);
		data1.push_back(0.6);
		data1.push_back(0.7);
		data2 = rw::math::Math::ranQ( Q(3,0,0,0), Q(3,1,1,1) );

	}

	bool operator== (const SerializationData& rhs) const {
	    bool result = true;
	    result &= data1.size() == rhs.data1.size();
	    if(data1.size() == rhs.data1.size()){
	        for(size_t i=0;i<data1.size();i++){
	            result &= data1[i] == rhs.data1[i];
	        }
	    }
	    result &= data2 == rhs.data2;
	    result &= data3 == rhs.data3;
	    result &= data4 == rhs.data4;
	    result &= data5 == rhs.data5;
	    result &= data6 == rhs.data6;
	    result &= data7 == rhs.data7;
	    return result;
	}


	void read(InputArchive& iarchive, const std::string& id){
        iarchive.read(data1,"data1");
        iarchive.read(data2,"data2");

        iarchive.readEnterScope("primitives");
        iarchive.read(data3,"data3");
        iarchive.read(data4,"data4");
        iarchive.read(data5,"data5");
        iarchive.read(data6,"data6");
        iarchive.read(data7,"data7");
        iarchive.readLeaveScope("primitives");
	}

	void write(OutputArchive& oarchive, const std::string& id) const{
		oarchive.write(data1,"data1");
		oarchive.write(data2,"data2");

		oarchive.writeEnterScope("primitives");
		oarchive.write(data3,"data3");
		oarchive.write(data4,"data4");
		oarchive.write(data5,"data5");
		oarchive.write(data6,"data6");
		oarchive.write(data7,"data7");
		oarchive.writeLeaveScope("primitives");
	}


	std::vector<double> data1;
	rw::math::Q data2;

	double data3;
	uint32_t data4;
	uint16_t data5;
	uint8_t data6;

	rw::math::Vector2D<> data7;
};


BOOST_AUTO_TEST_CASE( INIArchiveTest )
{
	SerializationData sdata, sdata_in;
	BOOST_CHECK( true );
	{
	    RW_WARN("1");
        INIArchive iniarchive;
        RW_WARN("1");
        iniarchive.open("testfile.ini");
        RW_WARN("1");
        iniarchive.write( sdata, "sdata" );
        RW_WARN("1");
        iniarchive.close();
        RW_WARN("1");
	}
	BOOST_CHECK( true );
	{
	    RW_WARN("1");
	    INIArchive iniarchive;
	    RW_WARN("1");
	    iniarchive.open("testfile.ini");
	    RW_WARN("1");
	    iniarchive.read( sdata_in, "sdata");
	    RW_WARN("1");
	    iniarchive.close();
	    RW_WARN("1");
	}
	BOOST_CHECK( sdata == sdata_in );
}

BOOST_AUTO_TEST_CASE( BINArchiveTest )
{
	SerializationData sdata;

	BINArchive iniarchive;
	iniarchive.open("testfile.ini");

	iniarchive.write( sdata, "sdata" );
	iniarchive.close();

}


BOOST_AUTO_TEST_CASE( BoostXMLParser )
{


}

