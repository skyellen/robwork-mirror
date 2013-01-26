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

using namespace rw::common;

class A {
public:
    virtual void print(){
        std::cout << "A" << std::endl;
    }
};

typedef rw::common::Ptr<A> APtr;

class B: public A {
public:
    virtual void print(){
        std::cout << "B" << std::endl;
    }
};
typedef rw::common::Ptr<B> BPtr;

BOOST_AUTO_TEST_CASE( PtrTest )
{
    std::vector<APtr> aptrs;
    B *b = new B();
    aptrs.push_back( ownedPtr(b) );

    std::vector<APtr> aptrsCopy = aptrs;
    APtr aptr = aptrsCopy[0];
    A* a  = aptr.get();
    B* bcast = dynamic_cast<B*>(a);
    BOOST_CHECK(bcast != NULL);
}

BOOST_AUTO_TEST_CASE( TimerTest )
{
	Timer t1(3030);
	BOOST_CHECK(t1.getTimeMs()==3030);
	BOOST_CHECK(t1.getTimeSec()==3);

	std::string tstr1 = Timer(1,2,30,10).toString("hh:mm:ss");
	BOOST_CHECK_MESSAGE(tstr1 == "01:02:30", "Should be: " << tstr1);
	std::string tstr2 = Timer(1,2,2,10).toString("hh:mm");
	BOOST_CHECK_MESSAGE(tstr2 == "01:02", "Should be: " << tstr2);
	std::string tstr3 = Timer(1,2,30,10).toString("h:m:s");
	BOOST_CHECK_MESSAGE(tstr3 == "1:2:30", "Should be: " << tstr3);
	std::string tstr4 = Timer(1,2,2,10).toString("h:m");
	BOOST_CHECK_MESSAGE(tstr4 == "1:2", "Should be: " << tstr4);

}
