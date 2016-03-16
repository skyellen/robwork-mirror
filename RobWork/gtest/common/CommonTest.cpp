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

#include <rw/common/Ptr.hpp>
#include <rw/common/Timer.hpp>
#include <rw/common/Event.hpp>

#include <boost/any.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

using namespace rw::common;

class A {
public:
	virtual ~A() {}
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

TEST(PtrTest, CastToSubType) {
    std::vector<APtr> aptrs;
    B* const b = new B();
    aptrs.push_back( ownedPtr(b) );

    std::vector<APtr> aptrsCopy = aptrs;
    const APtr aptr = aptrsCopy[0];
    A* const a  = aptr.get();
    B* const bcast = dynamic_cast<B*>(a);
    EXPECT_TRUE(bcast != NULL);
    const BPtr bptr = aptr.cast<B>();
    EXPECT_TRUE(bptr != NULL);
    EXPECT_FALSE(bptr.isNull());
}

TEST(TimerTest, GetTimeFunctions) {
	Timer t1(3030);
	EXPECT_EQ(t1.getTimeMs(),3030);
	EXPECT_EQ(t1.getTimeSec(),3);
}

TEST(TimerTest, ToStringFunctions) {
	std::string tstr1 = Timer(1,2,30,10).toString("hh:mm:ss");
	EXPECT_EQ(tstr1,"01:02:30") << "Should be: " << tstr1;
	std::string tstr2 = Timer(1,2,2,10).toString("hh:mm");
	EXPECT_EQ(tstr2,"01:02") << "Should be: " << tstr2;
	std::string tstr3 = Timer(1,2,30,10).toString("h:m:s");
	EXPECT_EQ(tstr3,"1:2:30") << "Should be: " << tstr3;
	std::string tstr4 = Timer(1,2,2,10).toString("h:m");
	EXPECT_EQ(tstr4,"1:2") << "Should be: " << tstr4;
}

typedef boost::function<void(const std::string&, boost::any)> GenericAnyEventListener;
typedef rw::common::Event<GenericAnyEventListener, const std::string&, boost::any> GenericAnyEvent;

namespace {
	bool b1,b2;
	void cb1(const std::string& id, boost::any data){
		b1 = true;
	}

	void cb2(const std::string& id, boost::any data){
		b2 = true;
	}

	class AnyEventListener {
	public:
		AnyEventListener(bool &check): _check(check) {}
		void cb(const std::string& id, boost::any data) {
			_check = true;
		}
		bool &_check;
	};
}

TEST( EventTest, GlobalCallbackFunctions ) {
	GenericAnyEvent event;
	event.add( boost::bind(&cb1, _1, _2), (void*)&cb1 );
	event.add( boost::bind(&cb2, _1, _2), (void*)&cb2 );

	const boost::any data;
	b1=false;
	b2=false;
	event.fire("Msg1",data);
	EXPECT_TRUE(b1);
	EXPECT_TRUE(b2);

	// remove cb1
	b1=false;
	b2=false;
	event.remove((void*)&cb1 );
	event.fire("Msg2",data);
	EXPECT_FALSE(b1);
	EXPECT_TRUE(b2);
}

TEST( EventTest, EventListener ) {
	AnyEventListener listener1(b1);
	AnyEventListener listener2(b2);

	GenericAnyEvent event;
	event.add( boost::bind(&AnyEventListener::cb, &listener1, _1, _2), &listener1 );
	event.add( boost::bind(&AnyEventListener::cb, &listener2, _1, _2), &listener2 );

	const boost::any data;
	b1=false;
	b2=false;
	event.fire("Msg1",data);
	EXPECT_TRUE(b1);
	EXPECT_TRUE(b2);

	// remove cb1
	b1=false;
	b2=false;
	event.remove(&listener1);
	event.fire("Msg2",data);
	EXPECT_FALSE(b1);
	EXPECT_TRUE(b2);
}
