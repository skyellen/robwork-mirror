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

#include <boost/test/unit_test.hpp>
#include <iostream>

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

BOOST_AUTO_TEST_CASE( CommonTest )
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
