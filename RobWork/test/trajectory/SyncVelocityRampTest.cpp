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

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

#include <rwlibs/control/SyncVelocityRamp.hpp>


#include <fstream>


using namespace rw;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;
using namespace rw::sandbox;


void SyncVelocityRampTest(){
    BOOST_MESSAGE("- Testing SyncVelocityRamp");
    const size_t n = 6;

    //Test when reaching the velocity limit
    {
        Q dqlimit(n);
        Q ddqlimit(n);
        for (size_t i = 0; i<n; i++) {
            dqlimit(i) = 2;
            ddqlimit(i) = 4;
        }


        SyncVelocityRamp ramp(dqlimit, ddqlimit);
        Q q1(n);
        Q q2(n);
        for (size_t i = 0; i<n; i++) {
            q1(i) = 1;
            q2(i) = 7;
        }

        ramp.setTarget(q1, q2);

        double t = ramp.duration();
        BOOST_CHECK(fabs(t - 3.5) < 1e-12);
        BOOST_CHECK(ramp.x(0) == q1);
        BOOST_CHECK(ramp.x(t) == q2);
        BOOST_CHECK(ramp.x(t/2.0) == (q1+q2)/2.0);

        BOOST_CHECK(ramp.dx(0) == Q::Zero(n));
        BOOST_CHECK(ramp.dx(t) == Q::Zero(n));
        BOOST_CHECK(ramp.dx(t/2) == dqlimit);

        BOOST_CHECK(ramp.ddx(0) == ddqlimit);
        BOOST_CHECK(ramp.ddx(t-0.001) == -ddqlimit);
        BOOST_CHECK(ramp.ddx(t/2.0) == Q::Zero(n));


/*        std::ofstream outfile("velramp.out");
        for (double t = 0; t<ramp.duration(); t += 0.01) {
            size_t index = 1;
            outfile<<ramp.x(t)(index)<<" "<<ramp.dx(t)(index)<<" ";
            outfile<<(ramp.x(t+0.01)-ramp.x(t))(index)/0.01<<" ";
            outfile<<ramp.ddx(t)(index)<<" "<<(ramp.dx(t+0.01)-ramp.dx(t))(1)/0.01<<std::endl;
        }
        */
    }


    //Test when not reaching the velocity limit
    {
        Q dqlimit(n);
        Q ddqlimit(n);
        for (size_t i = 0; i<n; i++) {
            dqlimit(i) = 4;
            ddqlimit(i) = 4;
        }


        SyncVelocityRamp ramp(dqlimit, ddqlimit);
        Q q1(n);
        Q q2(n);
        for (size_t i = 0; i<n; i++) {
            q1(i) = 0.5;
            q2(i) = -0.5;
        }

        ramp.setTarget(q1, q2);

        double t = ramp.duration();
        BOOST_CHECK(t == 1.0);
        BOOST_CHECK(ramp.x(0) == q1);
        BOOST_CHECK(ramp.x(t) == q2);
        BOOST_CHECK(ramp.x(t/2.0) == (q1+q2)/2.0);

        BOOST_CHECK(ramp.dx(0) == Q::Zero(n));
        BOOST_CHECK(ramp.dx(t) == Q::Zero(n));
        BOOST_CHECK(ramp.dx(t/2) == ddqlimit*(-t)/2.0);

        BOOST_CHECK(ramp.ddx(0) == -ddqlimit);
        BOOST_CHECK(ramp.ddx(t-0.001) == ddqlimit);


/*        std::ofstream outfile("velramp.out");
        for (double t = 0; t<ramp.duration(); t += 0.01) {
            size_t index = 1;
            outfile<<ramp.x(t)(index)<<" "<<ramp.dx(t)(index)<<" ";
            outfile<<(ramp.x(t+0.01)-ramp.x(t))(index)/0.01<<" ";
            outfile<<ramp.ddx(t)(index)<<" "<<(ramp.dx(t+0.01)-ramp.dx(t))(1)/0.01<<std::endl;
        }
*/
    }


}


