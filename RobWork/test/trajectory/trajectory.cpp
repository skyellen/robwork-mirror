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

#include <boost/test/unit_test.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

#include <rw/trajectory.hpp>


#include <fstream>


using namespace rw;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rw::common;

namespace {
    bool isContinues(QTrajectory& traj, double stepSize){
        double t=traj.startTime();
        Q last_x   = traj.x(t);
        Q last_dx  = traj.dx(t);
        Q last_ddx = traj.ddx(t);

        for(; t<=traj.endTime();t+=traj.duration()*stepSize){
            //if( MetricUtil::dist2(last_x,traj.x(t))>0.1 )
            //    return false;
            //if( MetricUtil::dist2(last_dx,traj.dx(t))>0.1 )
            //    return false;
            //if( MetricUtil::dist2(last_ddx,traj.ddx(t))>0.1 )
            //    return false;
            BOOST_CHECK_SMALL( MetricUtil::dist2(last_x,traj.x(t)), 0.15);
            BOOST_CHECK_SMALL( MetricUtil::dist2(last_dx,traj.dx(t)), 0.15);
            //BOOST_CHECK_SMALL( MetricUtil::dist2(last_ddx,traj.ddx(t)), 0.1);

            last_x = traj.x(t);
            last_dx = traj.dx(t);
            last_ddx = traj.ddx(t);

            //std::cout << t << "\t" << traj->x(t)[0] << "\t" << traj->dx(t)[0] << "\t" << traj->ddx(t)[0]<< std::endl;
        }
        return true;
    }

}


BOOST_AUTO_TEST_CASE( CubicSplineInterpolation ){

    {
        BOOST_MESSAGE("Testing if exceptions are cast when too small paths are given as arguments!");
        // check throw on empty QPath
        QPathPtr path = ownedPtr(new QPath());
        BOOST_CHECK_THROW( CubicSplineFactory::makeNaturalSpline(path), rw::common::Exception );
        path->push_back(Q(3));
        BOOST_CHECK_THROW( CubicSplineFactory::makeNaturalSpline(path), rw::common::Exception );

    }

    // here we test the cubic path interpolation
    {
        BOOST_MESSAGE("Testing natural spline generation on PATH");
        QPathPtr path = ownedPtr(new QPath());
        Q q = Q::zero(7);
        q(0) = 1;
        path->push_back(q);
        q(0) = 8;
        path->push_back(q);
        q(0) = -1;
        path->push_back(q);
        q(0) = 4;
        path->push_back(q);

        Q qzerovel = Q::zero(7);

        QTrajectory::Ptr traj = CubicSplineFactory::makeNaturalSpline(path);
        BOOST_CHECK(traj!=NULL);
        BOOST_CHECK_CLOSE( traj->duration(), (double)path->size()-1, 0.0001 );
        //std::cout << "sfdaf" << std::endl;
        //for(size_t time = 0; time < path->size(); time++)
        //    RW_CHECK_Q_CLOSE( (*path)[time], traj->x((double)time), 0.001 );
        BOOST_CHECK( isContinues(*traj, traj->duration()*0.0001) );

        //for(double t=traj->startTime(); t<=traj->endTime();t+=traj->duration()*0.01){
        //    std::cout << t << "\t" << traj->x(t)[0] << "\t" << traj->dx(t)[0] << "\t" << traj->ddx(t)[0]<< std::endl;
        //}

        BOOST_MESSAGE("Testing clamped spline generation on PATH");
        Q start = Q::zero(7), end = Q::zero(7);
        start[0] = 1;
        end[0] = -1;

        traj = CubicSplineFactory::makeClampedSpline(path, start, end);

        BOOST_CHECK(traj!=NULL);
        BOOST_CHECK_CLOSE( traj->duration(), (double)path->size()-1, 0.0001 );
        //std::cout << "sfdaf" << std::endl;
        //for(size_t time = 0; time < path->size(); time++)
        //    RW_CHECK_Q_CLOSE( (*path)[time], traj->x((double)time), 0.001 );
        BOOST_CHECK( isContinues(*traj, traj->duration()*0.0001) );

        //for(double t=traj->startTime(); t<=traj->endTime();t+=traj->duration()*0.01){
        //    std::cout << t << "\t" << traj->x(t)[0] << "\t" << traj->dx(t)[0] << "\t" << traj->ddx(t)[0]<< std::endl;
        //}

    }


    // here we test the cubic path interpolation
    //////// ************ for some reason this makes assertion
    {
        BOOST_MESSAGE("Testing natural spline generation on TIMEDPATH");
        Ptr< TimedQPath > path = rw::common::ownedPtr(new TimedQPath());
        Q q = Q::zero(7);
        q(0) = 1;
        path->push_back(Timed<Q>(0,q));
        q(0) = 8;
        path->push_back(Timed<Q>(1,q));
        q(0) = -1;
        path->push_back(Timed<Q>(4,q));
        q(0) = 4;
        path->push_back(Timed<Q>(6,q));

        Q qzerovel = Q::zero(7);

        QTrajectory::Ptr traj = CubicSplineFactory::makeNaturalSpline(  path );

        BOOST_CHECK(traj!=NULL);
        //std::cout << "sfdaf" << std::endl;
        BOOST_CHECK_CLOSE( traj->duration(), path->back().getTime()-path->front().getTime(), 0.0001 );
        //std::cout << "sfdaf" << std::endl;
        //for(size_t time = 0; time < path->size(); time++)
        //    RW_CHECK_Q_CLOSE( (*path)[time].getValue(), traj->x((double)time), 0.001 );
        BOOST_CHECK( isContinues(*traj, traj->duration()*0.0001) );

        //for(double t=traj->startTime(); t<=traj->endTime();t+=traj->duration()*1/80){
        //    std::cout << t << "\t" << traj->x(t)[0] << "\t" << traj->dx(t)[0] << "\t" << traj->ddx(t)[0]<< std::endl;
        //}

        BOOST_MESSAGE("Testing clamped spline generation on TIMEDPATH");
        Q start = Q::zero(7), end = Q::zero(7);
        start[0] = 1;
        end[0] = -1;
        traj = CubicSplineFactory::makeClampedSpline(path, start, end);
        //std::cout << "Clamped made " << std::endl;
        BOOST_CHECK(traj!=NULL);
        BOOST_CHECK_CLOSE( traj->duration(), path->back().getTime()-path->front().getTime(), 0.0001 );
        //std::cout << "sfdaf" << std::endl;
        //for(size_t time = 0; time < path->size(); time++)
        //    RW_CHECK_Q_CLOSE( (*path)[time], traj->x((double)time), 0.001 );
        BOOST_CHECK( isContinues(*traj, traj->duration()*0.0001) );

        //for(double t=traj->startTime(); t<=traj->endTime();t+=traj->duration()*0.01){
        //    std::cout << t << "\t" << traj->x(t)[0] << "\t" << traj->dx(t)[0] << "\t" << traj->ddx(t)[0]<< std::endl;
        //}

    }



}


