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

BOOST_AUTO_TEST_CASE( RampInterpolatorTest ) {
    size_t n=5;
    Q dqlimit(n);
    Q ddqlimit(n);
    for (size_t i = 0; i<n; i++) {
        dqlimit(i) = 2;
        ddqlimit(i) = 4;
    }

    Q q1(n);
    Q q2(n);
    for (size_t i = 0; i<n; i++) {
        q1(i) = 1;
        q2(i) = 7;
    }

    RampInterpolator<Q> ramp(q1,q2,dqlimit, ddqlimit);

    double stepSize = ramp.duration()/100.0;
    for(int i=0;i<100;i++){
        double t = stepSize*i;
        Q q = ramp.x(t);
        //std::cout << t << ";" << q(0) << ";" << q(1) << ";" << q(2) << ";" << q(3) << std::endl;
    }
    //std::cout << "duration: " << ramp.duration() << std::endl;


    RPY<> start(0.2,0,0);
    RPY<> end(0.2,0.6,0.9);

    RampInterpolator<Rotation3D<> > rampR(start.toRotation3D(),end.toRotation3D(),0.8,0.1);

    stepSize = rampR.duration()/100.0;
    for(int i=0;i<100;i++){
        double t = stepSize*i;
        Rotation3D<> rot = rampR.x(t);
        EAA<> eaa(rot);
        Vector3D<> axis = eaa.axis();
        double angle = eaa.angle();
        std::cout << t << ";" << angle << ";" << axis(0) << ";" << axis(1) << ";" << axis(2) << std::endl;
    }
    //std::cout << "duration: " << rampR.duration() << std::endl;
    //std::cout << RPY<>( rampR.x(0) ) << std::endl;
    //std::cout << RPY<>( rampR.x(rampR.duration()) ) << std::endl;

    Vector3D<> startP(0,0,0);
    Vector3D<> endP(0,0,0.04);
    RampInterpolator<Vector3D<> > rampP(startP,endP,0.05, 0.005);

    stepSize = rampP.duration()/100.0;
    for(int i=0;i<100;i++){
        double t = stepSize*i;
        Vector3D<> p = rampP.x(t);
        std::cout << t << ";" << p(0) << ";" << p(1) << ";" << p(2) << std::endl;
    }
    std::cout << "duration: " << rampP.duration() << std::endl;
    std::cout << Vector3D<>( rampP.x(0) ) << std::endl;
    std::cout << Vector3D<>( rampP.x(rampP.duration()) ) << std::endl;


    /*
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
   */
}


BOOST_AUTO_TEST_CASE( IteratorTest ) {
	QInterpolatorTrajectory traj;
	Q q1(1);
	Q q2(1);
	Q q3(1);
	Q q4(1);
	q1(0) = 1;
	q2(0) = -1;
	q3(0) = 2;
	q4(0) = -2; 
	QLinearInterpolator int1(q1, q2, 1);
	QLinearInterpolator int2(q2, q3, 1);
	QLinearInterpolator int3(q3, q4, 1);

	traj.add(&int1);
	traj.add(&int2);
	traj.add(&int3);

	TrajectoryIterator<Q>::Ptr pIterator = traj.getIterator(0.01);
	TrajectoryIterator<Q>& iterator = *pIterator;
	for (double t = 0; t<1.0; t += 0.01) {
		BOOST_CHECK(iterator.x() == int1.x(t));
		iterator++;
	}
	iterator += 1;
	for (double t = 0; t>0; t -= 0.01) {
		BOOST_CHECK(iterator.x() == int2.x(t));
		iterator--;
	}

	iterator += 1;
	
	for (double t = iterator.getTime()-2; t<1.0; t += 0.1) {
		BOOST_CHECK(iterator.x() == int3.x(t));
		iterator += 0.1;
	}


}

BOOST_AUTO_TEST_CASE( TrajectorySequenceTest ) {
	QInterpolatorTrajectory trajectory1;
	QInterpolatorTrajectory trajectory2;
	QInterpolatorTrajectory trajectory3;
	Q q1(2);
	q1(0) = 0; q1(1) = 0;
	Q q2(2);
	q2(0) = 1; q2(1) = 1;
	Q q3(2);
	q3(0) = -1; q3(1) = 2;
	Q q4(2);
	q4(0) = 1; q4(1) = 3;
	QLinearInterpolator int1(q1, q2, 1);
	QLinearInterpolator int2(q2, q3, 2);
	QLinearInterpolator int3(q3, q4, 3);

	trajectory1.add(&int1);
	trajectory2.add(&int2);
	trajectory3.add(&int3);

	std::vector<QTrajectory::Ptr> trajectories;
	trajectories.push_back(&trajectory1);
	trajectories.push_back(&trajectory2);
	trajectories.push_back(&trajectory3);

	TrajectorySequence<Q> seq(trajectories);
	
	BOOST_CHECK(seq.x(0) == trajectory1.x(0));
	BOOST_CHECK(seq.x(0.5) == trajectory1.x(0.5));
	BOOST_CHECK(seq.x(1) == trajectory1.x(1));
	BOOST_CHECK(seq.x(1) == trajectory2.x(0)); 
	BOOST_CHECK(seq.x(1.5) == trajectory2.x(0.5));
	BOOST_CHECK(seq.x(2.5) == trajectory2.x(1.5));
	BOOST_CHECK(seq.x(3) == trajectory3.x(0));
	BOOST_CHECK(seq.x(4) == trajectory3.x(1));
	BOOST_CHECK(seq.x(6) == trajectory3.x(3));

	const double dt = 0.1;
	TrajectoryIterator<Q>::Ptr pIterator = seq.getIterator(dt);
	TrajectoryIterator<Q>& iterator = *pIterator;
	BOOST_CHECK(iterator.x() == seq.x(0));
	iterator++;
	BOOST_CHECK(iterator.x() == seq.x(dt));
	iterator += 1;
	BOOST_CHECK(iterator.x() == seq.x(1+dt));
	iterator--;
	BOOST_CHECK(iterator.x() == seq.x(1));


}

BOOST_AUTO_TEST_CASE( CubicSplineInterpolation ){

    {
        BOOST_TEST_MESSAGE("Testing if exceptions are cast when too small paths are given as arguments!");
        // check throw on empty QPath
        QPath::Ptr path = ownedPtr(new QPath());
        BOOST_CHECK_THROW( CubicSplineFactory::makeNaturalSpline(path), rw::common::Exception );
        path->push_back(Q(3));
        BOOST_CHECK_THROW( CubicSplineFactory::makeNaturalSpline(path), rw::common::Exception );

    }

    // here we test the cubic path interpolation
    {
        BOOST_TEST_MESSAGE("Testing natural spline generation on PATH");
        QPath::Ptr path = ownedPtr(new QPath());
        Q q = Q::zero(7);
        q(0) = 1;
        path->push_back(q);
        q(0) = 8;
        path->push_back(q);
        q(0) = -1;
        path->push_back(q);
        q(0) = 4;
        path->push_back(q);

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

        BOOST_TEST_MESSAGE("Testing clamped spline generation on PATH");
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

        BOOST_CHECK_CLOSE( traj->dx(0)[0], 1, 0.001 );
        BOOST_CHECK_CLOSE( traj->dx( traj->duration() )[0], -1, 0.001 );

        //for(double t=traj->startTime(); t<=traj->endTime();t+=traj->duration()*0.01){
        //    std::cout << t << "\n\t" << traj->x(t) << "\n\t" << traj->dx(t) << "\n\t" << traj->ddx(t)<< std::endl;
        //}

    }


    // here we test the cubic path interpolation
    //////// ************ for some reason this makes assertion
    {
        BOOST_TEST_MESSAGE("Testing natural spline generation on TIMEDPATH");
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

        BOOST_TEST_MESSAGE("Testing clamped spline generation on TIMEDPATH");
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


