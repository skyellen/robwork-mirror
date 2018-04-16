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

#include <rw/math/Q.hpp>

#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation2D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Constants.hpp>

#include <rw/math/MetricUtil.hpp>
#include <rw/math/MetricFactory.hpp>

#include <sstream>
#include <iomanip>
#include <string>

using namespace rw::math;

BOOST_AUTO_TEST_CASE(MetricTest)
{
	BOOST_TEST_MESSAGE("- Testing MetricFactory");
	Vector2D<> v21(0.1, 0.2);
	Vector3D<> v31(0.1, 0.2, 0.3);
	Q q1(3, 0.1, 0.2, 0.3);
	Q q2(3, 0.1, 0.2, 0.3);

	{
		ManhattanMetric<Vector2D<> > mh1;
		ManhattanMetric<Vector3D<> > mh2;
		ManhattanMetric<Q> mh3;

		BOOST_CHECK_CLOSE( 0.1+0.2, mh1.distance(v21), 0.00001);
		BOOST_CHECK_CLOSE( 0.1+0.2+0.3, mh2.distance(v31), 0.00001);
		BOOST_CHECK_CLOSE( 0.1+0.2+0.3, mh3.distance(q1), 0.00001);

		BOOST_CHECK_CLOSE( 0, mh1.distance(v21, v21), 0.00001);
		BOOST_CHECK_CLOSE( 0, mh2.distance(v31,v31), 0.00001);
		BOOST_CHECK_CLOSE( 0, mh3.distance(q1,q1), 0.00001);

	}

	{
		WeightedManhattanMetric<Vector2D<> > mh1(Vector2D<>(1.0,1.0));
		WeightedManhattanMetric<Vector3D<> > mh2(Vector3D<>(1.0,1.0,1.0));
		WeightedManhattanMetric<Q> mh3( Q(3,1.0,1.0,1.0) );

		BOOST_CHECK_CLOSE( 0.1+0.2, mh1.distance(v21), 0.00001);
		BOOST_CHECK_CLOSE( 0.1+0.2+0.3, mh2.distance(v31), 0.00001);
		BOOST_CHECK_CLOSE( 0.1+0.2+0.3, mh3.distance(q1), 0.00001);

		BOOST_CHECK_CLOSE( 0, mh1.distance(v21, v21), 0.00001);
		BOOST_CHECK_CLOSE( 0, mh2.distance(v31,v31), 0.00001);
		BOOST_CHECK_CLOSE( 0, mh3.distance(q1,q1), 0.00001);
	}


	{
		EuclideanMetric<Vector2D<> > mh1;
		EuclideanMetric<Vector3D<> > mh2;
		EuclideanMetric<Q> mh3;

		BOOST_CHECK_CLOSE( std::sqrt(0.1*0.1+0.2*0.2), mh1.distance(v21), 0.00001);
		BOOST_CHECK_CLOSE( std::sqrt(0.1*0.1+0.2*0.2+0.3*0.3), mh2.distance(v31), 0.00001);
		BOOST_CHECK_CLOSE( std::sqrt(0.1*0.1+0.2*0.2+0.3*0.3), mh3.distance(q1), 0.00001);

		BOOST_CHECK_CLOSE( 0, mh1.distance(v21, v21), 0.00001);
		BOOST_CHECK_CLOSE( 0, mh2.distance(v31,v31), 0.00001);
		BOOST_CHECK_CLOSE( 0, mh3.distance(q1,q1), 0.00001);

	}

	{
		WeightedEuclideanMetric<Vector2D<> > mh1(Vector2D<>(1.0,1.0));
		WeightedEuclideanMetric<Vector3D<> > mh2(Vector3D<>(1.0,1.0,1.0));
		WeightedEuclideanMetric<Q> mh3( Q(3,1.0,1.0,1.0) );

		BOOST_CHECK_CLOSE( std::sqrt(0.1*0.1+0.2*0.2), mh1.distance(v21), 0.00001);
		BOOST_CHECK_CLOSE( std::sqrt(0.1*0.1+0.2*0.2+0.3*0.3), mh2.distance(v31), 0.00001);
		BOOST_CHECK_CLOSE( std::sqrt(0.1*0.1+0.2*0.2+0.3*0.3), mh3.distance(q1), 0.00001);

		BOOST_CHECK_CLOSE( 0, mh1.distance(v21, v21), 0.00001);
		BOOST_CHECK_CLOSE( 0, mh2.distance(v31,v31), 0.00001);
		BOOST_CHECK_CLOSE( 0, mh3.distance(q1,q1), 0.00001);

	}

	{
		InfinityMetric<Vector2D<> > mh1;
		InfinityMetric<Vector3D<> > mh2;
		InfinityMetric<Q> mh3;

		BOOST_CHECK_CLOSE( 0.2, mh1.distance(v21), 0.00001);
		BOOST_CHECK_CLOSE( 0.3, mh2.distance(v31), 0.00001);
		BOOST_CHECK_CLOSE( 0.3, mh3.distance(q1), 0.00001);

		BOOST_CHECK_CLOSE( 0, mh1.distance(v21, v21), 0.00001);
		BOOST_CHECK_CLOSE( 0, mh2.distance(v31,v31), 0.00001);
		BOOST_CHECK_CLOSE( 0, mh3.distance(q1,q1), 0.00001);
	}

}

BOOST_AUTO_TEST_CASE(Rotation2DTest)
{
    BOOST_TEST_MESSAGE("- Testing Rotation2D");

    const Vector2D<std::string> i("i1", "i2");
    const Vector2D<std::string> j("j1", "j2");
    const Rotation2D<std::string> r4(i, j);

    BOOST_CHECK(r4(1,0) == "i2");
    BOOST_CHECK(inverse(r4)(1, 0) == "j1");

    const Rotation2D<> r1 = Rotation2D<>::identity();
    const Vector2D<> v1(1, 2);
    BOOST_CHECK_SMALL( MetricUtil::normInf(v1 - r1 * v1), 0.0000001);

	Rotation2D<int> ri;
	ri = cast<int>(r1);
	for (size_t i = 0; i < 2; i++)
		for (size_t j = 0; j < 2; j++)
			BOOST_CHECK((int)r1(i, j) == ri(i, j));
	ri = rw::math::cast<int>(r1); // qualified lookup
	for (size_t i = 0; i < 2; i++)
		for (size_t j = 0; j < 2; j++)
			BOOST_CHECK((int)r1(i, j) == ri(i, j));

    /* Test comparison operators operator== and operator!= */
    const Rotation2D<> rotcomp1(1.1, -2.2, 3.3, 4.4);
    const Rotation2D<> rotcomp2(1.1, -2.2, 3.3, 4.4);
    BOOST_CHECK(rotcomp1 == rotcomp2);
    BOOST_CHECK(!(rotcomp1 != rotcomp2));

    const Rotation2D<> rotcomp3(-1.1, 2.2, 3.3, 4.4);
    BOOST_CHECK(rotcomp1 != rotcomp3);
    BOOST_CHECK(!(rotcomp1 == rotcomp3));

}

BOOST_AUTO_TEST_CASE(Rotation3DTest)
{
    BOOST_TEST_MESSAGE("- Testing Rotation3D");

    const Vector3D<std::string> i("i1", "i2", "i3");
    const Vector3D<std::string> j("j1", "j2", "j3");
    const Vector3D<std::string> k("k1", "k2", "k3");
    const Rotation3D<std::string> r4(i, j, k);

    BOOST_CHECK(r4(1,0) == "i2");
    BOOST_CHECK(inverse(r4)(1, 0) == "j1");

    const Rotation3D<> r1 = Rotation3D<>::identity();
    const Vector3D<> v1(1, 2, 3);
    BOOST_CHECK(MetricUtil::normInf(v1 - r1 * v1) == 0);

    const EAA<> eaa(Vector3D<>(1, 0, 0), Pi / 2);
    const Rotation3D<> r3 = eaa.toRotation3D();

    BOOST_CHECK(LinearAlgebra::isSO(r3.e()));
    BOOST_CHECK(r3.m().size1() == r3.m().size2() && r3.m().size1() == 3);

    BOOST_CHECK(r1 == r1);
    BOOST_CHECK(!(r1 == r3));

	Rotation3D<int> ri;
	ri = cast<int>(r3);
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
			BOOST_CHECK((int)r3(i, j) == ri(i, j));
	ri = rw::math::cast<int>(r3); // qualified lookup
	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
			BOOST_CHECK((int)r3(i, j) == ri(i, j));

    /* Test comparison operators operator== and operator!= */
    const EAA<> eaacomp1(Pi / 2, 0, 0);
    const Rotation3D<> rotcomp1 = eaacomp1.toRotation3D();
        
    const EAA<> eaacomp2(Pi / 2, 0, 0);
    const Rotation3D<> rotcomp2 = eaacomp2.toRotation3D();

    BOOST_CHECK(rotcomp1 == rotcomp2);
    BOOST_CHECK(!(rotcomp1 != rotcomp2));

    const EAA<> eaacomp3(Pi / 4, 0, 0);
    const Rotation3D<> rotcomp3 = eaacomp3.toRotation3D();
    BOOST_CHECK(rotcomp1 != rotcomp3);
    BOOST_CHECK(!(rotcomp1 == rotcomp3));

}


BOOST_AUTO_TEST_CASE(QTest)
{
	BOOST_TEST_MESSAGE("- Testing Q");
	{
		double arr[] = {0.1, 0.2, 0.3};
		Q q1(3,arr);
		BOOST_CHECK(q1.size()==3);
		BOOST_CHECK(q1(2)==0.3);
		BOOST_CHECK(q1[1]==0.2);

		std::stringstream sstr;
		sstr << std::setprecision(16) << q1;
		BOOST_CHECK(sstr.str()!="");
		Q q2;
		sstr >> q2;
		BOOST_CHECK(q2.size()==3);
		BOOST_CHECK_CLOSE(q1(1),q2(1),0.00001);

		Q q3(4,0.1,0.2,0.3,0.4);
		BOOST_CHECK(!(q3==q1));
		Q q4(3,0.1,0.2,0.4);
		BOOST_CHECK(!(q4==q1));

		Q q34 = concat(q3,q4);
		BOOST_CHECK(q34.size()==q3.size()+q4.size());
		BOOST_CHECK(q34(1)==q3(1));
		BOOST_CHECK(q34(q3.size()+1)==q4(1));

		double dprod = dot(q1,q1);
		BOOST_CHECK_CLOSE(dprod, q1(0)*q1(0)+q1(1)*q1(1)+q1(2)*q1(2),0.0001);

		// testing constructors
		Q qc1(1,0.1);
		Q qc2(2,0.1,0.2);
		Q qc3(3,0.1,0.2,0.2);
		Q qc4(4,0.1,0.2,0.2,0.2);
		Q qc5(5,0.1,0.2,0.2,0.2,0.2);
		Q qc6(6,0.1,0.2,0.2,0.2,0.2,0.2);
		Q qc7(7,0.1,0.2,0.2,0.2,0.2,0.2,0.2);
		Q qc8(8,0.1,0.2,0.2,0.2,0.2,0.2,0.2,0.2);
		Q qc9(9,0.1,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2);
		Q qc10(10,0.1,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2);

		BOOST_CHECK_THROW( Q qthrow(1,0.1,0.3), std::exception);
	}
}


