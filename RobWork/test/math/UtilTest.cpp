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
#include <rw/math/EAA.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/math/Wrench6D.hpp>


using namespace rw::math;

BOOST_AUTO_TEST_CASE(UtilTest)
{
    BOOST_TEST_MESSAGE("- Testing Utils");
    Quaternion<> q(0.1, 0.2, 0.3, 0.4);
    q.normalize();
    const EAA<> eaa(1,2,3);

    const EAA<> eaa_q = Math::quaternionToEAA(q);
    const Quaternion<> q_eaa = Math::eaaToQuaternion(eaa_q);

    BOOST_CHECK_CLOSE(q(0),q_eaa(0), 1e-12);
    BOOST_CHECK_CLOSE(q(1),q_eaa(1), 1e-12);
    BOOST_CHECK_CLOSE(q(2),q_eaa(2), 1e-12);
    BOOST_CHECK_CLOSE(q(3),q_eaa(3), 1e-12);
}

BOOST_AUTO_TEST_CASE(testCeilLog2)
{
    BOOST_CHECK_EQUAL(Math::ceilLog2(1), 0);
    BOOST_CHECK_EQUAL(Math::ceilLog2(2), 1);
    BOOST_CHECK_EQUAL(Math::ceilLog2(3), 2);
    BOOST_CHECK_EQUAL(Math::ceilLog2(4), 2);
    BOOST_CHECK_EQUAL(Math::ceilLog2(5), 3);
    BOOST_CHECK_EQUAL(Math::ceilLog2(8), 3);
    BOOST_CHECK_EQUAL(Math::ceilLog2(9), 4);
}

BOOST_AUTO_TEST_CASE(testVector3D_norm){
    Vector3D<> v1(1.0, 2.0, 2.0);

    BOOST_CHECK_EQUAL( MetricUtil::norm1(v1), 5.0);
    BOOST_CHECK_EQUAL( MetricUtil::norm2(v1), 3.0);
    BOOST_CHECK_EQUAL( MetricUtil::normInf(v1), 2.0);
}

BOOST_AUTO_TEST_CASE(testVector3D_cross){
    Vector3D<> v1(1.0, 2.0, 3.0);
    Vector3D<> v2(v1);

    BOOST_CHECK_EQUAL( MetricUtil::normInf(cross(v1, v2)), 0);
}

BOOST_AUTO_TEST_CASE(testTransform3DAngleMetric){
    Vector3D<> v1(1.0, 2.0, 2.0);
	Transform3D<> t1(Vector3D<>(1, 0, 0), Rotation3D<>::identity());
    Transform3D<> t2(Vector3D<>(1, 0, 0), RPY<>(1.4, 0, 0).toRotation3D());
	BOOST_CHECK_CLOSE(Transform3DAngleMetric<double>(1.0, 0.0).distance(t1, t2), 0, 1e-15);
	BOOST_CHECK_CLOSE(Transform3DAngleMetric<double>(0.0, 1.0).distance(t1, t2), 1.4, 1e-15);
}


BOOST_AUTO_TEST_CASE(testRotation3D_inverse){
    Vector3D<std::string> i("i1", "i2", "i3");
    Vector3D<std::string> j("j1", "j2", "j3");
    Vector3D<std::string> k("k1", "k2", "k3");
    Rotation3D<std::string> rot(i, j, k);
    BOOST_CHECK(rot(1,0) == "i2");
    BOOST_CHECK(inverse(rot)(1,0) == "j1");

    Rotation3D<std::string> rotInv(inverse(rot));
    for(int r=0;r<3;r++){
        for(int c=0;c<3;c++){
            BOOST_CHECK(rot(r,c) == rotInv(c,r));
        }
    }
}

BOOST_AUTO_TEST_CASE(testWrench6D){
	rw::math::Wrench6D<> wrench(1,2,3,4,5,6);
	BOOST_CHECK(wrench.force()[0] == 1 );
	BOOST_CHECK(wrench.force()[1] == 2 );
	BOOST_CHECK(wrench.force()[2] == 3 );
	BOOST_CHECK(wrench.torque()[0] == 4 );
	BOOST_CHECK(wrench.torque()[1] == 5 );
	BOOST_CHECK(wrench.torque()[2] == 6 );

}

BOOST_AUTO_TEST_CASE(RandomSeedTest)
{
	// The boost random generator uses Mersenne twister pseudo-random generator
	// The random values should therefore be deterministic for a given seed
    BOOST_TEST_MESSAGE("- Testing Random Seeds");
    // Check that the correct values are obtained, and that seeding with the same
    // value gives same result.
    for (unsigned int i = 0; i < 2; i++) {
    	Math::seed(10);
    	const double ran = Math::ran();
    	const double ranFromTo = Math::ran(-0.2,0.4);
    	const int ranI = Math::ranI(-200,7000);
    	const double ranNormalDist = Math::ranNormalDist(1., 2.2);
    	const Q ranQ = Math::ranQ(Q(2,0.1,0.5),Q(2,0.5,1.0));
    	const Q ranQpair = Math::ranQ(std::make_pair<Q,Q>(Q(2,0.1,0.5),Q(2,0.5,1.0)));
    	const Q ranDir = Math::ranDir(4, 2.1);
    	const Q ranWeightedDir = Math::ranWeightedDir(3, Q(3,1,2,3), 0.1);
        // Comparing with a tolerance of 1.0e-06%. This value was found by looking at the highest difference in % (i.e. 2.01204e-07% in my/mband's case) that was reported when running these tests with a too low tolerance (e.g. 1.0e-09).
   	BOOST_CHECK_CLOSE(ran,0.77132064313627779,1.0e-06);
    	BOOST_CHECK_CLOSE(ranFromTo,-0.020743304910138233,1.0e-06);
    	BOOST_CHECK_EQUAL(ranI,-51);
    	BOOST_CHECK_CLOSE(ranNormalDist,-2.1159354853352048,1.0e-06);
    	BOOST_CHECK_CLOSE(ranQ[0],0.27720597842708228,1.0e-06);
    	BOOST_CHECK_CLOSE(ranQ[1],0.87440194131340832,1.0e-06);
    	BOOST_CHECK_CLOSE(ranQpair[0],0.43276454471051695,1.0e-06);
    	BOOST_CHECK_CLOSE(ranQpair[1],0.74925350688863546,1.0e-06);
    	BOOST_CHECK_CLOSE(ranDir[0],0.10420574144754288,1.0e-06);
    	BOOST_CHECK_CLOSE(ranDir[1],-1.3371934765112323,1.0e-06);
    	BOOST_CHECK_CLOSE(ranDir[2],-0.77189916796530089,1.0e-06);
    	BOOST_CHECK_CLOSE(ranDir[3],1.4195867160267628,1.0e-06);
    	BOOST_CHECK_CLOSE(ranWeightedDir[0],0.0021004772778212945,1.0e-06);
    	BOOST_CHECK_CLOSE(ranWeightedDir[1],-0.0086033368952058847,1.0e-06);
    	BOOST_CHECK_CLOSE(ranWeightedDir[2],-0.03282871096443158,1.0e-06);
    }
    // Check another seed
    for (unsigned int i = 0; i < 2; i++) {
    	Math::seed(123456);
    	const double ran = Math::ran();
    	const double ranFromTo = Math::ran(0,1);
    	const int ranI = Math::ranI(-200,7000);
    	const double ranNormalDist = Math::ranNormalDist(1., 2.2);
    	const Q ranQ = Math::ranQ(Q(2,0.1,0.5),Q(2,0.5,1.0));
    	const Q ranQpair = Math::ranQ(std::make_pair<Q,Q>(Q(2,0.1,0.5),Q(2,0.5,1.0)));
    	const Q ranDir = Math::ranDir(4, 2.1);
    	const Q ranWeightedDir = Math::ranWeightedDir(3, Q(3,1,2,3), 0.1);
    	BOOST_CHECK_CLOSE(ran,0.12696982943452895,1.0e-06);
    	BOOST_CHECK_CLOSE(ranFromTo,0.5149132558144629,1.0e-06);
    	BOOST_CHECK_EQUAL(ranI,6760);
    	BOOST_CHECK_CLOSE(ranNormalDist,2.3707402760475729,1.0e-06);
    	BOOST_CHECK_CLOSE(ranQ[0],0.38232804937288167,1.0e-06);
    	BOOST_CHECK_CLOSE(ranQ[1],0.94861826102714986,1.0e-06);
    	BOOST_CHECK_CLOSE(ranQpair[0],0.41153188152238729,1.0e-06);
    	BOOST_CHECK_CLOSE(ranQpair[1],0.68837485776748508,1.0e-06);
    	BOOST_CHECK_CLOSE(ranDir[0],-0.76867727692667076,1.0e-06);
    	BOOST_CHECK_CLOSE(ranDir[1],1.3635143841520583,1.0e-06);
    	BOOST_CHECK_CLOSE(ranDir[2],-0.62472325234244208,1.0e-06);
    	BOOST_CHECK_CLOSE(ranDir[3],-1.2528705544188166,1.0e-06);
    	BOOST_CHECK_CLOSE(ranWeightedDir[0],-0.014881418304403815,1.0e-06);
    	BOOST_CHECK_CLOSE(ranWeightedDir[1],-0.019235042703143274,1.0e-06);
    	BOOST_CHECK_CLOSE(ranWeightedDir[2],0.030365543188295908,1.0e-06);
    }
}

BOOST_AUTO_TEST_CASE(ranRotation3D) {
    const int times = 10;
    Math::seed(423); // seed the RNG - The seed was arbitrarily chosen
    for (int i = 0; i < times; ++i) {
        Rotation3D<> rot = Math::ranRotation3D<double>();
        BOOST_CHECK(rot.isProperRotation(1.0e-15));
        // Use the following to have the value of the determinant printed, if the precision of the comparison within the implemented functions used by isProperRotation(), has to be updated.
        // BOOST_CHECK_EQUAL(rot.e().determinant(), 1.0);
    }

    for (int i = 0; i < times; ++i) {
        Rotation3D<float> rot = Math::ranRotation3D<float>();
        BOOST_CHECK(rot.isProperRotation(1.0e-06));
        // BOOST_CHECK_EQUAL(rot.e().determinant(), 1.0);
    }
}

BOOST_AUTO_TEST_CASE(ranTransform3D) {
    /*
     * Not testing the translation part as that should just be some random doubles or floats
     */
    const int times = 10;
    Math::seed(829); // seed the RNG - The seed was arbitrarily chosen
    for (int i = 0; i < times; ++i) {
        Transform3D<> transform = Math::ranTransform3D<double>();
        BOOST_CHECK(transform.R().isProperRotation(1.0e-15));
    }

    for (int i = 0; i < times; ++i) {
        Transform3D<float> transform = Math::ranTransform3D<float>();
        BOOST_CHECK(transform.R().isProperRotation(1.0e-06));
    }
}

BOOST_AUTO_TEST_CASE(MatrixVectorConvertionTest) {
    BOOST_TEST_MESSAGE("- Testing matrix to/from vector utiliy functions");
    // Test Math convenience functions for serialization and deserialization of matrix types.
    Math::seed(829);
    const Transform3D<> T = Math::ranTransform3D<double>();
    std::vector<double> Tserialized = Math::toStdVector(T, 3, 4);
    BOOST_CHECK(Tserialized.size() == 12);
    Transform3D<> Trestored;
    Math::fromStdVectorToMat(Tserialized, Trestored, 3, 4);
    BOOST_CHECK(T.equal(Trestored));
}

BOOST_AUTO_TEST_CASE(NanFunctionality) {
    double aNumber = 0.3;
    double aNaN = Math::NaN();
    BOOST_CHECK(!Math::isNaN(aNumber));
    BOOST_CHECK(Math::isNaN(aNaN));
}
