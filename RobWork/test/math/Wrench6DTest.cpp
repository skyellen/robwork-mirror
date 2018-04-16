#include "../TestSuiteConfig.hpp"

#include <rw/math/Wrench6D.hpp>
#include <rw/math/Vector3D.hpp>

BOOST_AUTO_TEST_CASE(Wrench6DTest_ADL) {
	rw::math::Vector3D<> vec1(1.4, 2.5, 3.6);
	rw::math::Vector3D<> vec2(4.7, 5.8, 6.9);
	rw::math::Wrench6D<> wrench(vec1, vec2);

	// Test Argument-Dependent Lookup (ADL)
	BOOST_CHECK(fabs(norm1(wrench) - (1.4 + 2.5 + 3.6 + 4.7 + 5.8 + 6.9))<1e-15);
	BOOST_CHECK(fabs(norm2(wrench) - sqrt(1.4 * 1.4 + 2.5 * 2.5 + 3.6 * 3.6 + 4.7 * 4.7 + 5.8 * 5.8 + 6.9 * 6.9))<1e-15);
	BOOST_CHECK(fabs(normInf(wrench) - 6.9)<1e-15);
}

using namespace rw::math;

BOOST_AUTO_TEST_CASE(Wrench6DTest) {
    BOOST_TEST_MESSAGE("- Testing Wrench6D");
  {
      /* Verify that a default wrench contains 0 */
      Wrench6D<> wrench;

      BOOST_CHECK_EQUAL(wrench(0), 0);
      BOOST_CHECK_EQUAL(wrench(1), 0);
      BOOST_CHECK_EQUAL(wrench(2), 0);
      BOOST_CHECK_EQUAL(wrench(3), 0);
      BOOST_CHECK_EQUAL(wrench(4), 0);
      BOOST_CHECK_EQUAL(wrench(5), 0);
  }
  {
      /* Verify that the wrench force and torque constructor appears to be working as intended
       * Verify that the force() and torque() members return the proper values
       */
      Vector3D<> vec1(1.4, 2.5, 3.6);
      Vector3D<> vec2(4.7, 5.8, 6.9);
      Wrench6D<> wrench(vec1, vec2);

      BOOST_CHECK(wrench.force() == vec1);
      BOOST_CHECK(wrench.torque() == vec2);

	  // Test unqualified lookup
	  BOOST_CHECK(fabs(norm1(wrench) - (1.4 + 2.5 + 3.6 + 4.7 + 5.8 + 6.9))<1e-15);
	  BOOST_CHECK(fabs(norm2(wrench) - sqrt(1.4 * 1.4 + 2.5 * 2.5 + 3.6 * 3.6 + 4.7 * 4.7 + 5.8 * 5.8 + 6.9 * 6.9))<1e-15);
	  BOOST_CHECK(fabs(normInf(wrench) - 6.9)<1e-15);

	  // Test qualified lookup
	  BOOST_CHECK(fabs(rw::math::norm1(wrench) - (1.4 + 2.5 + 3.6 + 4.7 + 5.8 + 6.9))<1e-15);
	  BOOST_CHECK(fabs(rw::math::norm2(wrench) - sqrt(1.4 * 1.4 + 2.5 * 2.5 + 3.6 * 3.6 + 4.7 * 4.7 + 5.8 * 5.8 + 6.9 * 6.9))<1e-15);
	  BOOST_CHECK(fabs(rw::math::normInf(wrench) - 6.9)<1e-15);
  }

  {
	  Vector3D<> vec1(0.1, 0.2, 0.3);
	  Vector3D<> vec2(0.4, 0.5, 0.6);
	  Wrench6D<> wrench(vec1, vec2);
	  {
		  Wrench6D<float> vsf = cast<float>(wrench);
		  for (size_t i = 0; i < 6; i++)
			  BOOST_CHECK((float)wrench(i) == vsf(i));
	  }
	  {
		  Wrench6D<float> vsf = rw::math::cast<float>(wrench); // qualified lookup
		  for (size_t i = 0; i < 6; i++)
			  BOOST_CHECK((float)wrench(i) == vsf(i));
	  }
  }

  {
      /* Verify that the setForce() and setTorque() members appear to be working as intended */
      Wrench6D<> wrench;
      Vector3D<> vec1(4.1, 5.2, 6.3);
      Vector3D<> vec2(7.4, 8.5, 9.6);

      wrench.setForce(vec1);
      wrench.setTorque(vec2);

      BOOST_CHECK(wrench.force() == vec1);
      BOOST_CHECK(wrench.torque() == vec2);

      const Eigen::VectorXd eigen = wrench.e();
      for (size_t i = 0; i<6; i++)
      	BOOST_CHECK(eigen(i) == wrench(i));
  }
}
