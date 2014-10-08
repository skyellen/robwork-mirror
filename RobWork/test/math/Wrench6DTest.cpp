#include "../TestSuiteConfig.hpp"

#include <rw/math/Wrench6D.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rw::math;

BOOST_AUTO_TEST_CASE(Wrench6DTest) {
    BOOST_MESSAGE("- Testing Wrench6D");
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
  }
}
