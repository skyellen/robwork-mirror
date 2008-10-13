#include <rw/math/EAA.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Math.hpp>

#include <boost/test/unit_test.hpp>

using namespace rw::math;

void UtilTest()
{
    BOOST_MESSAGE("- Testing Utils");
    Quaternion<> q(0.1, 0.2, 0.3, 0.4);
    q.normalize();
    const EAA<> eaa(1,2,3);

    const EAA<> eaa_q = Math::quaternionToEAA(q);
    const Quaternion<> q_eaa = Math::eaaToQuaternion(eaa_q);

    BOOST_CHECK(fabs(q(0) - q_eaa(0)) < 1e-12);
    BOOST_CHECK(fabs(q(1) - q_eaa(1)) < 1e-12);
    BOOST_CHECK(fabs(q(2) - q_eaa(2)) < 1e-12);
    BOOST_CHECK(fabs(q(3) - q_eaa(3)) < 1e-12);
}
