
/**
 * We need to specify the wrapper classes,
 */
#include <rw/math.hpp>

#ifndef RWLIBS_LUA_MATH_HPP
#define RWLIBS_LUA_MATH_HPP


const double Rad2Deg = rw::math::Rad2Deg;

namespace rwlibs {
namespace lua {

    //! @addtogroup lua
    // @{


    //! @brief wrapper class for rw::math::Q
    typedef rw::math::Q Q;

    //! @brief lua wrapper class for rw::math::Vector3D<double>
    typedef rw::math::Vector3D<double> Vector3D;

    //! @brief lua wrapper class for rw::math::Rotation3D<double>
    typedef rw::math::Rotation3D<double> Rotation3D;

    //! @brief lua wrapper class for rw::math::EAA<double>
    typedef rw::math::EAA<double> EAA;

    //! @brief lua wrapper class for rw::math::RPY<double>
    typedef rw::math::RPY<double> RPY;

    //! @brief lua wrapper class for rw::math::Quaternion<double>
    typedef rw::math::Quaternion<double> Quaternion;

    //! @brief lua wrapper class for rw::math::Transform3D<double>
    typedef rw::math::Transform3D<double> Transform3D;

    //! @brief lua wrapper class for rw::math::Pose6D<double>
    typedef rw::math::Pose6D<double> Pose6D;

    typedef rw::math::VelocityScrew6D<double> VelocityScrew6D;

    //const double Rad2Deg = rw::math::Rad2Deg;
    //const double Deg2Rad = rw::math::Deg2Rad;

    //! @brief calculates the inverse rotation of \b val
    Rotation3D inverse(const Rotation3D& val);

    //! @brief calculates the inverse transform of \b val
    Transform3D inverse(const Transform3D& val);

    // @}
}}

#endif

