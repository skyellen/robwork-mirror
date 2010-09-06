
/**
 * We need to specify the wrapper classes,
 */
#include <rw/math.hpp>

#ifndef RWLIBS_LUA_MATH_HPP
#define RWLIBS_LUA_MATH_HPP


namespace rwlibs {
namespace lua {

    //! @addtogroup lua
    // @{

    /**
     * @brief wrapper class for rw::math::Q
     */
    class Q : public rw::math::Q
    {
    public:
        // first we define functions that are native to Q
        Q(int n, double vals[]);
        Q(const rw::math::Q& q);

        // rw::amth::Q functions
        //size_t size() const;
        //double& operator[](unsigned int i) ;
        //double norm2();
        //double norm1();
        //double normInf();

        // specific lua functions
        // necesary copy constructor from rw type to lua type

        Q operator-() const;
        Q operator-(const Q& b);
        Q operator+(const Q& b);
        Q operator*(double s);
        Q operator/(double s);
        bool operator==(const Q& q);
        std::string __tostring() const;

    };

    /**
     * @brief lua wrapper class for rw::math::Vector3D<double>
     */
    class Vector3D: public rw::math::Vector3D<double>
    {
    public:
        Vector3D(const rw::math::Vector3D<double>& v);
        Vector3D(double x,double y, double z);

        //size_t size() const;
        //double norm2();
        //double norm1();
        //double normInf();
        //double& operator[](unsigned int i) ;

        // lua functions
        Vector3D operator*(double scale) const;
        Vector3D operator/(double s);
        Vector3D operator+(const Vector3D& other) const;
        Vector3D operator-(const Vector3D& other) const;
        bool operator==(const Vector3D& q);
        std::string __tostring() const;
    };

    class EAA;

    /**
     * @brief lua wrapper class for rw::math::Rotation3D<double>
     */
    class Rotation3D: public rw::math::Rotation3D<double>
    {
    public:
        // Lua methods:
        Rotation3D(double vals[9]);
        Rotation3D(const Vector3D& i, const Vector3D& j, const Vector3D& k);
        Rotation3D(const rw::math::Rotation3D<double>& R);

        //static const Rotation3D& identity();
        //static Rotation3D skew(const Vector3D& v);
        //bool equal(const Rotation3D& rot, double precision);
        //bool operator==(const Rotation3D &rhs) const;

        //double& operator[](unsigned int i) ;

        Rotation3D operator*(const Rotation3D& other) const;
        EAA operator*(const EAA& other) const;

        Vector3D operator*(const Vector3D& vec) const;
        Rotation3D inverse() const;

        std::string __tostring() const;
    };

    /**
     * @brief lua wrapper class for rw::math::EAA<double>
     */
    class EAA: public rw::math::EAA<double>
    {
    public:
        // Lua methods:
        EAA(const rw::math::EAA<double>& eaa);
        EAA(const rw::math::Rotation3D<double>& rot);

        EAA(double vals[3]);
        EAA(const Vector3D& v1, const Vector3D& v2);

        //const double& operator[](unsigned int i) const;

        //double angle() const;
        //Vector3D axis() const;

        //EAA operator*(const Rotation3D& other) const;

        //Rotation3D toRotation3D() const;

        bool operator==(const EAA &rhs) const;
        std::string __tostring() const;
    };

    /**
     * @brief lua wrapper class for rw::math::RPY<double>
     */
    class RPY: public rw::math::RPY<double>
    {
    public:
        // Lua methods:
        RPY(const rw::math::RPY<double>& eaa);
        RPY(const rw::math::Rotation3D<double>& rot);
        RPY(double vals[3]);

        //RPY operator*(const Rotation3D& other) const;
        //Rotation3D toRotation3D() const;

        bool operator==(const RPY &rhs) const;
        std::string __tostring() const;
    };

    /**
     * @brief lua wrapper class for rw::math::Quaternion<double>
     */
    class Quaternion: public rw::math::Quaternion<double>
    {
    public:
        // Lua methods:
        Quaternion(const rw::math::Quaternion<double>& eaa);
        Quaternion(const rw::math::Rotation3D<double>& rot);
        Quaternion(double vals[4]);

        Quaternion operator*(const Quaternion& other) const;
        Quaternion operator*(double s);

        //void normalize();

        //Rotation3D toRotation3D() const;
        //Quaternion slerp(const Quaternion& v, const double t) const;

/*        double getQx @ x() const;
        double getQy @ y() const;
        double getQz @ z() const;
        double getQw @ w() const;
*/
        //Quaternion operator*(double s, const Quaternion& v);

        bool operator==(const Quaternion &rhs) const;

        std::string __tostring() const;
    };

    //Quaternion operator*(double s, const Quaternion& v);

    /**
     * @brief lua wrapper class for rw::math::Transform3D<double>
     */
    class Transform3D: public rw::math::Transform3D<double>
    {
    public:
        Transform3D(const rw::math::Transform3D<double>& t3d);

        // Lua methods:
        Transform3D(
            const Vector3D& position,
            const Rotation3D& rotation);

        Transform3D operator*(const Transform3D& other) const;
        Vector3D operator*(const Vector3D& other) const;
        Transform3D inverse() const;

        //static Transform3D DH(double alpha, double a, double d, double theta);
        //static Transform3D craigDH(double alpha, double a, double d, double theta);

        //Vector3D P() const;
        //Rotation3D R() const;

        std::string __tostring() const;
    };


    /**
     * @brief lua wrapper class for rw::math::Pose6D<double>
     */
    class Pose6D: public rw::math::Pose6D<double>
    {
    public:
        Pose6D(const rw::math::Pose6D<double>& p6d);
        Pose6D(const Vector3D& position, const EAA& rotation);
        Pose6D(const rw::math::Transform3D<double>& t3d);

        //Transform3D toTransform3D();

        //Vector3D P() const;
        //EAA R() const;

        std::string __tostring() const;
    };


    //! @brief calculates the inverse rotation of \b val
    Rotation3D inverse(const Rotation3D& val);

    //! @brief calculates the inverse transform of \b val
    Transform3D inverse(const Transform3D& val);

    // @}
}}

#endif

