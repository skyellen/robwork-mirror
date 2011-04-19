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


#ifndef RW_MATH_MATH_HPP
#define RW_MATH_MATH_HPP

/**
 * @file rw/math/Math.hpp
 */

#include <cmath>
#include "LinearAlgebra.hpp"
#include "EAA.hpp"
#include "RPY.hpp"
#include "Quaternion.hpp"
#include "Q.hpp"
#include "Metric.hpp"

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
       @brief Utility functions for the rw::math module.
    */
    class Math
    {
    public:
        /**
         * @brief Quaternion to equivalent angle axis conversion.
         *
         * @param quat [in] the Quaternion object that is to be converted.
         *
         * @return a EAA object that represents the converted quaternion
         */
        template <class A>
        static EAA<A> quaternionToEAA(const Quaternion<A> &quat)
        {
            Quaternion<A> q = quat;

            // If w > 1 then acos and sqrt will produce errors. This can't
            // happen if the quaternion is normalised.
            if (q.getQw() > 1)
                q.normalize();

            const A angle = 2 * acos(q.getQw());

            // assuming quaternion normalised then w is less than 1, so term
            // always positive.
            const A s = std::sqrt(1 - q.getQw() * q.getQw());

            // Check to avoid a division by zero. s is always positive due to
            // sqrt.
            A kx, ky, kz;
            if (s < 1e-3) {
                // If s close to zero then direction of axis not important. If
                // it is important that the axis is of the right length then
                // replace with x = 1 and y = z = 0.
                kx = q.getQx();
                ky = q.getQy();
                kz = q.getQz();
            } else {
                kx = q.getQx() / s; // normalise axis
                ky = q.getQy() / s;
                kz = q.getQz() / s;
            }

            return EAA<A>(kx * angle, ky * angle, kz * angle);
        }

        /**
         * @brief Equivalent angle axis to quaternion conversion.
         *
         * @param eaa [in] the EAA object that is to be converted
         *
         * @return a Quaternion object that represents the converted EAA
         */
        template <class A>
        static Quaternion<A> eaaToQuaternion(const EAA<A> &eaa)
        {
            const Vector3D<A> v = eaa.axis();
            const A a2 = eaa.angle() / 2;
            const A s = sin(a2);
            const A x = v[0] * s;
            const A y = v[1] * s;
            const A z = v[2] * s;
            const A w = cos(a2);
            return Quaternion<A>(x, y, z, w);
        }

        /**
         * @brief this function converts a EAA object to a Quaternion
         *
         * @param roll [in] rotation around z
         * @param pitch [in] rotation around y
         * @param yaw [in] rotation around x
         *
         * @return a Quaternion object that represents the converted EAA
         */
        template <class A>
        static Rotation3D<A> zyxToRotation3D(A roll, A pitch, A yaw)
        {
            return RPY<A>(roll, pitch, yaw).toRotation3D();
        }

        /**
         * \brief Constructs a 3x3 skew-symmetric matrix \f$ S\in so(3)\f$
         * \param s [in] the \f$ s_x \f$, \f$ s_y \f$ and \f$ s_z \f$ of the matrix
         * \return The 3x3 skew-symmetric matrix \f$S\f$
         *
         * \f$
         * S =
         * \left[
         * \begin{array}{ccc}
         *    0 & -s_z &  s_y\\
         *  s_z &    0 & -s_x\\
         * -s_y &  s_x &    0
         * \end{array}
         * \right]
         * \f$
         */
        template<class R>
        static inline boost::numeric::ublas::bounded_matrix<R, 3, 3> skew(const boost::numeric::ublas::bounded_vector<R, 3>& s)
        {
            boost::numeric::ublas::bounded_matrix<R, 3, 3> S;
            S(0,0) =   0.0; S(0,1) = -s[2]; S(0,2) =  s[1];
            S(1,0) =  s[2]; S(1,1) =   0.0; S(1,2) = -s[0];
            S(2,0) = -s[1]; S(2,1) =  s[0]; S(2,2) =   0.0;

            assert(LinearAlgebra::isSkewSymmetric(S));
            return S;
        }

        /**
         * @brief clamp val to either min or max
         *
         * @param val [in] the value that is to be clamped
         * @param min [in] the minimum allowed value
         * @param max [in] the maximum allowed value
         * @return the clamped value of val
         */
        static double clamp(double val, double min, double max)
        {
            if (val < min)
                return min;
            if(val > max)
                return max;
            return val;
        }


        /**
         * @brief Clamps values of \b q with \b min and \b max
         *
         * @param q [in] Values to clamp
         * @param min [min] The minimum value
         * @param max [min] The maximum value
         * @return The clamped values
         */
        static rw::math::Q clampQ(
            const rw::math::Q& q,
            const rw::math::Q& min,
            const rw::math::Q& max)
        {
            assert(q.size() == min.size());
            assert(q.size() == max.size());

            Q qres(q.size());
            for (size_t i = 0; i<q.size(); i++)
                qres(i) = clamp(q(i), min(i), max(i));

            return qres;
        }

        /**
         * @brief Clamps values of \b q with \b min and \b max
         *
         * @param q [in] Values to clamp
         * @param min [min] The minimum value
         * @param max [min] The maximum value
         * @return The clamped values
         */
        static rw::math::Vector3D<> clamp(
            const rw::math::Vector3D<>& q,
            const rw::math::Vector3D<>& min,
            const rw::math::Vector3D<>& max)
        {
            assert(q.size() == min.size());
            assert(q.size() == max.size());

            Vector3D<> qres;
            for (size_t i = 0; i<3; i++)
                qres(i) = clamp(q(i), min(i), max(i));

            return qres;
        }


        // Global random number generation.

        /**
         * @brief A random double in the range [0, 1[.
         *
         * @note Uses boost::random
         */
        static double ran();

        /**
         * @brief Seeds the random number generator.
         *
         * @note Uses boost::random
         */
        static void seed(unsigned seed);

        /**
         * @brief A random double in the range [from, to[.
         *
         * @note Uses boost::random
         */
        static double ran(double from, double to);

        /**
         * @brief A random integer in the range [from, to[.
         *
         * @note Uses boost::random
         */
        static int ranI(int from, int to);

        /**
         * @brief Returns a random sample around \b mean with standard deviation \b sigma
         *
         * @note Uses boost::random
         *
         * @param mean [in] Means value
         * @param sigma [in] Standard deviation
         * @return Random sample
         */
        static double ranNormalDist(double mean, double sigma);

        /**
         * @brief Returns a random Q between with values in the range [from, to[.
         *
         * @note Uses boost::random
         *
         * @param from [in] The lower bound
         * @param to [in] The upper bound
         * @return Random Q
         */
        static rw::math::Q ranQ(const rw::math::Q& from, const rw::math::Q& to);

		/**
		 * @brief Returns a uniformly distributed random orientation
		 *
		 * @return Random orientation
		 */
		template<class T>
		static rw::math::Rotation3D<T> ranRotation3D() {
			Vector3D<T> axis(ran(-1,1),ran(-1,1),ran(-1,1));			
			EAA<> eaa(axis.normalize(), ran(0,Pi));
			return eaa.toRotation3D();
		}

        /**
         * @brief Rounds off to nearest integer
         *
         * With some compilers \b round can be found in math.h. This however does not
         * appear to be ansi C/C++ standard
         *
         * @param d [in] number to round
         * @return d rounded to nearest integer.
         */
        static double round(double d) { return floor(d + 0.5); }

        /**
          @brief The square of \b d

          @param d [in] Number to square
          @return d * d
        */
        template <class T>
        static inline T sqr(const T& d) { return d * d; }

        /**
           @brief The squares of the elements of \b q.
        */
        static Q sqr(const Q& q);

        /**
           @brief The square roots of the elements of \b q.
        */
        static Q sqrt(const Q& q);

        /**
         * @brief Returns vector with the absolute values
         *
         * Given a vector \f$v=[v_1,v_2,\ldots,v_n]\f$ then Abs(v) is defined as
         * \f$Abs(v)=[abs(v_1),abs(v_i),\ldots,abs(v_n)] \f$
         *
         * @param v [in] the vector \f$v\f$
         * @return the vector \f$Abs(v)\f$
         */
        template<class T>
        static boost::numeric::ublas::vector<T> abs(const boost::numeric::ublas::vector<T>& v)
        {
            boost::numeric::ublas::vector<T> result(v.size());
            for (size_t i = 0; i < v.size(); i++)
                result[i] = std::fabs(v[i]);

            return result;
        }

        /**
          * @brief Returns vector with the absolute values
          *
          * Given a vector \f$v=[v_1,v_2,\ldots,v_n]\f$ then Abs(v) is defined as
          * \f$Abs(v)=[abs(v_1),abs(v_i),\ldots,abs(v_n)] \f$
          *
          * @param v [in] the vector \f$v\f$
          * @return the vector \f$Abs(v)\f$
          */
        static Q abs(const Q& v)
        {
            Q result(v.size());
            for (size_t i = 0; i<v.size(); i++)
                result[i] = std::fabs(v[i]);
            return result;
        }

        /**
         * @brief Returns the smallest element of v
         *
         * If the vector has zero length, the method returns 0
         *
         * @param v [in] the vector v
         * @return the smallest element
         */
        template<class T>
        static T min(const boost::numeric::ublas::vector<T>& v)
        {
            if (v.size() == 0)
                return 0;

            T minval = v(0);
            for (size_t i = 1; i<v.size(); i++)
                if (v(i)<minval)
                    minval = v(i);
            return minval;
        }

        /**
         * @brief Returns the largest element of v
         *
         * If the vector has zero length, the method returns 0
         *
         * @param v [in] the vector v
         * @return the largest element
         */
        template<class T>
        static T max(const boost::numeric::ublas::vector<T>& v)
        {
            if (v.size() == 0)
                return 0;

            T maxval = v(0);
            for (size_t i = 1; i<v.size(); i++)
                if (v(i)>maxval)
                    maxval = v(i);
            return maxval;
        }

        /**
         * @brief Returns the smallest element of v
         *
         * If the vector has zero length, the method returns 0
         *
         * @param v [in] the vector v
         * @return the smallest element
         */
        static double min(const Q& v) { return min(v.m()); }

        /**
         * @brief Returns the largest element of v
         *
         * If the vector has zero length, the method returns 0
         *
         * @param v [in] the vector v
         * @return the largest element
         */
        static double max(const Q& v) { return max(v.m()); }

        /**
         * @brief Returns vector with the elementwise smallest elements of \b a and \b b
         *
         * @param a [in] the vector \b a
         * @param b [in] the vector \b b
         * @return Q with smallest elements
         */
        template<class T>
        static T min(const T& a, const T& b)
        {
            assert(a.size() == b.size());

            T result(a.size());
            for (size_t i = 1; i<a.size(); i++)
                result[i] = std::min(a[i], b[i]);
            return result;
        }

        /**
         * @brief Returns vector with the elementwise largest elements of \b a and \b b
         *
         * @param a [in] the vector \b a
         * @param b [in] the vector \b b
         * @return Q with largest elements
         */
        template<class T>
        static T max(const T& a, const T& b)
        {
            assert(a.size() == b.size());

            T result(a.size());
            for (size_t i = 1; i<a.size(); i++)
                result(i) = std::max(a[i], b[i]);
            return result;
        }

        /**
         * @brief Returns vector with the absolute values
         *
         * Given a vector \f$v=[v_1,v_2,\ldots,v_n]\f$ then Abs(v) is defined as
         * \f$Abs(v)=[abs(v_1),abs(v_i),\ldots,abs(v_n)] \f$
         *
         * @param v [in] the vector \f$v\f$
         * @return the vector \f$Abs(v)\f$
         */
        template<class T>
        static Vector3D<T> abs(const Vector3D<T>& v)
        {
            Vector3D<T> result;
            for (size_t i = 0; i<3; i++)
                result[i] = std::fabs(v[i]);
            return result;
        }

        /**
         * @brief Returns the smallest element of v
         *
         * @param v [in] the vector v
         * @return the smallest element
         */
        template<class T>
        static T min(const Vector3D<T>& v)
        {
            T minval = v(0);
            for (size_t i = 1; i<3; i++)
                if (v(i)<minval)
                    minval = v(i);
            return minval;
        }

        /**
         * @brief Returns the largest element of v
         *
         * @param v [in] the vector v
         * @return the largest element
         */
        template<class T>
        static T max(const Vector3D<T>& v)
        {
            T maxval = v(0);
            for (size_t i = 1; i<3; i++)
                if (v(i)>maxval)
                    maxval = v(i);
            return maxval;
        }

        /**
         * @brief Returns vector with the elementwise smallest elements of \b a and \b b
         *
         * @param a [in] the vector \b a
         * @param b [in] the vector \b b
         * @return Vector with smallest elements
         */
        template<class T>
        static Vector3D<T> min(const Vector3D<T>& a, const Vector3D<T>& b)
        {
            Vector3D<T> result;
            for (size_t i = 1; i<3; i++)
                result(i) = std::min(a[i], b[i]);
            return result;
        }

        /**
         * @brief Returns vector with the elementwise largest elements of \b a and \b b
         *
         * @param a [in] the vector \b a
         * @param b [in] the vector \b b
         * @return Vector with largest elements
         */
        template<class T>
        static Vector3D<T> max(const Vector3D<T>& a, const Vector3D<T>& b)
        {
            Vector3D<T> result;
            for (size_t i = 1; i<3; i++)
                result(i) = std::max(a[i], b[i]);
            return result;
        }

        /**
         * @brief Returns the sign of s
         *
         * If s < 0 it return 0. If s >= 0 then 1 is returned.
         *
         * @param s [in] The value for which to return the sign
         * @return The sign
         */
        static double sign(double s) { return s >= 0 ? 1 : -1; }

        /**
         * @brief Returns the sign of each element
         *
         * For each element either -1 or 1 is returned depending on the sign. If \b q(i) equals 0
         * the method returns 1
         *
         * @param q [in] Vector for which to get the signs
         * @return Vector of sign values
         */
        static Q sign(const Q& q)
        {
            Q res(q.size());
            for (size_t i = 0; i < q.size(); i++)
                res(i) = sign(q(i));
            return res;
        }

        /**
           @brief Exact implementation of ceil(log_2(n)) for n > 0.
        */
        static int ceilLog2(int n);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
