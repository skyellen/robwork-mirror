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


#ifndef RW_MATH_QUATERNION_HPP
#define RW_MATH_QUATERNION_HPP

/**
 * @file Quaternion.hpp
 */

#include "Rotation3D.hpp"
#include "Rotation3DVector.hpp"

#include <ostream>
#include <boost/math/quaternion.hpp>

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A Quaternion @f$ \mathbf{q}\in \mathbb{R}^4 @f$ a complex
     * number used to describe rotations in 3-dimensional space.
     * @f$ q_w+{\bf i}\ q_x+ {\bf j} q_y+ {\bf k}\ q_z @f$
     *
     * Quaternions can be added and multiplied in a similar way as usual
     * algebraic numbers. Though there are differences. Quaternion
     * multiplication is not commutative which means
     * @f$Q\cdot P \neq P\cdot Q @f$
     */
    template<class T = double>
    class Quaternion :
        public boost::math::quaternion<T>,
        public Rotation3DVector<T>
    {
    private:
        typedef boost::math::quaternion<T> Base_quaternion;

    public:
        Quaternion() : Base_quaternion(0, 0, 0, 1) {}

        /**
         * @brief Creates a Quaternion
         * @param qx [in] @f$ q_x @f$
         * @param qy [in] @f$ q_y @f$
         * @param qz [in] @f$ q_z @f$
         * @param qw  [in] @f$ q_w @f$
         */
        Quaternion(T qx, T qy, T qz, T qw) : Base_quaternion(qx, qy, qz, qw) {}

        /**
         * @brief Creates a Quaternion from another Quaternion
         * @param quat [in] Quaternion
         */
        Quaternion(const Quaternion<T>& quat) : Base_quaternion(quat) {}

        /**
         * @brief Extracts a Quaternion from Rotation matrix using
         * setRotation(const Rotation3D<R>& rot)
         * @param rot [in] A 3x3 rotation matrix @f$ \mathbf{rot} @f$
         *
         */
        Quaternion(const Rotation3D<T>& rot) : Base_quaternion()
        {
            setRotation(rot);
        }

        /**
         * @brief Creates a Quaternion from a boost quaternion
         * @param r [in] a boost quaternion
         */
        Quaternion(const Base_quaternion& r) : Base_quaternion(r) {}

        /**
         * @brief copy a boost quaternion to this Quaternion
         * @param r [in] - boost quaternion
         */
        inline void operator=(const Base_quaternion& r)
        {
            Base_quaternion::operator=(r);
        }

        /**
         * @brief get method for the x component
         * @return the x component of the quaternion
         */
        inline T getQx() const { return Base_quaternion::R_component_1(); }

        /**
         * @brief get method for the y component
         * @return the y component of the quaternion
         */
        inline T getQy() const { return Base_quaternion::R_component_2(); }

        /**
         * @brief get method for the z component
         * @return the z component of the quaternion
         */
        inline T getQz() const { return Base_quaternion::R_component_3(); }

        /**
         * @brief get method for the w component
         * @return the w component of the quaternion
         */
        inline T getQw() const { return Base_quaternion::R_component_4(); }

        /**
         * @brief get length of quaternion
         * @f$ \sqrt{q_x^2+q_y^2+q_z^2+q_w^2} @f$
         * @return the length og this quaternion
         */
        inline T getLength() const
        {
            return static_cast<T>(sqrt(getLengthSquared()));
        }

        /**
         * @brief get squared length of quaternion
         * @f$ q_x^2+q_y^2+q_z^2+q_w^2 @f$
         * @return the length og this quaternion
         */
        inline T getLengthSquared() const
        {
            return static_cast<T>(
                this->a * this->a +
                this->b * this->b +
                this->c * this->c +
                this->d * this->d);
        }

        /**
         * @brief normalizes this quaternion so that
         * @f$ normalze(Q)=\frac{Q}{\sqrt{q_x^2+q_y^2+q_z^2+q_w^2}} @f$
         */
        inline void normalize()
        {
            const T mag = getLengthSquared();
            const T n = ((T)1.0)/sqrt(mag);

            this->a *= n;
            this->b *= n;
            this->c *= n;
            this->d *= n;
        };

        /**
         * @brief Calculates the @f$ 3\times 3 @f$ Rotation matrix
         *
         * @return A 3x3 rotation matrix @f$ \mathbf{rot} @f$
         * @f$
         * \mathbf{rot} =
         *  \left[
         *   \begin{array}{ccc}
         *      1-2(q_y^2-q_z^2) & 2(q_x\ q_y+q_z\ q_w)& 2(q_x\ q_z-q_y\ q_w) \\
         *      2(q_x\ q_y-q_z\ q_w) & 1-2(q_x^2-q_z^2) & 2(q_y\ q_z+q_x\ q_w)\\
         *      2(q_x\ q_z+q_y\ q_w) & 2(q_y\ q_z-q_x\ q_z) & 1-2(q_x^2-q_y^2)
         *    \end{array}
         *  \right]
         * @f$
         *
         */
        inline const Rotation3D<T> toRotation3D() const
        {
            T qx = this->a;
            T qy = this->b;
            T qz = this->c;
            T qw = this->d;

            return
                Rotation3D<T>(
                    1-2*qy*qy-2*qz*qz, 2*(qx*qy-qz*qw)    , 2*(qx*qz+qy*qw),
                    2*(qx*qy+qz*qw)  ,  1-2*qx*qx-2*qz*qz , 2*(qy*qz-qx*qw),
                    2*(qx*qz-qy*qw)  ,  2*(qy*qz+qx*qw)   , 1-2*qx*qx-2*qy*qy);
        }

        /**
         * @brief Returns reference to Quaternion element
         * @param i [in] index in the quaternion \f$i\in \{0,1,2,3\} \f$
         * @return const reference to element
         */
        inline const T& operator()(size_t i) const
        {
            switch(i){
            case 0: return this->a;
            case 1: return this->b;
            case 2: return this->c;
            case 3: return this->d;
            default:
                assert(0);
                return this->a;
            }
        }

        /**
         * @brief Returns reference to Quaternion element
         * @param i [in] index in the quaternion \f$i\in \{0,1,2,3\} \f$
         * @return reference to element
         */
        inline T& operator()(size_t i)
        {
            switch(i){
            case 0: return this->a;
            case 1: return this->b;
            case 2: return this->c;
            case 3: return this->d;
            default:
                assert(0);
                return this->a;
            }
        }

        /**
         * @brief Calculates a slerp interpolation between \b this and \b v.
         *
         * The slerp interpolation ensures a constant velocity across the interpolation.
         * For \f$t=0\f$ the result is \b this and for \f$t=1\f$ it is \b v.
         *
         * @note Algorithm and implementation is thanks to euclideanspace.com
         */
        inline const Quaternion<T> slerp(const Quaternion<T>& v, const T t) const
        {
            const T qax = this->a;
            const T qay = this->b;
            const T qaz = this->c;
            const T qaw = this->d;
            const T qbx = v(0);
            const T qby = v(1);
            const T qbz = v(2);
            const T qbw = v(3);

            // Calculate angle between them.
            const T cosHalfTheta = qaw * qbw + qax * qbx + qay * qby + qaz * qbz;

            // if qa=qb or qa=-qb then theta = 0 and we can return qa
            if (fabs(cosHalfTheta) >= 1.0) {
                return Quaternion<T>(qax, qay, qaz, qaw);
                //qmw = qaw;qm.x = qa.x;qm.y = qa.y;qm.z = qa.z;
            }

            // Calculate temporary values.
            const T halfTheta = acos(cosHalfTheta);
            const T sinHalfTheta = (T)sqrt(1.0 - cosHalfTheta * cosHalfTheta);

            // if theta = 180 degrees then result is not fully defined
            // we could rotate around any axis normal to qa or qb
            if (fabs(sinHalfTheta) < static_cast<T>(0.001)){
                const T t05 = static_cast<T>(0.5);
                T qmw = (qaw * t05 + qbw * t05);
                T qmx = (qax * t05 + qbx * t05);
                T qmy = (qay * t05 + qby * t05);
                T qmz = (qaz * t05 + qbz * t05);
                return Quaternion<T>(qmx,qmy,qmz,qmw);
            }

            const T ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
            const T ratioB = sin(t * halfTheta) / sinHalfTheta;

            // calculate Quaternion.
            const T qmw = (qaw * ratioA + qbw * ratioB);
            const T qmx = (qax * ratioA + qbx * ratioB);
            const T qmy = (qay * ratioA + qby * ratioB);
            const T qmz = (qaz * ratioA + qbz * ratioB);
            return Quaternion<T>(qmx,qmy,qmz,qmw);
        }

        /**
           @brief Scalar multiplication.
         */
        inline const Quaternion<T> operator*( T s) const
        {
            Quaternion<T> q( *this );
            q *= s;
            return q;
        }

        /**
           @brief Scalar multiplication.
         */
        inline friend const Quaternion<T> operator*(T s, const Quaternion<T>& v)
        {
            Quaternion<T> q(v);
            q *= s;
            return q;
        }


        /** @brief Converts a Rotation3D to a Quaternion and saves the Quaternion
         * in this.
         *
         * @param rot [in] A 3x3 rotation matrix @f$ \mathbf{R} @f$
         *
         * @f$
         * \begin{array}{c}
         * q_x\\ q_y\\ q_z\\ q_w
         * \end{array}
         * =
         *  \left[
         *   \begin{array}{c}
         *      \\
         *      \\
         *
         *    \end{array}
         *  \right]
         * @f$
         *
         */
        template <class R>
        void setRotation(const Rotation3D<R>& rot)
        {
/*
            #define USE_OLD_CONVERSION
#ifdef USE_OLD_CONVERSION
            const T tr = static_cast<T>(rot(0, 0) + rot(1, 1) + rot(2, 2) + 1);

            if (tr > 1e-5) {
                const T s = static_cast<T>(0.5) / static_cast<T>(sqrt(tr));
                this->d = static_cast<T>(0.25) / s;
                this->a = static_cast<T>(rot(2, 1) - rot(1, 2)) * s;
                this->b = static_cast<T>(rot(0, 2) - rot(2, 0)) * s;
                this->c = static_cast<T>(rot(1, 0) - rot(0, 1)) * s;
            } else {
                if (rot(0, 0) > rot(1, 1) && rot(0, 0) > rot(2, 2)) {
                    const T sa = static_cast<T>(sqrt(rot(0, 0) - rot(1, 1) - rot(2, 2) + 1.0));
                    this->a = static_cast<T>(0.5)  *  sa;

                    // s == 1 / (2.0  *  sa) == 0.25 / (0.5  *  sa)
                    const T s = static_cast<T>(0.25) / this->a;
                    this->b = static_cast<T>(rot(0, 1) + rot(1, 0)) * s;
                    this->c = static_cast<T>(rot(0, 2) + rot(2, 0)) * s;
                    this->d = static_cast<T>(rot(1, 2) - rot(2, 1)) * s;
                } else if (rot(1, 1) > rot(2, 2)) {
                    const T sb = static_cast<T>(sqrt(rot(1, 1) - rot(2, 2) - rot(0, 0)  + 1));
                    this->b = static_cast<T>(0.5) * sb;

                    const T s = static_cast<T>(0.25) / this->b;
                    this->a = static_cast<T>(rot(0, 1) + rot(1, 0)) * s;
                    this->c = static_cast<T>(rot(1, 2) + rot(2, 1)) * s;
                    this->d = static_cast<T>(rot(0, 2) - rot(2, 0)) * s;
                } else {
                    const T sc = static_cast<T>(sqrt(rot(2, 2) - rot(0, 0) - rot(1, 1)  + 1));
                    this->c = static_cast<T>(0.5) * sc;

                    const T s = static_cast<T>(0.25) / this->c;
                    this->a = static_cast<T>(rot(0, 2) + rot(2, 0)) * s;
                    this->b = static_cast<T>(rot(1, 2) + rot(2, 1)) * s;
                    this->d = static_cast<T>(rot(0, 1) - rot(1, 0)) * s;
                }
            }
#else
*/
            this->d = sqrt( std::max( static_cast<R>(0.0), 1 + rot(0,0) + rot(1,1) + rot(2,2) ) ) / 2;
            this->a = sqrt( std::max( static_cast<R>(0.0), 1 + rot(0,0) - rot(1,1) - rot(2,2) ) ) / 2;
            this->b = sqrt( std::max( static_cast<R>(0.0), 1 - rot(0,0) + rot(1,1) - rot(2,2) ) ) / 2;
            this->c = sqrt( std::max( static_cast<R>(0.0), 1 - rot(0,0) - rot(1,1) + rot(2,2) ) ) / 2;
            this->a = copysign( this->a, rot(2,1) - rot(1,2) );
            this->b = copysign( this->b, rot(0,2) - rot(2,0) );
            this->c = copysign( this->c, rot(1,0) - rot(0,1) );
//#endif

        }



        /**
         * @brief Casts Quaternion<T> to Quaternion<Q>
         * @param quaternion [in] Quarternion with type T
         * @return Quaternion with type Q
         */
        template<class Q>
        inline friend const Quaternion<Q> cast(const Quaternion<T>& quaternion)
        {
            return Quaternion<Q>(
                static_cast<Q>(quaternion(0)),
                static_cast<Q>(quaternion(1)),
                static_cast<Q>(quaternion(2)),
                static_cast<Q>(quaternion(3)));
        }
    };

    /**
       @brief Streaming operator.

       @relates Quaternion
    */
    template <class T>
    std::ostream& operator<<(std::ostream& out, const Quaternion<T>& v)
    {
        return out
            << "Quaternion {"
            << v(0)
            << ", "
            << v(1)
            << ", "
            << v(2)
            << ", "
            << v(3)
            << "}";
    }

    /*@}*/
}} // end namespaces

#endif // end include guard
