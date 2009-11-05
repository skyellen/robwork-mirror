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


#ifndef RW_MATH_TRANSFORM3D_HPP
#define RW_MATH_TRANSFORM3D_HPP

/**
 * @file Transform3D.hpp
 */

#include "Vector3D.hpp"
#include "Rotation3D.hpp"
#include "Rotation3DVector.hpp"

#include <cassert>

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A 4x4 homogeneous transform matrix @f$ \mathbf{T}\in SE(3) @f$
     *
     * @f$
     * \mathbf{T} =
     * \left[
     *  \begin{array}{cc}
     *  \mathbf{R} & \mathbf{d} \\
     *  \begin{array}{ccc}0 & 0 & 0\end{array} & 1
     *  \end{array}
     * \right]
     * @f$
     *
     */
    template<class T = double>
    class Transform3D
    {
    public:
        /**
         * @brief Default Constructor.
         *
         * Initializes with 0 translation and Identity matrix as rotation
         */
        Transform3D() :
            _d(),
            _R(Rotation3D<T>::identity())
        {}

        /**
         * @brief Constructs a homogeneous transform
         * @param d [in] @f$ \mathbf{d} @f$ A 3x1 translation vector
         * @param R [in] @f$ \mathbf{R} @f$ A 3x3 rotation matrix
         */
        Transform3D(const Vector3D<T>& d, const Rotation3D<T>& R) :
            _d(d),
            _R(R)
        {}

        /**
           @brief A homogeneous transform with a rotation of \b R and a
           translation of zero.
        */
        explicit Transform3D(const Rotation3D<T>& R) :
            _d(0, 0, 0),
            _R(R)
        {}

        /**
           @brief A homogeneous transform with a rotation of zero and a
           translation of \b d.
        */
        explicit Transform3D(const Vector3D<T>& d) :
            _d(d),
            _R(Rotation3D<T>::identity())
        {}

        /**
         * @brief Constructs a homogeneous transform
         *
         * Calling this constructor is equivalent to the transform
         * Transform3D(d, r.toRotation3D()).
         *
         * @param d [in] @f$ \mathbf{d} @f$ A 3x1 translation vector
         * @param r [in] @f$ \mathbf{r} @f$ A 3x1 rotation vector
         */
        Transform3D(const Vector3D<T>& d, const Rotation3DVector<T>& r) :
            _d(d),
            _R(r.toRotation3D())
        {}

        /**
         * @brief Constructs a homogeneous transform using the original
         * Denavit-Hartenberg notation
         *
         * @param alpha [in] @f$ \alpha_i @f$
         * @param a [in] @f$ a_i @f$
         * @param d [in] @f$ d_i @f$
         * @param theta [in] @f$ \theta_i @f$
         * @return @f$ ^{i-1}\mathbf{T}_i @f$
         *
         * @f$
         *  \robabx{i-1}{i}{\mathbf{T}}=
         *  \left[
         *    \begin{array}{cccc}
         *      c\theta_i & -s\theta_i c\alpha_i &  s\theta_i s\alpha_i & a_i c\theta_i \\
         *      s\theta_i &  c\theta_i c\alpha_i & -c\theta_i s\alpha_i & a_i s\theta_i \\
         *      0         &  s\alpha_i           &  c\alpha_i           & d_i \\
         *      0         &  0                   & 0                    & 1
         *    \end{array}
         *  \right]
         * @f$
         */
        static Transform3D DH(T alpha, T a, T d, T theta);

        /**
         * @brief Constructs a homogeneous transform using the Craig (modified)
         * Denavit-Hartenberg notation
         *
         * @param alpha [in] @f$ \alpha_{i-1} @f$
         * @param a [in] \f$ a_{i-1} \f$
         * @param d [in] \f$ d_i \f$
         * @param theta [in] \f$ \theta_i \f$
         * @return @f$ \robabx{i-1}{i}{\mathbf{T}} @f$
         *
         * @note The Craig (modified) Denavit-Hartenberg notation differs from
         * the original Denavit-Hartenberg notation and is given as
         *
         * @f$
         * \robabx{i-1}{i}{\mathbf{T}} =
         * \left[
         * \begin{array}{cccc}
         * c\theta_i & -s\theta_i & 0 & a_{i-1} \\
         * s\theta_i c\alpha_{i-1} & c\theta_i c\alpha_{i-1} & -s\alpha_{i-1} & -s\alpha_{i-1}d_i \\
         * s\theta_i s\alpha_{i-1} & c\theta_i s\alpha_{i-1} &  c\alpha_{i-1} &  c\alpha_{i-1}d_i \\
         * 0 & 0 & 0 & 1
         * \end{array}
         * \right]
         * @f$
         *
         */
        static Transform3D craigDH(T alpha, T a, T d, T theta);



        /**
         * @brief Constructs the identity transform
         * @return the identity transform
         *
         * @f$
         * \mathbf{T} =
         * \left[
         * \begin{array}{cccc}
         * 1 & 0 & 0 & 0\\
         * 0 & 1 & 0 & 0\\
         * 0 & 0 & 1 & 0\\
         * 0 & 0 & 0 & 1
         * \end{array}
         * \right]
         * @f$
         */
        // Is implemented in header because of some bugs in mingw
        static const Transform3D& identity()
        {
            static const Transform3D id(
                Vector3D<T>(0, 0, 0),
                Rotation3D<T>::identity());
            return id;
        }



        /**
         * @brief Returns matrix element reference
         * @param row [in] row, row must be @f$ < 3 @f$
         * @param col [in] col, col must be @f$ < 4 @f$
         * @return reference to matrix element
         */
        T& operator()(std::size_t row, std::size_t col)
        {
            assert(row < 3);
            assert(col < 4);
            if(row < 3 && col < 3)
                return _R( row, col);
            else
                return _d( row );
        }

        /**
         * @brief Returns const matrix element reference
         * @param row [in] row, row must be @f$ < 3 @f$
         * @param col [in] col, col must be @f$ < 4 @f$
         * @return const reference to matrix element
         */
        const T& operator()(std::size_t row, std::size_t col) const {
            assert(row < 3);
            assert(col < 4);
            if(row < 3 && col < 3)
                return _R( row, col);
            else
                return _d( row );
        }


        /**
         * @brief Comparison operator.
         *
         * The comparison operator makes a element wise comparison with the precision of T.
         * Returns true only if all elements are equal.
         *
         * @param rhs [in] Transform to compare with
         * @return True if equal.
         */
        bool operator==(const Transform3D<> &rhs) const {
            return (R() == rhs.R());// && (P() == rhs.P());
        }

        /**
         * @brief Calculates @f$ \robabx{a}{c}{\mathbf{T}} = \robabx{a}{b}{\mathbf{T}} \robabx{b}{c}{\mathbf{T}} @f$
         * @param aTb [in] @f$ \robabx{a}{b}{\mathbf{T}} @f$
         * @param bTc [in] @f$ \robabx{b}{c}{\mathbf{T}} @f$
         * @return @f$ \robabx{a}{c}{\mathbf{T}} @f$
         *
         * @f$
         * \robabx{a}{c}{\mathbf{T}} =
         * \left[
         *  \begin{array}{cc}
         *  \robabx{a}{b}{\mathbf{R}}\robabx{b}{c}{\mathbf{R}} & \robabx{a}{b}{\mathbf{d}} + \robabx{a}{b}{\mathbf{R}}\robabx{b}{c}{\mathbf{d}} \\
         *  \begin{array}{ccc}0 & 0 & 0\end{array} & 1
         *  \end{array}
         * \right]
         * @f$
         */
        friend Transform3D operator*(const Transform3D& aTb, const Transform3D& bTc)
        {
            return Transform3D(aTb._d + aTb._R * bTc._d, aTb._R * bTc._R);
        }

        /**
         * @brief Calculates @f$ \robax{a}{\mathbf{p}} = \robabx{a}{b}{\mathbf{T}} \robax{b}{\mathbf{p}} \f$ thus transforming point @f$ \mathbf{p} @f$ from frame @f$ b @f$ to frame @f$ a @f$
         * @param aTb [in] @f$ \robabx{a}{c}{\mathbf{T}} @f$
         * @param bP [in] @f$ \robax{b}{\mathbf{p}} @f$
         * @return @f$ \robax{a}{\mathbf{p}} @f$
         */
        friend Vector3D<T> operator*(const Transform3D& aTb, const Vector3D<T>& bP)
        {
            return aTb._R * bP + aTb._d ;
        }

        /**
         * @brief Gets the rotation part @f$ \mathbf{R} @f$ from @f$ \mathbf{T} @f$
         * @return @f$ \mathbf{R} @f$
         */
        Rotation3D<T>& R() { return _R; }

        /**
         * @brief Gets the rotation part @f$ \mathbf{R} @f$ from @f$ \mathbf{T} @f$
         * @return @f$ \mathbf{R} @f$
         */
        const Rotation3D<T>& R() const { return _R; }

        /**
         * \brief Gets the position part @f$ \mathbf{d} @f$ from @f$ \mathbf{T} @f$
         * \return @f$ \mathbf{d} @f$
         */
        Vector3D<T>& P() { return _d; }

        /**
         * @brief Gets the position part @f$ \mathbf{d} @f$ from @f$ \mathbf{T} @f$
         * @return @f$ \mathbf{d} @f$
         */
        const Vector3D<T>& P() const { return _d; }

        /**
         * @brief Outputs transform to stream
         * @param os [in/out] an output stream
         * @param t [in] the transform that is to be sent to the output stream
         * @return os
         */
        friend std::ostream& operator<<(std::ostream &os, const Transform3D<T>& t)
        {
            // This format matches the Lua notation.
            return os
                << "Transform3D("
                << t.P()
                << ", "
                << t.R()
                << ")";
        }

        /**
         * @brief Cast Transform3D<T> to Transform3D<Q>
         * @param trans [in] Transform3D with type T
         * @return Transform3D with type Q
         */
        template<class Q>
        friend Transform3D<Q> cast(const Transform3D<T>& trans)
        {
            Transform3D<Q> res;
            for (size_t i = 0; i<3; i++)
                for (size_t j = 0; j<4; j++)
                    res(i,j) = static_cast<Q>(trans(i,j));
            return res;
        }

        /**
           @brief Write to \b result the product \b a * \b b.
        */
        static inline void multiply(const Transform3D<T>& a,
                                    const Transform3D<T>& b,
                                    Transform3D<T>& result)
        {
            Rotation3D<T>::multiply(a.R(), b.R(), result.R());
            Rotation3D<T>::multiply(a.R(), b.P(), result.P());
            result.P() += a.P();
        }

    private:
        Vector3D<T> _d;
        Rotation3D<T> _R;
    };



    /**
     * @brief Calculates
     * @f$ \robabx{b}{a}{\mathbf{T}} = \robabx{a}{b}{\mathbf{T}}^{-1} @f$
     *
     * @relates Transform3D
     *
     * @param aTb [in] the transform matrix @f$ \robabx{a}{b}{\mathbf{T}} @f$
     * @return @f$ \robabx{b}{a}{\mathbf{T}} = \robabx{a}{b}{\mathbf{T}}^{-1} @f$
     *
     * @f$
     * \robabx{a}{b}{\mathbf{T}}^{-1} =
     * \left[
     *  \begin{array}{cc}
     *  \robabx{a}{b}{\mathbf{R}}^{T} & - \robabx{a}{b}{\mathbf{R}}^{T} \robabx{a}{b}{\mathbf{d}} \\
     *  \begin{array}{ccc}0 & 0 & 0\end{array} & 1
     *  \end{array}
     * \right]
     *
     * @f$
     */
    template <class T>
    Transform3D<T> inverse(const Transform3D<T>& aTb)
    {
        return Transform3D<T>(
            -(inverse(aTb.R()) * aTb.P()),
            inverse(aTb.R()));
    }

    /*@}*/

}} // end namespaces

#endif // end include guard
