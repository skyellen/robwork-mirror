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


#ifndef RW_MATH_VELOCITYSCREW6D_HPP
#define RW_MATH_VELOCITYSCREW6D_HPP

/**
 * @file VelocityScrew6D.hpp
 */

#include <rw/common/Serializable.hpp>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>

#include <Eigen/Eigen>
#include <Eigen/Core>

#include "Transform3D.hpp"
#include "EAA.hpp"
#include "Vector3D.hpp"
#include "Math.hpp"

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief Class for representing 6 degrees of freedom velocity screws.
     *
     * \f[
     * \mathbf{\nu} =
     * \left[
     *  \begin{array}{c}
     *  v_x\\
     *  v_y\\
     *  v_z\\
     *  \omega_x\\
     *  \omega_y\\
     *  \omega_z
     *  \end{array}
     * \right]
     * \f]
     *
     * A VelocityScrew is the description of a frames linear and rotational velocity
     * with respect to some reference frame.
     *
     */
    template<class T = double>
    class VelocityScrew6D
    {
	private:
		T _screw[6]; 

	public:
		/**
         * @brief Constructs a 6 degrees of freedom velocity screw
         *
         * @param vx [in] @f$ v_x @f$
         * @param vy [in] @f$ v_y @f$
         * @param vz [in] @f$ v_z @f$
         * @param wx [in] @f$ \omega_x @f$
         * @param wy [in] @f$ \omega_y @f$
         * @param wz [in] @f$ \omega_z @f$
         */
        VelocityScrew6D(T vx, T vy, T vz, T wx, T wy, T wz);

        /**
         * @brief Construct from Eigen vector representation.
         * @param v [in] Eigen matrix with either one row or one column.
         */
		template <class R>
		VelocityScrew6D(const Eigen::MatrixBase<R>& v) {
			if (v.cols() != 1 || v.rows() != 6)
				RW_THROW("Unable to initialize VectorND with "<<v.rows()<< " x "<<v.cols()<<" matrix");
			/* For some reason the following does not WORK AT ALL (JIMMY)
			_screw[0] = v(0,0);
			_screw[1] = v(1,0);
			_screw[2] = v(2,0);
			_screw[3] = v(3,0);
			_screw[4] = v(4,0);
			_screw[5] = v(5,0);
			*/ // instead use
			_screw[0] = v.row(0)(0);
			_screw[1] = v.row(1)(0);
			_screw[2] = v.row(2)(0);
			_screw[3] = v.row(3)(0);
			_screw[4] = v.row(4)(0);
			_screw[5] = v.row(5)(0);
		}

        /**
         * @brief Default Constructor. Initialized the velocity to 0
         */
		VelocityScrew6D() 
		{
			_screw[0] = _screw[1] = _screw[2] = _screw[3] = _screw[4] = _screw[5] = 0;
		}

        /**
         * @brief Constructs a velocity screw in frame @f$ a @f$ from a
         * transform @f$\robabx{a}{b}{\mathbf{T}} @f$.
         *
         * @param transform [in] the corresponding transform.
         */
        explicit VelocityScrew6D(const Transform3D<T>& transform);

        /**
         * @brief Constructs a velocity screw from a linear and angular velocity
         *
         * @param linear [in] linear velocity
         * @param angular [in] angular velocity
         */
        VelocityScrew6D(const Vector3D<T>& linear, const EAA<T>& angular);

        /**
         * @brief Extracts the linear velocity
         *
         * @return the linear velocity
         */
        const Vector3D<T> linear() const {
			return Vector3D<T>(_screw[0], _screw[1], _screw[2]);
        }

        /**
         * @brief Extracts the angular velocity and represents it using an
         * equivalent-angle-axis as @f$ \dot{\Theta}\mathbf{k} @f$
         *
         * @return the angular velocity
         */
        const EAA<T> angular() const {
            return EAA<T>(_screw[3], _screw[4], _screw[5]);
        }

        /**
         * @brief Returns reference to velocity screw element
         *
         * @param index [in] index in the screw, index must be @f$ < 6 @f$.
         *
         * @return reference to velocity screw element
         */
        T& operator()(std::size_t index) {
            assert(index < 6);
            return _screw[index];
        }

        /**
         * @brief Returns const reference to velocity screw element
         *
         * @param index [in] index in the screw, index must be @f$ < 6 @f$.
         *
         * @return const reference to velocity screw element
         */
        const T& operator()(std::size_t index) const {
            assert(index < 6);
            return _screw[index];
        }

        //! @copydoc operator()
        const T& operator[](size_t i) const { return (*this)(i); }

        //! @copydoc operator()
        T& operator[](size_t i) { return (*this)(i); }


        /**
         * @brief Adds the velocity screw given as a parameter to the velocity screw.
         *
         * @param screw [in] Velocity screw to add
         *
         * @return reference to the VelocityScrew6D to support additional
         * assignments.
         */
        VelocityScrew6D<T>& operator+=(const VelocityScrew6D<T>& screw) {
            _screw[0] += screw(0);
			_screw[1] += screw(1);
			_screw[2] += screw(2);
			_screw[3] += screw(3);
			_screw[4] += screw(4);
			_screw[5] += screw(5);
            return *this;
        }

        /**
         * @brief Subtracts the velocity screw given as a parameter from the
         * velocity screw.
         *
         * @param screw [in] Velocity screw to subtract
         *
         * @return reference to the VelocityScrew6D to support additional
         * assignments.
         */
        VelocityScrew6D<T>& operator-=(const VelocityScrew6D<T>& screw) {
            _screw[0] -= screw(0);
			_screw[1] -= screw(1);
			_screw[2] -= screw(2);
			_screw[3] -= screw(3);
			_screw[4] -= screw(4);
			_screw[5] -= screw(5);            
			return *this;
        }

        /**
         * @brief Scales velocity screw with s
         *
         * @param s [in] scaling value
         *
         * @return reference to the VelocityScrew6D to support additional
         * assigments
         */
        VelocityScrew6D<T>& operator *= (T s) {
			_screw[0] *= s;
			_screw[1] *= s;
			_screw[2] *= s;
			_screw[3] *= s;
			_screw[4] *= s;
			_screw[5] *= s;
            return *this;
        }

        /**
         * @brief Comparison operator.
         *
         * The comparison operator makes a element wise comparison.
         * Returns true only if all elements are equal.
         *
         * @param rhs [in] VelocityScrew6D to compare with
         * @return True if equal.
         */
        bool operator==(const VelocityScrew6D<T> &rhs) const {
            for (int i = 0; i<6; ++i) {
                if (!(_screw[i] == rhs(i))) {
                    return false;
                }
            }
            return true;
        }

        /**
         * @brief Comparison operator.
         *
         * The comparison operator makes a element wise comparison.
         * Returns true if any of the elements are different.
         *
         * @param rhs [in] VelocityScrew6D to compare with
         * @return True if not equal.
         */
        bool operator!=(const VelocityScrew6D<T> &rhs) const {
            return !(*this == rhs);
        }


        /**
         * @brief Scales velocity screw and returns scaled version
         *
         * @param s [in] scaling value
         * @param screw [in] Screw to scale
         * @return Scales screw
         */
        friend const VelocityScrew6D<T> operator*(T s, const VelocityScrew6D& screw) {
            VelocityScrew6D result = screw;
            result *= s;
            return result;
        }

        /**
         * @brief Scales velocity screw and returns scaled version
         * @param s [in] scaling value
         * @return Scales screw
         */
        const VelocityScrew6D<T> operator*( T s) const {
            VelocityScrew6D result = *this;
            result *= s;
            return result;
        }

        /**
         * @brief Changes frame of reference and velocity referencepoint of
         * velocityscrew: @f$ \robabx{b}{b}{\mathbf{\nu}}\to
         * \robabx{a}{a}{\mathbf{\nu}} @f$
         *
         * The frames @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * rigidly connected.
         *
         * @param aTb [in] the location of frame @f$ \mathcal{F}_b @f$ wrt.
         * frame @f$ \mathcal{F}_a @f$: @f$ \robabx{a}{b}{\mathbf{T}} @f$
         *
         * @param bV [in] velocity screw wrt. frame @f$ \mathcal{F}_b @f$: @f$
         * \robabx{b}{b}{\mathbf{\nu}} @f$
         *
         * @return the velocity screw wrt. frame @f$ \mathcal{F}_a @f$: @f$
         * \robabx{a}{a}{\mathbf{\nu}} @f$
         *
         * Transformation of both the velocity reference point and of the base to
         * which the VelocityScrew is expressed
         *
         * \f[
         * \robabx{a}{a}{\mathbf{\nu}} =
         * \left[
         *  \begin{array}{c}
         *  \robabx{a}{a}{\mathbf{v}} \\
         *  \robabx{a}{a}{\mathbf{\omega}}
         *  \end{array}
         * \right] =
         * \left[
         *  \begin{array}{cc}
         *    \robabx{a}{b}{\mathbf{R}} & S(\robabx{a}{b}{\mathbf{p}})
         *    \robabx{a}{b}{\mathbf{R}} \\
         *    \mathbf{0}^{3x3} & \robabx{a}{b}{\mathbf{R}}
         *  \end{array}
         * \right]
         * \robabx{b}{b}{\mathbf{\nu}} =
         * \left[
         *  \begin{array}{c}
         *    \robabx{a}{b}{\mathbf{R}} \robabx{b}{b}{\mathbf{v}} +
         *    \robabx{a}{b}{\mathbf{p}} \times \robabx{a}{b}{\mathbf{R}}
         *    \robabx{b}{b}{\mathbf{\omega}}\\
         *    \robabx{a}{b}{\mathbf{R}} \robabx{b}{b}{\mathbf{\omega}}
         *  \end{array}
         * \right]
         * \f]
         *
         */
        friend const VelocityScrew6D<T> operator*(const Transform3D<T>& aTb,
                                                  const VelocityScrew6D<T>& bV)
        {
            const Vector3D<T>& bv = bV.linear();
            const EAA<T>& bw = bV.angular();
            const EAA<T>& aw = aTb.R() * bw;
            const Vector3D<T>& av = aTb.R() * bv + cross(aTb.P(), aw);
            return VelocityScrew6D<T>(av, aw);
        }

        /**
         * @brief Changes velocity referencepoint of
         * velocityscrew: @f$ \robabx{a}{q}{\mathbf{\nu}}\to
         * \robabx{a}{p}{\mathbf{\nu}} @f$
         *
         * The vector should describe a translation from the current
         * velocity reference point q to the wanted/new velocity reference point p
         * seen from frame @f$ \mathcal{F}_a @f$
         *
         * @param aPqTop [in] the translation from point q to point p seen in
         * frame @f$ \mathcal{F}_a @f$
         *
         * @param aV [in] velocity screw wrt. frame @f$ \mathcal{F}_a @f$: @f$
         * \robabx{a}{q}{\mathbf{\nu}} @f$
         *
         * @return the velocity screw wrt. frame @f$ \mathcal{F}_a @f$: @f$
         * \robabx{a}{p}{\mathbf{\nu}} @f$
         *
         * Transformation of the velocity reference point
         *
         * \f[
         * \robabx{a}{p}{\mathbf{\nu}} =
         * \left[
         *  \begin{array}{c}
         *  \robabx{a}{p}{\mathbf{v}} \\
         *  \robabx{a}{p}{\mathbf{\omega}}
         *  \end{array}
         * \right] =
         * \left[
         *  \begin{array}{cc}
         *    \robabx{a}{b}{\mathbf{R}} & S(\robabx{a}{b}{\mathbf{p}})
         *    \robabx{a}{b}{\mathbf{R}} \\
         *    \mathbf{0}^{3x3} & \robabx{a}{b}{\mathbf{R}}
         *  \end{array}
         * \right]
         * \robabx{a}{p}{\mathbf{\nu}} =
         * \left[
         *  \begin{array}{c}
         *    \robabx{a}{p}{\mathbf{v}} +
         *    \robabx{a}{qTop}{\mathbf{p}}
         *    \robabx{b}{b}{\mathbf{\omega}}\\
         *    \robabx{a}{b}{\mathbf{R}} \robabx{b}{b}{\mathbf{\omega}}
         *  \end{array}
         * \right]
         * \f]
         *
         */
        friend const VelocityScrew6D<T> operator*(const Vector3D<T>& aPqTop,
                                                  const VelocityScrew6D<T>& bV)
        {
            const Vector3D<T>& bv = bV.linear();
            const EAA<T>& bw = bV.angular();
            const Vector3D<T>& av = bv + cross(aPqTop, bw);
            return VelocityScrew6D<T>(av, bw);
        }

        /**
         * @brief Changes frame of reference for velocityscrew: @f$
         * \robabx{b}{i}{\mathbf{\nu}}\to \robabx{a}{i}{\mathbf{\nu}}
         * @f$
         *
         * @param aRb [in] the change in orientation between frame
         * @f$ \mathcal{F}_a @f$ and frame
         * @f$ \mathcal{F}_b @f$: @f$ \robabx{a}{b}{\mathbf{R}} @f$
         *
         * @param bV [in] velocity screw wrt. frame
         * @f$ \mathcal{F}_b @f$: @f$ \robabx{b}{i}{\mathbf{\nu}} @f$
         *
         * @return the velocity screw wrt. frame @f$ \mathcal{F}_a @f$:
         * @f$ \robabx{a}{i}{\mathbf{\nu}} @f$
         *
         * Transformation of the base to which the VelocityScrew is expressed. The velocity
         * reference point is left intact
         *
         * \f[
         * \robabx{a}{i}{\mathbf{\nu}} =
         * \left[
         *  \begin{array}{c}
         *  \robabx{a}{i}{\mathbf{v}} \\
         *  \robabx{a}{i}{\mathbf{\omega}}
         *  \end{array}
         * \right] =
         * \left[
         *  \begin{array}{cc}
         *    \robabx{a}{b}{\mathbf{R}} & \mathbf{0}^{3x3} \\
         *    \mathbf{0}^{3x3} & \robabx{a}{b}{\mathbf{R}}
         *  \end{array}
         * \right]
         * \robabx{b}{i}{\mathbf{\nu}} =
         * \left[
         *  \begin{array}{c}
         *    \robabx{a}{b}{\mathbf{R}} \robabx{b}{i}{\mathbf{v}} \\
         *    \robabx{a}{b}{\mathbf{R}} \robabx{b}{i}{\mathbf{\omega}}
         *  \end{array}
         * \right]
         * \f]
         */
        friend const VelocityScrew6D<T> operator*(const Rotation3D<T>& aRb, const VelocityScrew6D<T>& bV)
        {
            Vector3D<T> bv = bV.linear();
            EAA<T> bw = bV.angular();

            return VelocityScrew6D<T>(aRb*bv, aRb*bw);
        }

        /**
         * @brief Adds two velocity screws together @f$
         * \mathbf{\nu}_{12}=\mathbf{\nu}_1+\mathbf{\nu}_2 @f$
         *
         * @param screw2 [in] @f$ \mathbf{\nu}_2 @f$
         *
         * @return the velocity screw @f$ \mathbf{\nu}_{12} @f$
         */
        const VelocityScrew6D<T> operator+(const VelocityScrew6D<T>& screw2) const
        {			
			return VelocityScrew6D<T>(_screw[0]+screw2._screw[0],
									  _screw[1]+screw2._screw[1],
									  _screw[2]+screw2._screw[2],
									  _screw[3]+screw2._screw[3],
									  _screw[4]+screw2._screw[4],
									  _screw[5]+screw2._screw[5]);
        }

        /**
         * @brief Subtracts two velocity screws
         * \f$\mathbf{\nu}_{12}=\mathbf{\nu}_1-\mathbf{\nu}_2\f$
         *
         * \param screw2 [in] \f$\mathbf{\nu}_2\f$
         * \return the velocity screw \f$\mathbf{\nu}_{12} \f$
         */
        const VelocityScrew6D<T> operator-(const VelocityScrew6D<T>& screw2) const
        {
            return VelocityScrew6D<T>(_screw[0]-screw2._screw[0],
									  _screw[1]-screw2._screw[1],
									  _screw[2]-screw2._screw[2],
									  _screw[3]-screw2._screw[3],
									  _screw[4]-screw2._screw[4],
									  _screw[5]-screw2._screw[5]);
        }

        /**
         * @brief Ouputs velocity screw to stream
         *
         * @param os [in/out] stream to use
         * @param screw [in] velocity screw
         * @return the resulting stream
         */
        friend std::ostream& operator<<(std::ostream& os, const VelocityScrew6D<T>& screw)
        {
			return os << "{{"<<screw(0)<<","<<screw(1)<<","<<screw(2)<<"},{"<<screw(3)<<","<<screw(4)<<","<<screw(5)<<"}}";
        }

        /**
         * @brief Takes the 1-norm of the velocity screw. All elements both
         * angular and linear are given the same weight.
         *
         * @return the 1-norm
         */
        T norm1() const {
			return fabs(_screw[0])+fabs(_screw[1])+fabs(_screw[2])+fabs(_screw[3])+fabs(_screw[4])+fabs(_screw[5]);
        }

        /**
         * @brief Takes the 2-norm of the velocity screw. All elements both
         * angular and linear are given the same weight
         *
         * @return the 2-norm
         */
        T norm2() const {
			return std::sqrt(Math::sqr(_screw[0])+Math::sqr(_screw[1])+Math::sqr(_screw[2])+Math::sqr(_screw[3])+Math::sqr(_screw[4])+Math::sqr(_screw[5]));
        }

        /**
         * @brief Takes the infinite norm of the velocity screw. All elements
         * both angular and linear are given the same weight.
         *
         * @return the infinite norm
         */
        T normInf() const {
			return std::max(fabs(_screw[0]), std::max(fabs(_screw[1]), std::max(fabs(_screw[2]), std::max(fabs(_screw[3]), std::max(fabs(_screw[4]),fabs(_screw[5]))))));
        }

        /**
           @brief Construct a velocity screw from a Boost vector expression.
        */
        template <class R>
        explicit VelocityScrew6D(const boost::numeric::ublas::vector_expression<R>& r)            
        {
			boost::numeric::ublas::bounded_vector<T, 6> v(r);
			for (size_t i = 0; i<6; i++) {
				_screw[i] = v(i);
			}
		}


        /**
           @brief Converter to Boost bounded_vector
         */
        boost::numeric::ublas::bounded_vector<T, 6> m() const { 
			boost::numeric::ublas::bounded_vector<T, 6> m;
			for (size_t i = 0; i<6; i++)
				m(i) = _screw[i];
			return m; 
		}

        /**
           @brief Converter to Eigen vector
         */
		Eigen::Matrix<T, 6, 1> e() const {
			Eigen::Matrix<T, 6, 1> res;
			for (size_t i = 0; i<6; i++)
				res(i) = _screw[i];
			return res;
		}


	private:


    };

	/**
	* @brief Takes the 1-norm of the velocity screw. All elements both
	* angular and linear are given the same weight.
	*
	* @param screw [in] the velocity screw
	* @return the 1-norm
	*/
	template <class T>
	T norm1(const VelocityScrew6D<T>& screw)
	{
		return screw.norm1();
	}

	/**
	* @brief Takes the 2-norm of the velocity screw. All elements both
	* angular and linear are given the same weight
	*
	* @param screw [in] the velocity screw
	* @return the 2-norm
	*/
	template <class T>
	T norm2(const VelocityScrew6D<T>& screw)
	{
		return screw.norm2();
	}

	/**
	* @brief Takes the infinite norm of the velocity screw. All elements
	* both angular and linear are given the same weight.
	*
	* @param screw [in] the velocity screw
	*
	* @return the infinite norm
	*/
	template <class T>
	T normInf(const VelocityScrew6D<T>& screw)
	{
		return screw.normInf();
	}

	/**
	* @brief Casts VelocityScrew6D<T> to VelocityScrew6D<Q>
	*
	* @param vs [in] VelocityScrew6D with type T
	*
	* @return VelocityScrew6D with type Q
	*/
	template<class Q, class T>
	const VelocityScrew6D<Q> cast(const VelocityScrew6D<T>& vs)
	{
		return VelocityScrew6D<Q>(
			static_cast<Q>(vs(0)),
			static_cast<Q>(vs(1)),
			static_cast<Q>(vs(2)),
			static_cast<Q>(vs(3)),
			static_cast<Q>(vs(4)),
			static_cast<Q>(vs(5)));
	}

    /*@}*/
}} // end namespaces


namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
	/**
	 * @copydoc rw::common::serialization::write
	 * @relatedalso rw::math::VelocityScrew6D
	 */
    template<> void write(const rw::math::VelocityScrew6D<double>& sobject, rw::common::OutputArchive& oarchive, const std::string& id);

	/**
	 * @copydoc rw::common::serialization::write
	 * @relatedalso rw::math::VelocityScrew6D
	 */
    template<> void write(const rw::math::VelocityScrew6D<float>& sobject, rw::common::OutputArchive& oarchive, const std::string& id);

	/**
	 * @copydoc rw::common::serialization::read
	 * @relatedalso rw::math::VelocityScrew6D
	 */
    template<> void read(rw::math::VelocityScrew6D<double>& sobject, rw::common::InputArchive& iarchive, const std::string& id);

	/**
	 * @copydoc rw::common::serialization::read
	 * @relatedalso rw::math::VelocityScrew6D
	 */
    template<> void read(rw::math::VelocityScrew6D<float>& sobject, rw::common::InputArchive& iarchive, const std::string& id);
}}} // end namespaces

#endif // end include guard
