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


#ifndef RW_MATH_WRENCH6D_HPP
#define RW_MATH_WRENCH6D_HPP

/**
 * @file Wrench6D.hpp
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
     * @brief Class for representing 6 degrees of freedom wrenches.
     *
     * \f[
     * \mathbf{\nu} =
     * \left[
     *  \begin{array}{c}
     *  f_x\\
     *  f_y\\
     *  f_z\\
     *  \tau_x\\
     *  \tau_y\\
     *  \tau_z
     *  \end{array}
     * \right]
     * \f]
     *
     * A Wrench is the description of a frames linear force and rotational torque
     * with respect to some reference frame.
     *
     */
    template<class T = double>
    class Wrench6D
    {
	private:
		T _wrench[6];

	public:		
		/**
         * @brief Constructs a 6 degrees of freedom velocity screw
         *
         * @param fx [in] @f$ f_x @f$
         * @param fy [in] @f$ f_y @f$
         * @param fz [in] @f$ f_z @f$
         * @param tx [in] @f$ \tau_x @f$
         * @param ty [in] @f$ \tau_y @f$
         * @param tz [in] @f$ \tau_z @f$
         */
        Wrench6D(T fx, T fy, T fz, T tx, T ty, T tz);

		/**
		 * @brief Constructs based on Eigen data type
		 */
		template <class R>
		Wrench6D(const Eigen::MatrixBase<R>& v) {
			if (v.cols() != 1 || v.rows() != 6)
				RW_THROW("Unable to initialize VectorND with "<<v.rows()<< " x "<<v.cols()<<" matrix");
			for (size_t i = 0; i<6; i++)
				_wrench[i] = v.row(i)(0);
		}

        /**
         * @brief Default Constructor. Initialized the wrench to 0
         */
		Wrench6D() 
        {
			_wrench[0] = _wrench[1] = _wrench[2] = _wrench[3] = _wrench[4] = _wrench[5] = 0;
		}

        /**
         * @brief Constructs a wrench from a force and torque
         *
         * @param force [in] linear force
         * @param torque [in] angular torque
         */
        Wrench6D(const Vector3D<T>& force, const Vector3D<T>& torque);


        /**
           @brief Construct a wrench from a Boost vector expression.
        */
        template <class R>
        explicit Wrench6D(const boost::numeric::ublas::vector_expression<R>& r)
        {
			boost::numeric::ublas::bounded_vector<T, 6> v(r);
			for (size_t i = 0; i<6; i++) {
				_wrench[i] = v(i);
			}
		}

        /**
         * @brief Sets the force component
         *
         * @param force [in] linear force
         */
        void setForce(const Vector3D<T>& force) {
            _wrench[0] = force(0);
            _wrench[1] = force(1);
            _wrench[2] = force(2);
        }

        /**
         * @brief Sets the torque component
         *
         * @param torque [in] angular torque
         */
        void setTorque(const Vector3D<T>& torque) {
            _wrench[3] = torque(0);
            _wrench[4] = torque(1);
            _wrench[5] = torque(2);
        }

        /**
         * @brief Extracts the force
         *
         * @return the force
         */
        const Vector3D<T> force() const {
            return Vector3D<T>(_wrench[0], _wrench[1], _wrench[2]);
        }

        /**
         * @brief Extracts the torque and represents it using an Vector3D<T>         
         *
         * @return the torque
         */
        const Vector3D<T> torque() const {
            return Vector3D<T>(_wrench[3], _wrench[4], _wrench[5]);
        }

        /**
         * @brief Returns reference to wrench element
         *
         * @param index [in] index in the wrench, index must be @f$ < 6 @f$.
         *
         * @return reference to wrench element
         */
        T& operator()(std::size_t index) {
            assert(index < 6);
			return _wrench[index];
        }

        /**
         * @brief Returns const reference to wrench element
         *
         * @param index [in] index in the wrench, index must be @f$ < 6 @f$.
         *
         * @return const reference to wrench element
         */
        const T& operator()(std::size_t index) const {
            assert(index < 6);
            return _wrench[index];
        }

        //! @copydoc operator()
        const T& operator[](size_t i) const { return (*this)(i); }

        //! @copydoc operator()
        T& operator[](size_t i) { return (*this)(i); }


        /**
         * @brief Adds the wrench given as a parameter to the wrench. 
		 *
		 * Assumes the wrenches are represented in the same coordinate system.
         *
         * @param wrench [in] Wrench to add
         *
         * @return reference to the Wrench6D to support additional assignments.
         */
        Wrench6D<T>& operator+=(const Wrench6D<T>& wrench) {
            for (size_t i = 0; i<6; i++)	 
				_wrench[i] += wrench(i);
            return *this;
        }

        /**
         * @brief Subtracts the wrench given as a parameter from the wrench.
         *
		 * Assumes the wrenches are represented in the same coordinate system.
		 *
         * @param wrench [in] Velocity screw to subtract
         *
         * @return reference to the Wrench6D to support additional
         * assignments.
         */
        Wrench6D<T>& operator-=(const Wrench6D<T>& wrench) {
            for (size_t i = 0; i<6; i++)	 
				_wrench[i] -= wrench(i);
            return *this;
        }

        /**
         * @brief Scales wrench with s
         *
         * @param s [in] scaling value
         *
         * @return reference to the Wrench6D to support additional
         * assigments
         */
        Wrench6D<T>& operator *= (T s) {
            for (size_t i = 0; i<6; i++)	 
				_wrench[i] *= s;

            return *this;
        }



        /**
         * @brief Scales wrench and returns scaled version
         * @param s [in] scaling value
         * @return Scaled wrench
         */
        const Wrench6D<T> operator*( T s) const {
            Wrench6D result = *this;
            result *= s;
            return result;
        }

        /**
         * @brief Changes frame of reference and referencepoint of
         * wrench: @f$ \robabx{b}{b}{\mathbf{w}}\to
         * \robabx{a}{a}{\mathbf{w}} @f$
         *
         * The frames @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * rigidly connected.
         *
         * @param aTb [in] the location of frame @f$ \mathcal{F}_b @f$ wrt.
         * frame @f$ \mathcal{F}_a @f$: @f$ \robabx{a}{b}{\mathbf{T}} @f$
         *
         * @param bV [in] wrench wrt. frame @f$ \mathcal{F}_b @f$: @f$
         * \robabx{b}{b}{\mathbf{\nu}} @f$
         *
         * @return the wrench wrt. frame @f$ \mathcal{F}_a @f$: @f$
         * \robabx{a}{a}{\mathbf{\nu}} @f$
         *
         * Transformation of both the wrench reference point and of the base to
         * which the wrench is expressed
         *
         * \f[
         * \robabx{a}{a}{\mathbf{w}} =
         * \left[
         *  \begin{array}{c}
         *  \robabx{a}{a}{\mathbf{force}} \\
         *  \robabx{a}{a}{\mathbf{torque}}
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
        friend const Wrench6D<T> operator*(const Transform3D<T>& aTb,
                                                  const Wrench6D<T>& bV)
        {
            const Vector3D<T>& bv = bV.force();
            const Vector3D<T>& bw = bV.torque();
            const Vector3D<T>& aw = aTb.R() * bw;
            const Vector3D<T>& av = aTb.R() * bv + cross(aTb.P(), aw);
            return Wrench6D<T>(av, aw);
        }

        /**
         * @brief Changes wrench referencepoint of
         * wrench: @f$ \robabx{b}{b}{\mathbf{w}}\to
         * \robabx{a}{a}{\mathbf{w}} @f$
         *
         * The frames @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * rigidly connected.
         *
         * @param aPb [in] the location of frame @f$ \mathcal{F}_b @f$ wrt.
         * frame @f$ \mathcal{F}_a @f$: @f$ \robabx{a}{b}{\mathbf{T}} @f$
         *
         * @param bV [in] wrench wrt. frame @f$ \mathcal{F}_b @f$: @f$
         * \robabx{b}{b}{\mathbf{\nu}} @f$
         *
         * @return the wrench wrt. frame @f$ \mathcal{F}_a @f$: @f$
         * \robabx{a}{a}{\mathbf{\nu}} @f$
         *
         * Transformation of both the velocity reference point and of the base to
         * which the wrench is expressed
         *
         * \f[
         * \robabx{a}{a}{\mathbf{w}} =
         * \left[
         *  \begin{array}{c}
         *  \robabx{a}{a}{\mathbf{force}} \\
         *  \robabx{a}{a}{\mathbf{torque}}
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
        friend const Wrench6D<T> operator*(const Vector3D<T>& aPb,
                                           const Wrench6D<T>& bV)
        {
            const Vector3D<T>& bv = bV.force();
            const Vector3D<T>& bw = bV.torque();
            const Vector3D<T>& av = bv + cross(aPb, bw);
            return Wrench6D<T>(av, bw);
        }

        /**
         * @brief Changes frame of reference for wrench: @f$
         * \robabx{b}{i}{\mathbf{w}}\to \robabx{a}{i}{\mathbf{w}}
         * @f$
         *
         * @param aRb [in] the change in orientation between frame
         * @f$ \mathcal{F}_a @f$ and frame
         * @f$ \mathcal{F}_b @f$: @f$ \robabx{a}{b}{\mathbf{R}} @f$
         *
         * @param bV [in] velocity screw wrt. frame
         * @f$ \mathcal{F}_b @f$: @f$ \robabx{b}{i}{\mathbf{\nu}} @f$
         *
         * @return the wrench wrt. frame @f$ \mathcal{F}_a @f$:
         * @f$ \robabx{a}{i}{\mathbf{w}} @f$
         *
         * Transformation of the base to which the wrench is expressed. The wrench
         * reference point is left intact
         *
         * \f[
         * \robabx{a}{i}{\mathbf{w}} =
         * \left[
         *  \begin{array}{c}
         *  \robabx{a}{i}{\mathbf{force}} \\
         *  \robabx{a}{i}{\mathbf{torque}}
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
        friend const Wrench6D<T> operator*(const Rotation3D<T>& aRb, const Wrench6D<T>& bV)
        {
            Vector3D<T> bv = bV.force();
            Vector3D<T> bw = bV.torque();

            return Wrench6D<T>(aRb*bv, aRb*bw);
        }

        /**
         * @brief Adds two wrenches together @f$
         * \mathbf{w}_{12}=\mathbf{w}_1+\mathbf{w}_2 @f$
         *
         * @param rhs [in] @f$ \mathbf{\nu}_1 @f$
         *
         * @return the wrench @f$ \mathbf{w}_{12} @f$
         */
        const Wrench6D<T> operator+(const Wrench6D<T>& rhs) const
        {
			return Wrench6D<T>(_wrench[0]+rhs(0),_wrench[1]+rhs(1),_wrench[2]+rhs(2),_wrench[3]+rhs(3),_wrench[4]+rhs(4),_wrench[5]+rhs(5));
        }

        /**
         * @brief Subtracts two velocity screws
         * \f$\mathbf{\nu}_{12}=\mathbf{\nu}_1-\mathbf{\nu}_2\f$
         *
         * \param rhs [in] \f$\mathbf{w}_1\f$
         * \return the wrench \f$\mathbf{w}_{12} \f$
         */
        const Wrench6D<T> operator-(const Wrench6D<T>& rhs) const
        {
			return Wrench6D<T>(_wrench[0]-rhs(0),_wrench[1]-rhs(1),_wrench[2]-rhs(2),_wrench[3]-rhs(3),_wrench[4]-rhs(4),_wrench[5]-rhs(5));
        }

        /**
         * @brief Ouputs wrench to stream
         *
         * @param os [in/out] stream to use
         * @param wrench [in] the wrench
         * @return the resulting stream
         */
        friend std::ostream& operator<<(std::ostream& os, const Wrench6D<T>& wrench)
        {
			return os << "{{"<<wrench(0)<<","<<wrench(1)<<","<<wrench(2)<<"},{"<<wrench(3)<<","<<wrench(4)<<","<<wrench(5)<<"}}";
            //return os << wrench.e();
        }

        /**
         * @brief Takes the 1-norm of the wrench. All elements both
         * force and torque are given the same weight.
         * @return the 1-norm
         */
        T norm1() const {
			return fabs(_wrench[0])+fabs(_wrench[1])+fabs(_wrench[2])+fabs(_wrench[3])+fabs(_wrench[4])+fabs(_wrench[5]);
            //return _wrench.template lpNorm<1>();
        }

        /**
         * @brief Takes the 2-norm of the wrench. All elements both
         * force and torque are given the same weight
         * @return the 2-norm
         */
        T norm2() const {
            return std::sqrt(Math::sqr(_wrench[0])+Math::sqr(_wrench[1])+Math::sqr(_wrench[2])+Math::sqr(_wrench[3])+Math::sqr(_wrench[4])+Math::sqr(_wrench[5]));
        }

        /**
         * @brief Takes the infinite norm of the wrench. All elements
         * both force and torque are given the same weight.
         *
         * @return the infinite norm
         */
        T normInf() const {
			return std::max(fabs(_wrench[0]), std::max(fabs(_wrench[1]), std::max(fabs(_wrench[2]), std::max(fabs(_wrench[3]), std::max(fabs(_wrench[4]),fabs(_wrench[5]))))));
        }

        /**
           @brief Converter to Boost type.
         */
        boost::numeric::ublas::bounded_vector<T, 6> m() const { 
			boost::numeric::ublas::bounded_vector<T, 6> m;
			for (size_t i = 0; i<6; i++)
				m(i) = _wrench[i];
			return m; 
		}

        /**
           @brief Converter to Eigen data type
         */
		Eigen::Matrix<T, 6, 1> e() const {
			Eigen::Matrix<T, 6, 1> res;
			for (size_t i = 0; i<6; i++)
				res(i) = _wrench[i];
			return res;
		}

		/**
         * @brief Compares \b a and \b b for equality.
		 * @param b [in] other wrench to compare with.
         * @return True if a equals b, false otherwise.
		 */
        bool operator==(const Wrench6D<T>& b) const {
        	return _wrench[0] == b[0] && _wrench[1] == b[1] && _wrench[2] == b[2] && _wrench[3] == b[3] && _wrench[4] == b[4] && _wrench[5] == b[5];
        }

        /**
         * @brief Compares \b a and \b b for inequality.
		 * @param b [in] other wrench to compare with.
         * @return True if a and b are different, false otherwise.
         */
        bool operator!=(const Wrench6D<T>& b) const {
            return !(*this == b);
        }

    };

	/**
	* @brief Takes the 1-norm of the wrench. All elements both
	* force and torque are given the same weight.
	*
	* @param wrench [in] the wrench
	* @return the 1-norm
	*/
	template <class T>
	T norm1(const Wrench6D<T>& wrench)
	{
		return wrench.norm1();
	}

	/**
	* @brief Takes the 2-norm of the wrench. All elements both
	* force and tporque are given the same weight
	*
	* @param wrench [in] the wrench
	* @return the 2-norm
	*/
	template <class T>
	T norm2(const Wrench6D<T>& wrench)
	{
		return wrench.norm2();
	}

	/**
	* @brief Takes the infinite norm of the wrench. All elements
	* both force and torque are given the same weight.
	*
	* @param wrench [in] the wrench
	*
	* @return the infinite norm
	*/
	template <class T>
	T normInf(const Wrench6D<T>& wrench)
	{
		return wrench.normInf();
	}

	/**
	* @brief Casts Wrench6D<T> to Wrench6D<Q>
	*
	* @param vs [in] Wrench6D with type T
	*
	* @return Wrench6D with type Q
	*/
	template<class Q, class T>
	const Wrench6D<Q> cast(const Wrench6D<T>& vs)
	{
		return Wrench6D<Q>(
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
 * @relatedalso rw::math::Wrench6D
 */
template<> void write(const rw::math::Wrench6D<double>& sobject, rw::common::OutputArchive& oarchive, const std::string& id);

/**
 * @copydoc rw::common::serialization::write
 * @relatedalso rw::math::Wrench6D
 */
template<> void write(const rw::math::Wrench6D<float>& sobject, rw::common::OutputArchive& oarchive, const std::string& id);

/**
 * @copydoc rw::common::serialization::read
 * @relatedalso rw::math::Wrench6D
 */
template<> void read(rw::math::Wrench6D<double>& sobject, rw::common::InputArchive& iarchive, const std::string& id);

/**
 * @copydoc rw::common::serialization::read
 * @relatedalso rw::math::Wrench6D
 */
template<> void read(rw::math::Wrench6D<float>& sobject, rw::common::InputArchive& iarchive, const std::string& id);
}}} // end namespaces

#endif // end include guard
