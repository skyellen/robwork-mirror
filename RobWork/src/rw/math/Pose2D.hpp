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

#ifndef RW_MATH_POSE2D_HPP_
#define RW_MATH_POSE2D_HPP_

#include <rw/math/Transform2D.hpp>
#include <boost/numeric/ublas/vector.hpp>

namespace rw {
namespace math {

	/** @addtogroup math */
	/*@{*/

	/**
	 * @brief A Pose3D @f$ \mathbf{x}\in \mathbb{R}^6 @f$ describes a position
	 * and orientation in 3-dimensions.
	 *
	 * @f$ {\mathbf{x}} = \left[
	 *  \begin{array}{c}
	 *  x \\
	 *  y \\
	 *  z \\
	 *  \theta k_x \\
	 *  \theta k_y \\
	 *  \theta k_z
	 *  \end{array}
	 *  \right]
	 *  @f$
	 *
	 * where @f$ (x,y,z)@f$ is the 3d position and @f$ (\theta k_x, \theta k_y,
	 * \theta k_z)@f$ describes the orientation in equal angle axis (EAA)
	 * format.
	 */
	template<class T = double>

	class Pose2D
	{
	public:
		//! @brief Zero-initialized Pose2D.
		Pose2D() :
			_pos(0,0),
			_theta(0)
		{}

		/**
		 * @brief Constructor.
		 * @param pos [in] the position.
		 * @param theta [in] the angle.
		 */
		Pose2D(rw::math::Vector2D<T> pos, T theta) :
			_pos(pos),
			_theta(theta)
		{}

		/**
		 * @brief Constructor.
		 * @param x [in] the value of the first position dimension.
		 * @param y [in] the value of the second position dimension.
		 * @param theta [in] the angle.
		 */
		Pose2D(T x, T y, T theta) :
			_pos(x,y), _theta(theta)
		{}

		/**
		 * @brief Constructor.
		 * @param transform [in] a 2D transform giving the pose.
		 */
		Pose2D(const rw::math::Transform2D<T>& transform) :
			_pos(transform.P()),
			_theta(
				// Sigh.
				atan2(
					transform.R()(1, 0),
					transform.R()(0, 0)))
		{}

		/**
		 * @brief Get the first dimension of the position vector.
		 * @return the position in the first dimension.
		 */
		T& x(){return _pos[0];}

		/**
		 * @brief Get the second dimension of the position vector.
		 * @return the position in the second dimension.
		 */
		T& y(){return _pos[1];}

		/**
		 * @brief Get the angle.
		 * @return the angle.
		 */
		T& theta(){return _theta;}

		/**
		 * @brief Get the position vector.
		 * @return the position.
		 */
		rw::math::Vector2D<T>& getPos(){return _pos;}

		//! @copydoc x()
		T x() const { return _pos[0]; }

		//! @copydoc y()
		T y() const { return _pos[1]; }

		//! @copydoc theta()
		T theta() const { return _theta; }

		//! @copydoc getPos()
		const rw::math::Vector2D<T>& getPos() const { return _pos; }

        /**
         * @brief Returns reference to vector element (x,y,theta)
         *
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         *
         * @return const reference to element
         */
        const T& operator()(size_t i) const {
        	if(i<2)
        		return _pos[i];
        	return _theta;
        }

        /**
         * @brief Returns reference to vector element
         *
         * @param i [in] index in the vector \f$i\in \{0,1\} \f$
         *
         * @return reference to element
         */
        T& operator()(size_t i){
        	if(i<2)
        		return _pos[i];
        	return _theta;
        }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         * @return const reference to element
         */
        const T& operator[](size_t i) const {
        	if(i<2)
        		return _pos[i];
        	return _theta;
        }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector \f$i\in \{0,1,2\} \f$
         * @return reference to element
         */
        T& operator[](size_t i) {
        	if(i<2)
        		return _pos[i];
        	return _theta;
        }

        /**
         * @brief The transform corresponding to the pose.
         * @param pose [in] the pose.
         * @return equivalent 2D transform.
         */
		static rw::math::Transform2D<T> transform(const Pose2D<T>& pose){
			return rw::math::Transform2D<T>(
					rw::math::Vector2D<T>(pose.x(), pose.y()),
					rw::math::Rotation2D<T>(pose.theta()));
		}

		/**
		 * @brief Print to standard output.
		 * @param pose [in] the pose to print.
		 */
		static void print(const Pose2D<T>& pose)
		{
		    std::cout
		        << "( x: " << pose.x()
		        << ", y: " << pose.y()
		        << ", th: " << pose.theta()
		        << ")";
		}

		/**
		 * @brief Convert to a Boost vector of (x, y, theta).
		 * @param pose [in] the pose.
		 * @return boost vector.
		 */
		static boost::numeric::ublas::vector<T> toUblas(const Pose2D& pose){
			typedef boost::numeric::ublas::vector<T> Vec;
			    Vec vec(3);
			    vec(0) = pose.x();
			    vec(1) = pose.y();
			    vec(2) = pose.theta();
			    return vec;
		}
	private:
		rw::math::Vector2D<T> _pos;
		T _theta;

	};

	/*@}*/
}} // end namespaces

namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
	/**
	 * @copydoc rw::common::serialization::write
	 * @relatedalso rw::math::Pose2D
	 */
    template<> void write(const rw::math::Pose2D<double>& sobject, rw::common::OutputArchive& oarchive, const std::string& id);

	/**
	 * @copydoc rw::common::serialization::write
	 * @relatedalso rw::math::Pose2D
	 */
    template<> void write(const rw::math::Pose2D<float>& sobject, rw::common::OutputArchive& oarchive, const std::string& id);

	/**
	 * @copydoc rw::common::serialization::read
	 * @relatedalso rw::math::Pose2D
	 */
    template<> void read(rw::math::Pose2D<double>& sobject, rw::common::InputArchive& iarchive, const std::string& id);

	/**
	 * @copydoc rw::common::serialization::read
	 * @relatedalso rw::math::Pose2D
	 */
    template<> void read(rw::math::Pose2D<float>& sobject, rw::common::InputArchive& iarchive, const std::string& id);

}}} // end namespaces

#endif /* POSE2D_HPP_ */
