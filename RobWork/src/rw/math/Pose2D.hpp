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
		Pose2D() :
			_pos(0,0),
			_theta(0)
		{}

		Pose2D(rw::math::Vector2D<T> pos, T theta) :
			_pos(pos),
			_theta(theta)
		{}

		Pose2D(T x, T y, T theta) :
			_pos(x,y), _theta(theta)
		{}

		Pose2D(const rw::math::Transform2D<T>& transform) :
			_pos(transform.P()),
			_theta(
				// Sigh.
				atan2(
					transform.R()(1, 0),
					transform.R()(0, 0)))
		{}

		T& x(){return _pos[0];}
		T& y(){return _pos[1];}
		T& theta(){return _theta;}
		rw::math::Vector2D<T>& getPos(){return _pos;}

		T x() const { return _pos[0]; }
		T y() const { return _pos[1]; }
		T theta() const { return _theta; }
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


		// The transform corresponding to the pose.
		static
		rw::math::Transform2D<T> transform(const Pose2D<T>& pose){
			return rw::math::Transform2D<T>(
					rw::math::Vector2D<T>(pose.x(), pose.y()),
					rw::math::Rotation2D<T>(pose.theta()));
		}

		static
		void print(const Pose2D<T>& pose)
		{
		    std::cout
		        << "( x: " << pose.x()
		        << ", y: " << pose.y()
		        << ", th: " << pose.theta()
		        << ")";
		}


		// A vector of (x, y, theta).
		static
		boost::numeric::ublas::vector<T> toUblas(const Pose2D& pose){
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

    template<> void write(const rw::math::Pose2D<double>& tmp, rw::common::OutputArchive& oar, const std::string& id);
    template<> void write(const rw::math::Pose2D<float>& tmp, rw::common::OutputArchive& oar, const std::string& id);

    template<> void read(rw::math::Pose2D<double>& tmp, rw::common::InputArchive& iar, const std::string& id);
    template<> void read(rw::math::Pose2D<float>& tmp, rw::common::InputArchive& iar, const std::string& id);

}}} // end namespaces

#endif /* POSE2D_HPP_ */
