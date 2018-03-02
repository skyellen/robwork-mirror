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


#ifndef RW_TRAJECTORY_PATH_HPP
#define RW_TRAJECTORY_PATH_HPP

/**
   @file Path.hpp
*/

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <vector>
#include "Timed.hpp"

namespace rw { namespace trajectory {

template <class T>
class Path: public std::vector<T> 
{
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<Path> Ptr;

	/**
	 * @brief Default constructor
	 */
	Path() {};

	/**
	 * @brief Constructor adding \b cnt elements. Objects of type T is added using default constructor
	 * @param cnt [in] Number of elements in data structure. 
	 */
	Path(size_t cnt): std::vector<T>(cnt)
	{}

	/**
	 * @brief Constructor adding \b cnt elements with value \b value.
	 * @param cnt [in] Number of elements in data structure.
	 * @param value [in] Values with which to initialize elements.
	 */
	Path(size_t cnt, const T& value): std::vector<T>(cnt, value)
	{}


	/**
	 * @brief Constructs Path and copies elements from \b start to \b end into the path
	 * @param start [in] Start of iterator to input data
	 * @param end [in] End for iterator to input data. 
	 */
	template< typename input_iterator >
	Path( input_iterator start, input_iterator end ): std::vector<T>(start, end) 
	{}

	/**
	 * @brief Construct Path and copies elements from \b v
	 * @param v [in] vector to copy data from
	 */
	Path(const std::vector<T>& v): std::vector<T>(v)
	{}

};

/**
 *  @brief Path of rw::math::Q
 */
typedef Path<rw::math::Q> QPath;



/**
 * @brief Path of rw::math::Vector3D<>
 */
typedef Path<rw::math::Vector3D<> > Vector3DPath;

/**
 * @brief Path of rw::math::Rotation3D<>
 */
typedef Path<rw::math::Rotation3D<> > Rotation3DPath;


/**
 * @brief Path of rw::math::Transform3D<>
 */
typedef Path<rw::math::Transform3D<> > Transform3DPath;


/**
 *  @brief Path of rw::kinematics::State
 */
typedef Path<rw::kinematics::State> StatePath;



/**
   @brief Path of rw::math::Q with associated times
*/
typedef Path<TimedQ> TimedQPath;



/**
   @brief Path of rw::kinematics::State with associated times
*/
typedef Path<TimedState> TimedStatePath;

}} // end namespaces

#endif // end include guard
