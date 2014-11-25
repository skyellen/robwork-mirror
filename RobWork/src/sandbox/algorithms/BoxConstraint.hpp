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
 
 

#ifndef RW_ALGORITHMS_BoxConstraint_HPP
#define RW_ALGORITHMS_BoxConstraint_HPP



/**
 * @file BoxConstraint.hpp
 */

#include <rw/math/Vector3D.hpp>

#include "ConstraintModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A box constraint model.
 * 
 * Describes a box constraint, i.e. a volume in space.
 */
class BoxConstraint {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<BoxConstraint> Ptr;
		
		//! @copydoc ConstraintModel::MinSamples
		static const int MinSamples = 1; // ??
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		BoxConstraint() {};
		
		//! @brief Destructor.
		virtual ~BoxConstraint() {};

	public: // methods
		//! @copydoc sandbox::algorithms::RANSACModel::fitError
		virtual double fitError(rw::math::Transform3D<> sample) const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::invalid
		virtual bool invalid() const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::refit
		virtual void refit() const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::getMinReqData
		virtual int getMinReqData() const { return MinSamples; }
		
		//! @copydoc ConstraintModel::update
		virtual void update(rw::math::Transform3D<> sample);
	
	protected: // body
		// how to represent a box?
};



}} // /namespaces

#endif // include guard
