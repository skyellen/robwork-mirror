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
 
 

#ifndef RW_ALGORITHMS_StablePoseConstraint_HPP
#define RW_ALGORITHMS_StablePoseConstraint_HPP



/**
 * @file StablePoseConstraint.hpp
 */

#include <rw/math/Vector3D.hpp>

#include "ConstraintModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A stable pose constraint model.
 * 
 * Describes a stable pose constraint, i.e. an orientation of the object.
 */
class StablePoseConstraint : public ConstraintModel {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<StablePoseConstraint> Ptr;
		
		//! @copydoc ConstraintModel::MinSamples
		static const int MinSamples = 3;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		StablePoseConstraint() {};
		
		//! @brief Destructor.
		virtual ~StablePoseConstraint() {};

	public: // methods
		//! @copydoc sandbox::algorithms::RANSACModel::fitError
		virtual double fitError(ConstraintSample sample) const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::invalid
		virtual bool invalid() const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::refit
		virtual void refit() const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::getMinReqData
		virtual int getMinReqData() const { return MinSamples; }
		
		//! @copydoc ConstraintModel::update
		virtual void update(ConstraintSample sample);
	
	protected: // body
		// how to represent an orientation?
};



}} // /namespaces

#endif // include guard
