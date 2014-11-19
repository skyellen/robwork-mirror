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
 
 

#ifndef RW_ALGORITHMS_FixtureConstraint_HPP
#define RW_ALGORITHMS_FixtureConstraint_HPP



/**
 * @file FixtureConstraint.hpp
 */

#include <rw/math/Vector3D.hpp>

#include "ConstraintModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A fixture constraint model.
 * 
 * Describes a plane constraint, i.e. ... ???
 * ^ TODO: write
 */
class FixtureConstraint {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<FixtureConstraint> Ptr;
		
		//! @copydoc ConstraintModel::MinSamples
		static const int MinSamples = 3; // ??
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		FixtureConstraint() {};
		
		//! @brief Destructor.
		virtual ~FixtureConstraint() {};

	public: // methods
		/**
		 * @copydoc ConstraintModel::belongsTo
		 */
		virtual bool belongsTo(ConstraintSample sample) const;
		
		/**
		 * @copydoc ConstraintModel::fitError
		 * 
		 * Returns the shortest distance between the sample and the model.
		 */
		virtual double fitError(ConstraintSample sample) const;
		
		/**
		 * @copydoc ConstraintModel::update
		 */
		virtual void update(ConstraintSample sample);
	
	protected: // body
		// how to represent a fixture?
};



}} // /namespaces

#endif // include guard
