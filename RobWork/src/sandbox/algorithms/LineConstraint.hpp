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
 
 

#ifndef RW_ALGORITHMS_LINECONSTRAINT_HPP
#define RW_ALGORITHMS_LINECONSTRAINT_HPP



/**
 * @file LineConstraint.hpp
 */

#include <rw/math/Vector3D.hpp>

#include "ConstraintModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A line constraint model.
 * 
 * Describes a line constraint, i.e. an edge in Cartesian coordinates.
 */
class LineConstraint : public ConstraintModel {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<LineConstraint> Ptr;
		
		//! @copydoc ConstraintModel::MinSamples
		static const int MinSamples = 2;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		LineConstraint() : ConstraintModel() {}
		
		//! @brief Destructor.
		virtual ~LineConstraint() {}

	public: // methods
		//! @copydoc sandbox::algorithms::RANSACModel::fitError
		virtual double fitError(ConstraintSample sample) const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::invalid
		virtual bool invalid() const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::refit
		virtual void refit();
		
		//! @copydoc sandbox::algorithms::RANSACModel::getMinReqData
		virtual int getMinReqData() const { return MinSamples; }
		
		//! @copydoc ConstraintModel::update
		virtual void update(ConstraintSample sample);
	
	protected: // body
		// how to represent a line? -- there is a Line2D class, but no 3D model...; implement this?
};



}} // /namespaces

#endif // include guard
