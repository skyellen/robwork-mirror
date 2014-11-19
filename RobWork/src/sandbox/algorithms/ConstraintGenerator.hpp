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
 
 

#ifndef RW_ALGORITHMS_CONSTRAINTGENERATOR_HPP
#define RW_ALGORITHMS_CONSTRAINTGENERATOR_HPP



/**
 * @file ConstraintGenerator.hpp
 */

#include <rw/math/Pose6D.hpp>

#include <iostream>

#include "ConstraintModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief Generates a set of algorithms deduced from provided samples.
 * 
 * Constraint generator...
 * ^ TODO: write
 * 
 * @note
 * * what kind of input should it take besides samples? metric, ...
 */
class ConstraintGenerator {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<ConstraintGenerator> Ptr;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		ConstraintGenerator() {};
		
		//! @brief Destructor.
		virtual ~ConstraintGenerator() {};

	public: // methods
		/**
		 * @brief Add a sample.
		 */
		virtual void add(ConstraintSample sample);
		
		/**
		 * @brief Add samples.
		 */
		virtual void add(const std::vector<ConstraintSample >& samples);
		
		/**
		 * @brief Returns samples.
		 * ^ TODO: make it a reference?
		 */
		std::vector<ConstraintSample > getSamples() const { return _samples; }
		
		/**
		 * @brief Recalculates the algorithms.
		 * ^ TODO: should it return new algorithms at the same time?
		 */
		virtual bool recalculate();
		
		/**
		 * @brief Returns a vector of algorithms found.
		 * ^ TODO:
		 * * shall this be an ordered list? (hierarchy/accuracy/...)
		 * * shall it recalculate algorithms if not yet called?
		 */
		std::vector<ConstraintModel> getConstraints() const { return _algorithms; }
	
	private: // body
		std::vector<ConstraintSample> _samples;
		std::vector<ConstraintModel> _algorithms;
};



}} // /namespaces

#endif // include guard
