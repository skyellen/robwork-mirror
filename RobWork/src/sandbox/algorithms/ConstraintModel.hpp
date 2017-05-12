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
 
 

#ifndef RW_ALGORITHMS_CONSTRAINTMODEL_HPP
#define RW_ALGORITHMS_CONSTRAINTMODEL_HPP



/**
 * @file ConstraintModel.hpp
 */

#include <rw/math/Transform3D.hpp>
#include <rwlibs/algorithms/RANSACModel.hpp>

#include <iostream>



namespace rwlibs { namespace algorithms {

/**
 * @brief An interface for constraint models.
 * 
 * Constraint model describes...
 * 
 * @note You can't extend the base class with purely abstract virtual functions... Because it needs to be possible to instantiate. D'oh!
 * @note Do i even need this class though?
 */
class ConstraintModel : public RANSACModel<ConstraintModel, rw::math::Transform3D<> > {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<ConstraintModel> Ptr;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		ConstraintModel() {}
		
		//! @brief Destructor.
		virtual ~ConstraintModel() {}

	public: // methods
		//! @copydoc RANSACModel::fitError
		virtual double fitError(const rw::math::Transform3D<>& sample) const { return 0.0; }
		
		//! @copydoc RANSACModel::invalid
		virtual bool invalid() const { return true; }
		
		//! @copydoc RANSACModel::refit
		virtual double refit(const std::vector<rw::math::Transform3D<> >& data) { return 0.0; }
		
		//! @copydoc RANSACModel::getMinReqData
		virtual int getMinReqData() const { return 0; }
		
		//! @copydoc RANSACModel::same
		virtual bool same(const ConstraintModel& model, double threshold) const { return false; }
		
		/**
		 * @brief Updates the model by adding a sample.
		 * TODO: should it check if the sample belongs to a model?
		 */
		//virtual void update(rw::math::Transform3D<> sample) = 0;
		
		/**
		 * @brief Updates the model by adding a vector of samples.
		 * TODO: should it check iif the sampel belongs to a model?
		 */
		//virtual void update(const std::vector<rw::math::Transform3D<> >& samples) = 0;		
	
	protected: // body
};



}} // /namespaces

#endif // include guard
