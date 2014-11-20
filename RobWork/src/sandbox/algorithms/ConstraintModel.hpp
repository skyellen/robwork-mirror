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

#include <rw/math/Pose6D.hpp>
#include <sandbox/algorithms/RANSACModel.hpp>

#include <iostream>



namespace rwlibs { namespace algorithms {
	
	
	
//! @brief Type for constraint model samples.
typedef rw::math::Pose6D<double> ConstraintSample;



/**
 * @brief An interface for constraint models.
 * 
 * Constraint model describes...
 * ^ TODO: write
 * 
 * @note
 * * there should be a way to sample the model space,
 * * define a wrapper class for sample? or a typedef,
 * * add a model adequacy evaluation method?
 * * should this be made a template? (ie. maybe it could take other types of samples, not only a pose...)
 */
class ConstraintModel : public rw::geometry::RANSACModel<ConstraintSample> {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<ConstraintModel> Ptr;
		
		//! @brief A number of inliers required to make a model.
		static const int MinSamples = 1;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		ConstraintModel() : rw::geometry::RANSACModel<ConstraintSample>() {}
		
		//! @brief Destructor.
		virtual ~ConstraintModel() {}
		
		//! @brief Create a model from a set of samples.
		static ConstraintModel& make(const std::vector<ConstraintSample>& data)
		{
			ConstraintModel::Ptr model = new ConstraintModel(); // ownedPtr?
			
			model->_data = data;
			
			return *model;
		}

	public: // methods		
		/**
		 * @copydoc sandbox::algorithms::RANSACModel::fitError
		 */
		virtual double fitError(ConstraintSample sample) const { return 0.0; }
		
		/**
		 * @copydoc sandbox::algorithms::RANSACModel::invalid
		 */
		virtual bool invalid() const { return false; }
		
		/**
		 * @copydoc sandbox::algorithms::RANSACModel::refit
		 */
		virtual void refit() {}
		
		/**
		 * @brief sandbox::algorithms::RANSACModel::getMinReqData
		 */
		virtual int getMinReqData() const { return MinSamples; }
		
		/**
		 * @brief Updates the model by adding a sample.
		 */
		virtual void update(ConstraintSample sample) { _inliers.push_back(sample); }
		
		/**
		 * @brief Returns the list of inliers.
		 */
		std::vector<ConstraintSample > getInliers() const { return _inliers; }
		
		/**
		 * @brief Check whether a sample belongs to the model.
		 * 
		 * Returns \b true when the sample is within a threshold distance of the model.
		 */
		virtual bool belongsTo(ConstraintSample sample, double threshold) const { return fitError(sample) <= threshold; }
	
	protected: // body
		std::vector<ConstraintSample> _inliers;
		
		// here should be model data
};



}} // /namespaces

#endif // include guard
