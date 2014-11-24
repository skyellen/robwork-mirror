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
 
 

#ifndef RWLIBS_ALGORITHMS_RANSACMODEL_HPP
#define RWLIBS_ALGORITHMS_RANSACMODEL_HPP



/**
 * @file RansacModel.hpp
 */

#include <rw/common/Ptr.hpp>

#include <iostream>
#include <vector>



namespace rwlibs { namespace algorithms {



/**
 * @brief An interface for RANSAC model fitting.
 */
template <class T>
class RANSACModel {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<RANSACModel<T> > Ptr;
		
	public: // constructors
		//! @brief Constructor.
		RANSACModel() {}
		
		//! @brief Destructor.
		virtual ~RANSACModel() {}
		
		//! @brief Create a model from a set of samples.
		static RANSACModel<T>& make(const std::vector<T>& data)
		{
			RANSACModel<T>::Ptr model = new RANSACModel<T>();
			model->_data = data;
			return *model;
		}

	public: // methods		
		/**
		 * @brief Returns fitting error of the sample.
		 * ^ TODO: needs metric? -- perhaps metric should be a part of the model
		 */
		virtual double fitError(T sample) const { return 0.0; }
		
		/**
		 * @brief Checks whether a model is invalid.
		 */
		virtual bool invalid() const { return false; }
		
		/**
		 * @brief Recalculates the model based on samples it holds.
		 */
		virtual void refit() {}
		
		/**
		 * @brief Returns the number of samples required to create a model.
		 */
		virtual int getMinReqData() const { return 0; }
		
		/**
		 * @brief Tests whether a model is same within a threshold of another model.
		 */
		virtual bool same(const RANSACModel& model, double threshold) const { return true; }
		
		//! @brief Get the model quality.
		double getQuality() const { return _quality; }
		
		//! @brief Set the model quality.
		void setQuality(double quality) { _quality = quality; }
		
		//! @brief Access data.
		std::vector<T>& getData() { return _data; }
	
	protected: // body
		std::vector<T> _data;
		
		double _quality;
};



}} // /namespaces

#endif // include guard
