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
 
 

#ifndef RW_ALGORITHMS_PointModel_HPP
#define RW_ALGORITHMS_PointModel_HPP



/**
 * @file PointModel.hpp
 */

#include <rw/math/Vector3D.hpp>

#include "RANSACModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A point model.
 */
class PointModel : public RANSACModel<PointModel, rw::math::Vector3D<> >
{
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<PointModel> Ptr;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		PointModel() {}
		
		/**
		 * @brief Constructor.
		 * 
		 * @param point [in] position
		 */
		PointModel(const rw::math::Vector3D<>& point) :
			_model(point)
		{}
		
		//! @brief Destructor.
		virtual ~PointModel() {}

	public: // methods
		//! @copydoc RANSACModel::fitError
		virtual double fitError(const rw::math::Vector3D<>& sample) const;
		
		//! @copydoc RANSACModel::invalid
		virtual bool invalid() const;
		
		/**
		 * @copydoc RANSACModel::getMinReqData
		 * 
		 * PointModel requires at least 1 sample.
		 */
		virtual int getMinReqData() const { return 1; }
		
		//! @copydoc RANSACModel::refit
		virtual double refit(const std::vector<rw::math::Vector3D<> >& samples);
		
		//! @copydoc RANSACModel::same
		virtual bool same(const PointModel& model, double threshold) const;
		
		//! @brief Get point position.
		inline rw::math::Vector3D<> p() const { return _model; }
		
		/**
		 * @brief Streaming operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const PointModel& model)
		{
			return out << model._model;
		}
	
	protected: // body
		rw::math::Vector3D<> _model;
};



}} // /namespaces

#endif // include guard
