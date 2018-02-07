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
 
 

#ifndef RW_ALGORITHMS_PlaneModel_HPP
#define RW_ALGORITHMS_PlaneModel_HPP



/**
 * @file PlaneModel.hpp
 */

#include <rw/math/Vector3D.hpp>
#include <rw/geometry/Plane.hpp>

#include "RANSACModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A plane model.
 */
class PlaneModel : public RANSACModel<PlaneModel, rw::math::Vector3D<> >
{
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<PlaneModel> Ptr;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		PlaneModel() {}
		
		/**
		 * @brief Constructor.
		 * 
		 * @param plane [in] plane model
		 */
		PlaneModel(const rw::geometry::Plane& plane) :
			_model(plane)
		{}
		
		//! @brief Destructor.
		virtual ~PlaneModel() {}

	public: // methods
		//! @copydoc RANSACModel::fitError
		virtual double fitError(const rw::math::Vector3D<>& sample) const;
		
		//! @copydoc RANSACModel::invalid
		virtual bool invalid() const;
		
		/**
		 * @copydoc RANSACModel::getMinReqData
		 * 
		 * PlaneModel requires at least 3 samples.
		 */
		virtual int getMinReqData() const { return 3; }
		
		/**
		 * @copydoc RANSACModel::refit
		 * 
		 * Returns standard variance of point distances to the plane model (an average of distances squared).
		 */		
		virtual double refit(const std::vector<rw::math::Vector3D<> >& samples);
		
		//! @copydoc RANSACModel::same
		virtual bool same(const PlaneModel& model, double threshold) const;
		
		//! @brief Get plane.
		rw::geometry::Plane plane() const { return _model; }
		
		//! @brief Get plane normal.
		inline rw::math::Vector3D<> normal() const { return _model.normal(); }
		
		//! @brief Get plane distance from {0, 0, 0} along normal.
		inline double d() const { return -_model.d(); }
		
		/**
		 * @brief Streaming operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const PlaneModel& plane)
		{
			return out << plane._model;
		}
	
	protected: // body
		rw::geometry::Plane _model;
};



}} // /namespaces

#endif // include guard
