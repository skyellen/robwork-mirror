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
 
 

#ifndef RW_ALGORITHMS_StablePose1DModel_HPP
#define RW_ALGORITHMS_StablePose1DModel_HPP



/**
 * @file StablePose1DModel.hpp
 */

#include <rw/math/Rotation3D.hpp>

#include "RANSACModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A point model.
 */
class StablePose1DModel : public RANSACModel<StablePose1DModel, rw::math::Rotation3D<> >
{
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<StablePose1DModel> Ptr;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		StablePose1DModel()
		{}
		
		//! @brief Destructor.
		virtual ~StablePose1DModel() {}

	public: // methods
		//! @copydoc RANSACModel::fitError
		virtual double fitError(const rw::math::Rotation3D<>& sample) const;
		
		//! @copydoc RANSACModel::invalid
		virtual bool invalid() const;
		
		//! @copydoc RANSACModel::getMinReqData
		virtual int getMinReqData() const { return 4; }
		
		//! @copydoc RANSACModel::refit
		virtual double refit(const std::vector<rw::math::Rotation3D<> >& samples);
		
		//! @copydoc RANSACModel::same
		virtual bool same(const StablePose1DModel& model, double threshold) const;
		
		//! @brief Get stable pose normal.
		inline rw::math::Vector3D<> normal() const { return _normal; }
		
		//! @brief Get stable pose plane distances.
		inline rw::math::Vector3D<> d() const { return rw::math::Vector3D<>(_dx, _dy, _dz); }
		
		/**
		 * @brief Streaming operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const StablePose1DModel& model)
		{
			return out << "StablePose1D(normal: " << model._normal << ", distances: (" << model._dx << ", " << model._dy << ", " << model._dz << "))";
		}
	
	protected: // body
		rw::math::Vector3D<> _normal;
		double _dx, _dy, _dz;
		
		bool _invalid;
};



}} // /namespaces

#endif // include guard
