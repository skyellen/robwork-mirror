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
 
 

#ifndef RW_ALGORITHMS_StablePose0DModel_HPP
#define RW_ALGORITHMS_StablePose0DModel_HPP



/**
 * @file StablePose0DModel.hpp
 */

#include <rw/math/Rotation3D.hpp>

#include "RANSACModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A stable pose with 0 DoF model.
 * o
 * Models a stable pose with 0 DoF, i.e. essentially a single orientation in SE3.
 * 
 * The model requires at least 1 sample.
 */
class StablePose0DModel : public RANSACModel<StablePose0DModel, rw::math::Rotation3D<> >
{
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<StablePose0DModel> Ptr;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		StablePose0DModel()	{}
		
		/**
		 * @brief Constructor.
		 * 
		 * @param orientation [in] orientation for the model
		 */
		StablePose0DModel(const rw::math::Rotation3D<>& orientation) :
			_rot(orientation) // constructs internal quaternion representation using rotation
		{}
		
		//! @brief Destructor.
		virtual ~StablePose0DModel() {}

	public: // methods
		/**
		 * @copydoc RANSACModel::fitError
		 */
		virtual double fitError(const rw::math::Rotation3D<>& sample) const;
		
		//! @copydoc RANSACModel::invalid
		virtual bool invalid() const;
		
		/**
		 * @copydoc RANSACModel::getMinReqData
		 * 
		 * StablePose0DModel requires at least 1 sample.
		 */
		virtual int getMinReqData() const { return 1; }
		
		//! @copydoc RANSACModel::refit
		virtual double refit(const std::vector<rw::math::Rotation3D<> >& samples);
		
		//! @copydoc RANSACModel::same
		virtual bool same(const StablePose0DModel& model, double threshold) const;
		
		//! @brief Get stable pose orientation.
		inline rw::math::Rotation3D<> orientation() const { return _rot; }
		
		/**
		 * @brief Streaming operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const StablePose0DModel& model)
		{
			return out << "StablePose0D(Rotation3D: " << model._rot << ")";
		}
	
	protected: // body
		rw::math::Rotation3D<> _rot;
};



}} // /namespaces

#endif // include guard
