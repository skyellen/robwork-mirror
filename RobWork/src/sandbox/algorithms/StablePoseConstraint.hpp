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
 
 

#ifndef RW_ALGORITHMS_StablePoseConstraint_HPP
#define RW_ALGORITHMS_StablePoseConstraint_HPP



/**
 * @file StablePoseConstraint.hpp
 */

#include "ConstraintModel.hpp"

#include <rw/geometry/Plane.hpp>



namespace rwlibs { namespace algorithms {



/**
 * @brief A stable pose constraint model.
 * 
 * Describes a stable pose constraint, i.e. a valid pose expected for an object.
 * Current implementation considers a set of orientations obtained from a revolution around
 * a single axis as a stable pose (i.e. allowing 1 DoF freedom of rotation).
 */
class StablePoseConstraint : public ConstraintModel {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<StablePoseConstraint> Ptr;
		
		//! @copydoc ConstraintModel::MinSamples
		static const int MinSamples = 4;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		StablePoseConstraint() {}
		
		//! @brief Destructor.
		virtual ~StablePoseConstraint() {}
		
		//! @brief Constructor.
		StablePoseConstraint(const std::vector<rw::math::Transform3D<> >& data)
		{
			refit(data);
		}

	public: // methods
		//! @copydoc RANSACModel::fitError
		virtual double fitError(rw::math::Transform3D<> sample) const;
		
		/**
		 * @copydoc RANSACModel::invalid
		 *
		 * In the case of a stable pose, a model is invalid, if...
		 */
		virtual bool invalid() const;
		
		//! @copydoc RANSACModel::refit
		virtual double refit(const std::vector<rw::math::Transform3D<> >& samples);
		
		//! @copydoc RANSACModel::getMinReqData
		static int getMinReqData() { return MinSamples; }
		
		//! @copydoc RANSACModel::same
		virtual bool same(const StablePoseConstraint& model, double threshold) const;
	
	protected: // body
		rw::math::Vector3D<> _normal; // normal
		double _dx, _dy, _dz; // x, y, z plane distances respectively
		
		rw::geometry::Plane _xplane;
		rw::geometry::Plane _yplane;
		rw::geometry::Plane _zplane;
};



}} // /namespaces

#endif // include guard
