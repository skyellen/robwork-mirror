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
 
 

#ifndef RW_ALGORITHMS_PlaneConstraint_HPP
#define RW_ALGORITHMS_PlaneConstraint_HPP



/**
 * @file PlaneConstraint.hpp
 */

#include <rw/math/Vector3D.hpp>
#include <rw/geometry/Plane.hpp>

#include "ConstraintModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A plane constraint model.
 * 
 * Describes a plane constraint, i.e. a surface.
 */
class PlaneConstraint : public ConstraintModel {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<PlaneConstraint> Ptr;
		
		//! @copydoc ConstraintModel::MinSamples
		static const int MinSamples = 3; // ??
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		PlaneConstraint() :
			_model(rw::math::Vector3D<>(), rw::math::Vector3D<>::x(), rw::math::Vector3D<>::y())
		{};
		
		//! @brief Destructor.
		virtual ~PlaneConstraint() {};
		
		//! @brief Create a model from a set of samples.
		static PlaneConstraint& make(const std::vector<rw::math::Transform3D<> >& data)
		{
			PlaneConstraint::Ptr model = new PlaneConstraint();
			
			model->_data = data;
			model->refit(data);
			
			return *model;
		}

	public: // methods
		//! @copydoc sandbox::algorithms::RANSACModel::fitError
		virtual double fitError(rw::math::Transform3D<> sample) const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::invalid
		virtual bool invalid() const;
		
		//! @copydoc sandbox::algorithms::RANSACModel::refit
		virtual double refit(const std::vector<rw::math::Transform3D<> >& samples);
		
		//! @copydoc sandbox::algorithms::RANSACModel::getMinReqData
		static int getMinReqData() { return MinSamples; }
		
		//! @copydoc sandbox::algorithms::RANSACModel::same
		virtual bool same(const PlaneConstraint& model, double threshold) const;
		
		/**
		 * @brief Streaming operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const PlaneConstraint& plane)
		{
			return out << plane._model;
		}
	
	protected: // body
		rw::geometry::Plane _model;
};



}} // /namespaces

#endif // include guard
