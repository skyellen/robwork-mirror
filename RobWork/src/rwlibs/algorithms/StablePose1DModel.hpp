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
 * @brief A stable pose with 1 DoF model.
 * 
 * Models a stable pose with 1 DoF, i.e. a set of rotations allowed for object with 1 axis of rotation possible (e.g. a box on a surface).
 * For a case of coordinate frame rotating around single arbitrary axis, the points at the ends of x, y, and z versors fall into parallel planes on a unit sphere.
 * A stable pose with 1 DoF freedom is represented by a normal vector for the planes, and distances Dx, Dy, and Dz to respective planes, measured from the point {0, 0, 0}.
 * For some cases, only two planes are defined, omitting the third coordinate (NaN). If only one plane was found, the model is considered invalid.
 * 
 * At least 4 samples are required for this model.
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
		StablePose1DModel() :
			_normal(rw::math::Vector3D<>::z()),
			_dx(0.0),
			_dy(0.0),
			_dz(1.0)
		{}
		
		/**
		 * @brief Constructor.
		 * 
		 * @param normal [in] normal for x, y and z planes
		 * @param distances [in] Dx, Dy and Dz distances for the respective planes
		 */
		StablePose1DModel(const rw::math::Vector3D<>& normal, const rw::math::Vector3D<> distances) :
			_normal(normalize(normal)),
			_dx(distances[0]),
			_dy(distances[1]),
			_dz(distances[2])
		{}
		
		/**
		 * @brief Constructs stable pose from an axis of rotation described in the object frame of reference, and a placement of that axis in global frame of reference.
		 * 
		 * @param local [in] rotation axis in the body frame of reference
		 * @param global [in] location of the axis of rotation in the world coordinate frame
		 */
		static StablePose1DModel fromAxes(const rw::math::Vector3D<>& local, const rw::math::Vector3D<>& global) {
			rw::math::Vector3D<> normal = normalize(global);
			rw::math::Vector3D<> loc = normalize(local);
			
			double dx = dot(rw::math::Vector3D<>::x(), loc);
			double dy = dot(rw::math::Vector3D<>::y(), loc);
			double dz = dot(rw::math::Vector3D<>::z(), loc);
			
			return StablePose1DModel(normal, rw::math::Vector3D<>(dx, dy, dz));
		}
		
		//! @brief Destructor.
		virtual ~StablePose1DModel() {}

	public: // methods
		/**
		 * @copydoc RANSACModel::fitError
		 * 
		 * Fit error for the stable pose is calculated as norm2 of distances of its x, y and z versor end points to respective planes.
		 * In case the particular plane was not found (i.e. Di = NaN), distance to this plane is assumed to be 0.
		 */
		virtual double fitError(const rw::math::Rotation3D<>& sample) const;
		
		//! @copydoc RANSACModel::invalid
		virtual bool invalid() const;
		
		/**
		 * @copydoc RANSACModel::getMinReqData
		 * 
		 * StablePose1DModel requires at least 4 samples.
		 */
		virtual int getMinReqData() const { return 3; }
		
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
