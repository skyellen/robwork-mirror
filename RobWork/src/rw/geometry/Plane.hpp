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


#ifndef RW_GEOMETRY_PLANE_HPP_
#define RW_GEOMETRY_PLANE_HPP_


#include "Primitive.hpp"

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{

	class Plane: public Primitive {
	public:
		Plane(const rw::math::Q& initQ)
		{}

		Plane(const rw::math::Vector3D<>& n, double d):
			_normal(n),_d(d){}

		Plane(const rw::math::Vector3D<>& p1,
			  const rw::math::Vector3D<>& p2,
			  const rw::math::Vector3D<>& p3):
				  _normal(normalize( cross( p2 - p1, p3 - p1 )) )
		{
			_d = dot(_normal, p1);
		}

		virtual ~Plane(){};

		inline rw::math::Vector3D<>& normal(){return _normal;};

		inline const rw::math::Vector3D<>& normal() const {return _normal;};

		inline double& d(){return _d;};

		inline double d() const {return _d;};



		double refit( std::vector<rw::math::Vector3D<> >& data ){}

		//static Plane fitFrom(const std::vector<rw::math::Vector3D<> >& data){ return };

		// inherited from Primitive
		TriMeshPtr createMesh(int resolution) const { return NULL;} ;

		rw::math::Q getParameters() const;

		GeometryType getType() const{ return PlanePrim; };

	private:
		rw::math::Vector3D<> _normal;
		double _d;
	};
	// @}
} // geometry
} // rw


#endif /* PLANE_HPP_ */
