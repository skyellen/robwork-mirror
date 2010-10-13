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

    /**
     * @brief plane primitive
     */
	class Plane: public Primitive {
	public:
	    /**
	     * @brief constructor
	     * @param q
	     * @return
	     */
		Plane(const rw::math::Q& q)
		{
		    _normal(0) = q(0);
		    _normal(1) = q(1);
		    _normal(2) = q(2);
		    _d = q(3);
		}

		/**
		 * @brief constructor
		 * @param n
		 * @param d
		 * @return
		 */
		Plane(const rw::math::Vector3D<>& n, double d):
			_normal(n),_d(d){}

		Plane(const rw::math::Vector3D<>& p1,
			  const rw::math::Vector3D<>& p2,
			  const rw::math::Vector3D<>& p3):
				  _normal(normalize( cross( p2 - p1, p3 - p1 )) )
		{
			_d = dot(_normal, p1);
		}

		//! @brief destructor
		virtual ~Plane(){};

		//! @brief get plane normal
		inline rw::math::Vector3D<>& normal(){return _normal;};

		//! @brief get plane normal
		inline const rw::math::Vector3D<>& normal() const {return _normal;};

		//! @brief get distance to {0,0,0} from plane along normal.
		inline double& d(){return _d;};

		//! @brief get distance to {0,0,0} from plane along normal.
		inline double d() const {return _d;};

		/**
		 * @brief fit this plane to a set of points
		 * @param data [in] a set of points
		 * @return
		 */
		double refit( std::vector<rw::math::Vector3D<> >& data ){ return 0;}

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
