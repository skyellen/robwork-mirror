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

/**
 * @file
 * @copydoc rw::geometry::Plane
 */

#include "Primitive.hpp"

#include <rw/math/Metric.hpp>

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{
	
    /**
     * @brief plane primitive represented in Hessian normal-form: a*nx+b*ny+c*nz+d=0
     */
	class Plane: public Primitive {
	public:
		/**
		 * @brief Smart pointer to Plane
		 */
		typedef rw::common::Ptr<Plane> Ptr;

		//! @brief The value type for a plane (double precision).
		typedef double value_type;
		
		/**
		 * @brief Constructor.
		 * 
		 * Makes a plane on X-Y surface.
		 */
		Plane() :
			_normal(rw::math::Vector3D<>::z()),
			_d(0.0)
		{}

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
		 * @param n [in] normal of plane
		 * @param d [in] distance from plane to (0,0,0) in direction of normal
		 * @return
		 */
		Plane(const rw::math::Vector3D<>& n, double d):
			_normal(n) ,_d(d)
		{}

		/**
		 * @brief constructor - calculates the plane from 3 vertices
		 * @param p1 [in] vertice 1
		 * @param p2 [in] vertice 2
		 * @param p3 [in] vertice 3
		 */
		Plane(const rw::math::Vector3D<>& p1,
			  const rw::math::Vector3D<>& p2,
			  const rw::math::Vector3D<>& p3):
				  _normal(normalize(cross(p2 - p1, p3 - p1)))
		{
			_d = -dot(_normal, p1);
		}

		//! @brief destructor
		virtual ~Plane() {}

		//! @brief get plane normal
		inline rw::math::Vector3D<>& normal(){ return _normal; }

		//! @brief get plane normal
		inline const rw::math::Vector3D<>& normal() const { return _normal; }

		//! @brief get distance to {0,0,0} from plane along normal.
		inline double& d() { return _d; }

		//! @brief get distance to {0,0,0} from plane along normal.
		inline double d() const { return _d; }

		/**
		 * @brief Calculates the shortest distance from point to plane. 
		 *
		 * The distance includes the sign s.t. a negative distance corresponds to \b point
		 * being behind the plane and a positive distance in front of the plane.
		 *
		 * @param point
		 */
		double distance(const rw::math::Vector3D<>& point) const {
		    return dot(point, _normal) + _d;
		}

		/**
		 * @brief Default metric for computing the difference between 2 planes
		 * @param plane [in]
		 */
		double distance(const Plane& plane) const {
			double ang = angle(_normal, plane.normal());
			return (ang + fabs(_d - plane.d())) / 2.0;
		}

		/**
		 * @brief Fit this plane to a set of points.
		 *
		 * A PCA analysis of the points are made and the two axes with
		 * largest extends are used for defining the plane. The error is the
		 * sum of the squared mean of the points to the plane.
		 *
		 * @param data [in] a set of points
		 * @return fitting error
		 */
		double refit(std::vector<rw::math::Vector3D<> >& data);

		/**
		 * @brief Calculates the intersection between the line and plane.
		 *
		 * The line defined by \b p1 and \b p2 is considered infinitely long.
		 * Throws a rw::common::Exception if the line is parallel to the plane.
		 *
		 * @param p1 [in] point 1 on the line
		 * @param p2 [in] point 2 on the line
		 */
		rw::math::Vector3D<> intersection(const rw::math::Vector3D<>& p1, const rw::math::Vector3D<>& p2) const {
			double denominator = dot(_normal, p2-p1);
			
			if (fabs(denominator) < 1e-16) {
				RW_THROW("The specified line is parallel to the plane. Points: " << p1 << "; " << p2 << " Normal = " << _normal);
			}
			double t = (-_d-dot(_normal, p1)) / denominator;
			return p1 + t * (p2 - p1);
		}

		//static Plane fitFrom(const std::vector<rw::math::Vector3D<> >& data){ return };

		// inherited from Primitive
		//! @copydoc Primitive::createMesh()
		TriMesh::Ptr createMesh(int resolution) const ;

		/**
		 * @brief Create a triangle mesh representing the plane.
		 * 
		 * Provides the posibility to specify the size of the plan.
		 * @param resolution [in] Resolution of the mesh (not applicable for a plane)
		 * @param size [in] Size of the plane.
		 */
		TriMesh::Ptr createMesh(int resolution, double size) const;


		//! @copydoc Primitive::getParameters()
		virtual rw::math::Q getParameters() const;
		
		//! @copydoc Primitive::setParameters
		virtual void setParameters(const rw::math::Q& q);

		//! @copydoc Primitive::getType()
		GeometryType getType() const { return PlanePrim; }

		/**
		 * @brief Create a metric that can be used to compare distance between
		 * two planes. The distance between two planes is computed as follows:
		 *
		 * val = 0.5*angle(p1.normal, p2.normal)*angToDistWeight + 0.5*fabs(p1.d-p2.d);
		 * @return
		 */
		static rw::math::Metric<Plane>::Ptr makeMetric(double angToDistWeight=1.0);        
        
        /**
           @brief Streaming operator.
         */
        friend std::ostream& operator<<(std::ostream& out, const Plane& p)
        {
            return out
                << "Plane("
                << "n: " << p.normal() << ", d: " << p.d()
                << ")";
        };
	protected:
        bool doIsInside(const rw::math::Vector3D<>& point){
            // test if point is on the back side of the plane
            return fabs( dot(point, _normal) ) < fabs(_d);
        }

	private:
		rw::math::Vector3D<> _normal;
		double _d;
	};
	
	
	
	/**
	 * @brief A metric for calculating plane-to-plane distance.
	 */
	class PlaneMetric: public rw::math::Metric<Plane> {
	public: // constructors
		/**
		 * @brief Constructor.
		 * @param angToDistWeight [in] weighting of angle compared to linear distance.
		 */
		PlaneMetric(double angToDistWeight = 1.0) :
			_angToDistWeight(angToDistWeight)
		{}
		
	protected:
		//! @brief Calculates distance from the plane to reference plane (X-Y surface)
        double doDistance(const Plane& q) const
        {
			return doDistance(Plane(), q);
		}

		/**
		 * @brief Calculates distance between two planes
		 * 
		 * The distance is calculated according to formula:
		 * 
		 * val = 0.5*angle(p1.normal, p2.normal)*angToDistWeight + 0.5*fabs(p1.d-p2.d);
		 * 
		 * Angle is normalized, so that planes with opposite normals are treated as the same plane.
		 */
        double doDistance(const Plane& a, const Plane& b) const
        {
			
			double ang = rw::math::angle(a.normal(), b.normal());
			double d = b.d();
			
			// if the normal faces the other direction, the real angle is 180 deg - calculated angle
			// and one of the distances is reversed
			if ((rw::math::Pi - ang) < ang) {
				ang = rw::math::Pi - ang;
				d = -d;
			}
			
			//ang = ((rw::math::Pi - ang) < ang) ? (rw::math::Pi - ang) : ang;
			
			
			return 0.5*ang*_angToDistWeight + 0.5*fabs(a.d() - d);
		}

        /**
         * @copydoc rw::math::Metric::size
         * @note this function always return -1.
         */
        int doSize() const { return -1; }

		//! @param weighting of angle compared to linear distance.
		double _angToDistWeight;
	};
	// @}
} // geometry
} // rw


#endif /* PLANE_HPP_ */
