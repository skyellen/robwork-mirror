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

#ifndef RW_GEOMETRY_GEOMETRYUTIL_HPP_
#define RW_GEOMETRY_GEOMETRYUTIL_HPP_

#include <vector>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/InertiaMatrix.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VectorND.hpp>
//#include <rw/geometry/Face.hpp>

#include "Geometry.hpp"
#include "Primitive.hpp"
#include "TriMesh.hpp"

namespace rw {
namespace geometry {
/** @addtogroup geometry */
/*@{*/
/**
 * @brief Utility functions for calculating properties of geometry
 */
class GeometryUtil
{
public:

    /**
     * @brief estimates the inertia and center of mass for a group of frames that is anchored
     * to the reference frame. the inertia will be described relative to the frame
     */
    static std::pair<rw::math::Vector3D<>, rw::math::InertiaMatrix<> >
            estimateInertia(double mass, rw::kinematics::Frame &frame, const rw::kinematics::State& state);

    /**
     * @brief estimates the inertia of a list of geometries.
     * The inertia is described relative
     * to the ref coordinate system. Each geometry is assumed to be described relative to
     * ref.
     * @note the inertia is NOT described around center of gravity, but around ref
     */
	static rw::math::InertiaMatrix<> estimateInertia(double mass, const std::vector<Geometry::Ptr> &geoms,
	                                                 rw::kinematics::Frame* ref, const rw::kinematics::State& state,
                                                     const rw::math::Transform3D<>& reftrans =
                                                             rw::math::Transform3D<>::identity());

    /**
     * @brief estimates the inertia and center of gravity (COG) of a list of geometries.
     * The inertia is described relative
     * to the ref coordinate system translated to COG. Each geometry is assumed to
     * be described relative to ref.
     */
    static std::pair<rw::math::Vector3D<>, rw::math::InertiaMatrix<> > estimateInertiaCOG(
    		double mass,
    		const std::vector<Geometry::Ptr> &geoms,
    		rw::kinematics::Frame* ref,
    		const rw::kinematics::State& state,
    		const rw::math::Transform3D<>& reftrans = rw::math::Transform3D<>::identity());

    /**
      * @brief estimates the inertia and center of gravity (COG) of a list of geometries.
      * The inertia is described relative
      * to the ref coordinate system translated to COG. Each geometry is assumed to
      * be described relative to ref.
      */
     static rw::math::InertiaMatrix<> estimateInertia(
             double mass,
             const std::vector<Geometry::Ptr> &geoms,
             const rw::math::Transform3D<>& reftrans = rw::math::Transform3D<>::identity());


    /**
     * @brief estimates the center of gravity (COG) of a list of geometries.
     * @param geoms [in] the geometries
     * @return the center of gravity of the geometries
     */
    static rw::math::Vector3D<> estimateCOG(const std::vector<Geometry::Ptr> &geoms);

    /**
     * @brief estimates the center of gravity (COG) of a list of geometries.
     * @param geoms [in] the geometries
     * @return the center of gravity of the geometries
     */
	static rw::math::Vector3D<> estimateCOG(const std::vector<Geometry::Ptr> &geoms, rw::kinematics::Frame* ref, const rw::kinematics::State& state);

    /**
     * @brief estimates the center of gravity (COG) of a triangle mesh \b trimesh
     * @param trimesh [in] the triangle mesh
     * @return the center of gravity of the mesh
     */
    static rw::math::Vector3D<> estimateCOG(const TriMesh& trimesh);

    /**
      * @brief calculates the max distance to any triangle in the geoms, from some point \b center
      * @param geoms [in] the geometries containing the triangles
      * @param center [in] the point to calculate the distance from
      * @return the maximum distance to any triangle in the geometries
      */
    static double calcMaxDist(const std::vector<Geometry::Ptr> &geoms, const rw::math::Vector3D<> center, rw::kinematics::Frame* ref, const rw::kinematics::State& state);

    /**
     * @brief estimates center of gravity (COG) of a single tirangle mesh
     * @param trimesh [in] triangle mesh
     * @param t3d [in] transform
     * @return center of gravity
     */
    static rw::math::Vector3D<> estimateCOG(const TriMesh& trimesh, const rw::math::Transform3D<>& t3d);

    /**
     * @brief util function that locates all frames that is staticly connected to f
     * and that has geometry information.
     */
    static std::vector<rw::kinematics::Frame*> getAnchoredFrames(rw::kinematics::Frame &f,
                                                                 const rw::kinematics::State &state);

    /**
     * @brief util function that locates all frames in the sub tree of parent
     * that is staticly connected and that has geometry information.
     */
    static std::vector<rw::kinematics::Frame*> getAnchoredChildFrames(rw::kinematics::Frame *parent,
                                                                      const rw::kinematics::State &state);
                                                                      
    /**
     * @brief calculates volume of k-simplex
     * 
     * Volume of k-dimensional simplex (triangle is 2-simplex) is calculated as:
     * \f$ V(S) = \frac{1}{k!} \sqrt{W W^T}\f$, where:
     * 
     * \f$ W \f$ is a matrix consisting of rows
     * 
     \f$w_i = [v_{i,1}-v_{0,1}\:v_{i,2}-v_{0, 2}\:\cdots\:v_{i,k+1}-v_{0,k+1}] \f$
     
     * of i-th vertex coordinates.
     * (taken from: http://www.math.niu.edu/~rusin/known-math/97/volumes.polyh)
     */
    template<std::size_t N>
    static double simplexVolume(const std::vector<rw::math::VectorND<N> >& vertices) {
		RW_ASSERT(vertices.size() > 0);
		double volume = 0.0;
		
		// construct W matrix
		Eigen::Matrix<double, N-1, N> W;
		for (int idx = 1; idx < vertices.size(); ++idx) {
			// vector w_i
			Eigen::Matrix<double, 1, N> wi = (vertices[idx].e() - vertices[0].e()).transpose();
			//std::cout << wi << std::endl;
			W.row(idx-1) = wi;
		}
		//std::cout << W << std::endl;
		//std::cout << sqrt((W*W.transpose()).determinant()) << std::endl;
		
		// calculate volume
		volume = 1.0/rw::math::Math::factorial(N-1) * sqrt((W*W.transpose()).determinant()); // this sometimes gives NaN
		// even if matrix has a determinant...
		
		// now, for a bit of wishful thinking:
		if (std::isnan(volume)) {
			volume = 0.0;
		}
		
		return volume;
	}
};
//! @}
}
}
#endif /*DYNAMICUTIL_HPP_*/
