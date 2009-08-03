#ifndef RW_GEOMETRY_GEOMETRYUTIL_HPP_
#define RW_GEOMETRY_GEOMETRYUTIL_HPP_

#include <vector>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/InertiaMatrix.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/geometry/Face.hpp>

#include <sandbox/geometry/Geometry.hpp>

namespace rw { namespace geometry {
namespace sandbox {

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
        estimateInertia(
            double mass,
			rw::kinematics::Frame &frame,
			const rw::kinematics::State& state);

	/**
	 * @brief estimates the inertia of a Face soup. The inertia is described relative
	 * to the coordinate system that the faces are described, but around center of mass
	 */
	static rw::math::InertiaMatrix<>
        estimateInertia(
            double mass,
			const std::vector<rw::geometry::Face<float> >& faces,
			const rw::math::Transform3D<>& t3d);

   /**
     * @brief estimates the inertia of a list of geometries.
     * The inertia is described relative
     * to the ref coordinate system. Each geometry is assumed to be described relative to
     * ref.
     * @note the inertia is NOT described around center of gravity, but around ref
     */
   static rw::math::InertiaMatrix<>
        estimateInertia(
            double mass,
             const std::vector<Geometry*> &geoms,
            const rw::math::Transform3D<>& ref = rw::math::Transform3D<>::identity() );

   /**
     * @brief estimates the inertia and center of gravity (COG) of a list of geometries.
     * The inertia is described relative
     * to the ref coordinate system translated to COG. Each geometry is assumed to
     * be described relative to ref.
     */
   static std::pair<rw::math::Vector3D<>, rw::math::InertiaMatrix<> >
        estimateInertiaCOG(
            double mass,
            const std::vector<Geometry*> &geoms,
            const rw::math::Transform3D<>& ref = rw::math::Transform3D<>::identity() );

   /**
     * @brief estimates the center of gravity (COG) of a list of geometries.
     */
   static rw::math::Vector3D<>
        estimateCOG(const std::vector<Geometry*> &geoms);

    /**
      * @brief estimates the center of gravity (COG) of a list of geometries.
      */
    static double
         calcMaxDist(const std::vector<Geometry*> &geoms, const rw::math::Vector3D<> center);



	/**
	 * @brief util function that locates all frames that is staticly connected to f
	 * and that has geometry information.
	 */
	static std::vector<rw::kinematics::Frame*>
		getAnchoredFrames(rw::kinematics::Frame &f,
						  const rw::kinematics::State &state);

	/**
	 * @brief util function that locates all frames in the sub tree of parent
	 * that is staticly connected and that has geometry information.
	 */
	static std::vector<rw::kinematics::Frame*>
		getAnchoredChildFrames(rw::kinematics::Frame *parent, const rw::kinematics::State &state);

};

}
}}
#endif /*DYNAMICUTIL_HPP_*/
