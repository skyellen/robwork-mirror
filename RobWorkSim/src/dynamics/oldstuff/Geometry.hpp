/*
 * Geometry.hpp
 *
 *  Created on: 14-07-2008
 *      Author: jimali
 */

#ifndef GEOMETRY_HPP_
#define GEOMETRY_HPP_

#include <rw/math/Transform3D.hpp>
#include <sandbox/trimesh/IndexedTriMesh.hpp>

class Geometry {

public:

    Geometry(
        const std::vector<rw::kinematics::Frame*>& fs,
        const rw::math::Transform3D<>& t,
        rw::geometry::IndexedTriMesh<float> *tri):
            _frames(fs),
            transform(t),
            triMesh(tri)
    {}

    rw::kinematics::Frame *bodyFrame;

    std::vector<rw::kinematics::Frame*> _frames;

    rw::math::Transform3D<> transform;

    rw::geometry::IndexedTriMesh<float> *triMesh;
};

#endif /* GEOMETRY_HPP_ */
