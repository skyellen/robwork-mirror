
#ifndef RW_GEOMETRY_TRIMESHSURFACESAMPLER_HPP_
#define RW_GEOMETRY_TRIMESHSURFACESAMPLER_HPP_

#include <rw/geometry/Geometry.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/geometry/TriMesh.hpp>

namespace rw {
namespace geometry {


/**
 * @brief random sampling of points and orientations close to the surface of a geometry.
 *
 * A point \b p on the surface is randomly choosen.<br>
 * A rotation \b rot is randomly generated.<br>
 * A random distance \b d in the interval [minD, maxD] is generated<br>
 * The position \b pos is calculated as \f$ pos = p - rot*(0,0,d)^T \f$ <br>
 * The random pose \b X is thus \f$ X = (pos, rot) \f$
 * Optionally a random rotation of \b X can be generated such that z-axis of rot is
 * not allways pointing toward the surface.
 */
class TriMeshSurfaceSampler {
public:
    typedef rw::common::Ptr<TriMeshSurfaceSampler> Ptr;

    //! constructor
    TriMeshSurfaceSampler();

    /**
     * @brief constructor
     * @param geom [in] geometry representing the surface that should be sampled.
     */
    TriMeshSurfaceSampler(rw::geometry::Geometry::Ptr geom);

    /**
     * @brief constructor
     * @param geoms [in] list of geometries representing the surface that should be sampled.
     */
    TriMeshSurfaceSampler(const std::vector<rw::geometry::Geometry::Ptr>& geoms);

    //! destructor
    virtual ~TriMeshSurfaceSampler();

    /**
     * @brief add surface representing geometry to this sampler
     * @param geom [in] geometry representing surface to be sampled
     */
    void add(rw::geometry::Geometry::Ptr geom);

    /**
     * @brief add surface mesh.
     * @param mesh [in] mesh to add to sampler.
     */
    void add(rw::geometry::TriMesh::Ptr mesh);

    /**
     * @brief sample a pose on the surface of the object.
     * @return
     */
    rw::math::Transform3D<> sample();

    /**
     * @brief sample point on surface of object
     * @param point_dst [out] point on surface
     */
    rw::math::Vector3D<> samplePoint();

    ///// configuration options

    /**
     * @brief set the bounds to which the random position around the sampled surface
     * point will be generated. See setRandomPositionEnabled
     * @param minD [in]  minimum distance
     * @param maxD [in] maximum distance.
     */
    void setBoundsD(double minD, double maxD);

    /**
     * @brief enable the generation of a random rotation instead of a rotation with
     * the z-axis pointing in the surface normal direction.
     * @param enabled [in]
     */
    void setRandomRotationEnabled(bool enabled);

    /**
     * @brief enables generation of a random position within the sampled surface position.
     * The local point will be generated within a min and max distance from the sampled
     * surface point. See setBoundsD
     * @param enabled [in] true to enable local random generation around sampled surface point. False otherwise.
     */
    void setRandomPositionEnabled(bool enabled);

    /**
     * @brief enable z-axis direction filtering. See setZAxisDirection for information
     * on setting the direction.
     *
     * @param enabled
     */
    void setZAxisDirectionEnabled(bool enabled);

    /**
     * @brief sets the direction that the z-axis must point into. this is equal
     * to testing if the z-axis of the generated pose lies on the right side of
     * the plane defined by the plane normal dir and (0,0,0).
     *
     * Setting this also sets setZAxisDirectionEnabled( true )
     *
     * @param dir [in]
     */
    void setZAxisDirection(const rw::math::Vector3D<>& dir);

    /**
     * @brief return the mesh that is being sampled
     * @return mesh
     */
    rw::geometry::TriMesh::Ptr getMesh();

private:

    //! recursive search for finding value in list
    int binSearchRec(const double value, size_t start, size_t end){
        if(start==end)
            return (int)start;
        // choose a int between start and end
        size_t split = (end-start)/2+start;
        if(value<_surfaceArea[split])
            return binSearchRec(value, start, split);
        else
            return binSearchRec(value, split+1, end);
    }

    //! recursive search for finding value in list
    rw::geometry::Triangle<> getTriangle(const int value);


private:
    double _sAreaSum, _minD, _maxD;
    bool _genRandomRotation, _filterByDirection, _genRandomPostion;
    std::vector<double> _surfaceArea;
    rw::geometry::TriMesh::Ptr _mesh;

    std::vector<rw::geometry::TriMesh::Ptr> _meshes;
    std::vector<int> _surfaceAreaMesh;

    rw::math::Vector3D<> _direction;


};

}
}




#endif /* SURFACEPOSESAMPLER_HPP_ */
