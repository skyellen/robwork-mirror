/*
 * RenderBeam.hpp
 *
 *  Created on: 15/12/2010
 *      Author: jimali
 */

#ifndef RENDERBEAM_HPP_
#define RENDERBEAM_HPP_

#include <rw/graphics/Render.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <sandbox/models/BeamJoint.hpp>

namespace rwlibs {
namespace opengl {


    /** @addtogroup drawable */
    /*@{*/
    //! @file RenderSmoothSkin.hpp

    /**
     * @brief renders skin using vertice weights in relation to the bone structure.
     *
     * Each vertice has a number of bones attached. These attached bones influence the position of
     * the vertice.
     *
     * The transformation of each vertice is weighted against the transformation of each attached bone. Such
     * that the vertice position is defined by
     * $v' = \Sum w_i v . M_{[i]}
     * where M is the transform from \b Base frame to the the i'th attached bone (which is not the i'th bone).
     *
     * @note The number of attached bones to a single vertice should not be too large <5.
     */

    class RenderBeam: public rw::graphics::Render {
    public:

        typedef rw::common::Ptr<RenderBeam> Ptr;

        typedef std::pair<int, float> VerticeWeight;
        typedef std::vector<VerticeWeight> BoneWeights;

        //! @brief constructor
        RenderBeam(rw::models::BeamJoint* joint);

        RenderBeam(std::vector<rw::models::BeamJoint*> joints);

        //! @brief destructor
        virtual ~RenderBeam();

        void init(rw::geometry::IndexedTriMesh<>::Ptr mesh, rw::kinematics::State& state);

        /**
         * @brief draws the object.
         */
        virtual void draw(rw::graphics::DrawableNode::DrawType type, double alpha) const;


        void update(const rw::kinematics::State& state);

    private:

        int _nrOfSegments;
        std::vector<rw::models::BeamJoint*> _joints;
        std::vector<rw::math::Vector3D<> > _origvertices;
        rw::geometry::IndexedTriMesh<>::Ptr _mesh;
        std::vector<int> _verticetransform;
        struct Via {
            rw::math::Transform3D<> transform;
            double s;
        };

        std::vector<Via> _transforms, _origtransforms;
    };
}
}

#endif /* RENDERBEAM_HPP_ */
