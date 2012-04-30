/*
 * PlaceBody.hpp
 *
 *  Created on: 19/05/2011
 *      Author: jimali
 */

#ifndef RWSIM_DYNAMICS_BODYUTIL_HPP_
#define RWSIM_DYNAMICS_BODYUTIL_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include "Body.hpp"
#include "DynamicWorkCell.hpp"
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/kinematics/State.hpp>

namespace rwsim {
namespace dynamics {

    class BodyUtil {
    public:

        /**
         * @brief a utility function for calculating the transformation of a body if
         * it is kinematically projected in the direction of \b dir. Where dir is described
         * in world coordinates.
         * @param body
         * @param coldect
         * @param state
         * @param dir
         * @return
         */
        static rw::math::Transform3D<> placeBody( rwsim::dynamics::Body::Ptr body,
                                        rw::proximity::CollisionDetector::Ptr coldect,
                                        const rw::kinematics::State& state,
                                        const rw::math::Vector3D<>& dir = -rw::math::Vector3D<>::z());

        static Body* getParentBody(Body* child, DynamicWorkCell::Ptr dwc, const rw::kinematics::State& state);

        static Body* getParentBody(rw::kinematics::Frame* child, DynamicWorkCell::Ptr dwc, const rw::kinematics::State& state );


    };

}
}
#endif /* PLACEBODY_HPP_ */
