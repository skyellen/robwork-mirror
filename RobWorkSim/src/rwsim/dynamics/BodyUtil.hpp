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

namespace rw { namespace proximity { class CollisionDetector; } }
namespace rw { namespace kinematics { class State; } }

namespace rwsim {
namespace dynamics {
	class DynamicWorkCell;

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
                                        rw::common::Ptr<rw::proximity::CollisionDetector> coldect,
                                        const rw::kinematics::State& state,
                                        const rw::math::Vector3D<>& dir = -rw::math::Vector3D<>::z());

        static Body::Ptr getParentBody(Body::Ptr child, rw::common::Ptr<DynamicWorkCell> dwc, const rw::kinematics::State& state);

        static Body::Ptr getParentBody(rw::kinematics::Frame* child, rw::common::Ptr<DynamicWorkCell> dwc, const rw::kinematics::State& state );


    };

}
}
#endif /* PLACEBODY_HPP_ */
