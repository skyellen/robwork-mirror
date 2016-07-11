
#include "BodyUtil.hpp"
#include "DynamicWorkCell.hpp"

#include <rw/kinematics/MovableFrame.hpp>
#include <rw/proximity/CollisionDetector.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using rw::proximity::CollisionDetector;
using namespace rwsim::dynamics;

namespace {

    Transform3D<> moveToColllision(MovableFrame* mframe, const Transform3D<>& nstep, CollisionDetector::Ptr coldect, State& state){
        const size_t MAX_ITERATIONS = 200;
        CollisionDetector::QueryResult result;
        for(size_t i=0; i<MAX_ITERATIONS; i++){
            Transform3D<> cfree = mframe->getTransform(state);
            Transform3D<> ntransform = cfree*nstep;
            mframe->setTransform( ntransform, state );
            if(coldect->inCollision(state, &result, true)){
                return cfree;
            }
        }
        return mframe->getTransform(state);
    }

}

Transform3D<> BodyUtil::placeBody( rwsim::dynamics::Body::Ptr body,
                                   CollisionDetector::Ptr coldect,
                                   const rw::kinematics::State& fstate,
                                   const rw::math::Vector3D<>& dir)
{
    Frame *frame = body->getBodyFrame();
    // currently the body frame must be of type MovableFrame
    MovableFrame* mframe = dynamic_cast<MovableFrame*>(frame);
    if(mframe==NULL)
        RW_THROW("Body frame is not of type Movable!");

    State state = fstate;
    // basically we step the body in the direction of dir

    // first we transform dir to the parent frame
    Vector3D<> step = inverse(Kinematics::worldTframe(mframe->getParent(state), state))*dir;

    Transform3D<> ftrans = moveToColllision(mframe, Transform3D<>(step/200 ), coldect, state);

    return ftrans;
}



Body::Ptr BodyUtil::getParentBody(rwsim::dynamics::Body::Ptr child, rwsim::dynamics::DynamicWorkCell::Ptr dwc, const rw::kinematics::State& state){
    return getParentBody(child->getBodyFrame(),dwc,state);
}

Body::Ptr BodyUtil::getParentBody(rw::kinematics::Frame* child, rwsim::dynamics::DynamicWorkCell::Ptr dwc, const rw::kinematics::State& state ){
    Frame *f = child;
    Body::Ptr pbody = NULL;
    while(pbody==NULL){
        f = f->getParent(state);
        if(f==NULL)
            return NULL;
        pbody = dwc->getBody(f);
    }
    return pbody;
}



