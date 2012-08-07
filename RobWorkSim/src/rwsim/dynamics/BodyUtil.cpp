
#include "BodyUtil.hpp"

using namespace rwsim::dynamics;

#include <rw/rw.hpp>
USE_ROBWORK_NAMESPACE

using namespace robwork;

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
    Transform3D<> wTb = Kinematics::worldTframe(mframe, state);

    // first we transform dir to the parent frame
    Vector3D<> step = inverse(Kinematics::worldTframe(mframe->getParent(state), state))*dir;

    Transform3D<> ftrans = moveToColllision(mframe, Transform3D<>(step/200 ), coldect, state);

    return ftrans;
}



Body::Ptr BodyUtil::getParentBody(dynamics::Body::Ptr child, dynamics::DynamicWorkCell::Ptr dwc, const rw::kinematics::State& state){
    return getParentBody(child->getBodyFrame(),dwc,state);
}

Body::Ptr BodyUtil::getParentBody(rw::kinematics::Frame* child, dynamics::DynamicWorkCell::Ptr dwc, const rw::kinematics::State& state ){
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



