#include "Link.hpp"

#include <stddef.h>
#include <assert.h>


using namespace rw::task;

Link::Link(const std::string &name)
    :
    _motion_constraint(NoConstraint()),
    _name(name)
{
	_next = _prev = NULL;
}

Link::Link(const MotionConstraint &motion_constraint, const std::string &name) 
   :
    _motion_constraint(motion_constraint),
    _name(name)
{	
	_next = _prev = NULL;
}

Link::~Link()
{}


rw::interpolator::Pose6dStraightSegment LinearToolConstraint::Interpolator(rw::math::Transform3D<> &a, rw::math::Transform3D<> &b)
{
	return rw::interpolator::Pose6dStraightSegment(a, rw::math::VelocityScrew6D<>(b.P()-a.P(),inverse(a.R())*b.R()));
}



//Copy of linear interpolator - will be fixed later

/*
rw::interpolator::Pose6dStraightSegment CircularToolConstraint::Interpolator(
    rw::math::Transform3D<> &a, rw::math::Transform3D<> &b)
{
	return rw::interpolator::Pose6dStraightSegment(
        a, rw::math::VelocityScrew6D<>(b.P()-a.P(),inverse(a.R())*b.R()));
}

rw::interpolator::StraightSegment LinearJointConstraint::Interpolator(
    rw::math::Q &a, rw::math::Q &b)
{
	return rw::interpolator::StraightSegment(a,b,1);
}

*/

