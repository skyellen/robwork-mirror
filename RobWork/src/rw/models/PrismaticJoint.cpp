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


#include "PrismaticJoint.hpp"

#include <rw/kinematics/State.hpp>
#include <rw/math/Rotation3D.hpp>
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;






//----------------------------------------------------------------------
// PrismaticJoint
//----------------------------------------------------------------------


PrismaticJoint::PrismaticJoint(const std::string& name, const Transform3D<>& transform):
    Joint(name,1)
{
    const Rotation3D<>& rot = transform.R();
    if (rot == Rotation3D<>::identity())
        _impl = new PrismaticJointZeroRotationImpl(transform.P());
    else if (transform.P() == Vector3D<>(0, 0, 0))
        _impl = new PrismaticJointZeroOffsetImpl(transform.R());
    else
        _impl = new PrismaticJointImplBasic(transform);
}

PrismaticJoint::~PrismaticJoint(){
    delete _impl;
}
/*
void PrismaticJoint::getJointValueTransform(const Transform3D<>& parent,
                                            double q,
                                            Transform3D<>& result) const
{
    doGetJointValueTransform(parent, q, result);
}*/


void PrismaticJoint::multiplyJointTransform(const Transform3D<>& parent,
                                            const Q& q,
                                            Transform3D<>& result) const
{
    _impl->multiplyTransform(parent, q(0), result);
}



void PrismaticJoint::doMultiplyTransform(const Transform3D<>& parent,
                                         const State& state,
                                         Transform3D<>& result) const
{
    _impl->multiplyTransform(parent, getData(state)[0], result);
}

Transform3D<> PrismaticJoint::doGetTransform(const State& state) const
{
    return _impl->getTransform(getData(state)[0]);
}

void PrismaticJoint::getJacobian(size_t row, size_t col, const Transform3D<>& joint, const Transform3D<>& tcp, const State& state, Jacobian& jacobian) const {
	double q = getData(state)[0]; 
	_impl->getJacobian(row, col, joint, tcp, q, jacobian);
	//const Vector3D<> axis = joint.R().getCol(2);

    //jacobian.addPosition(axis, row, col);
}

rw::math::Transform3D<> PrismaticJoint::getFixedTransform() const{
	return _impl->getFixedTransform();
}

void PrismaticJoint::setFixedTransform( const rw::math::Transform3D<>& t3d) {
    PrismaticJointImpl *tmp = _impl;
    const Rotation3D<>& rot = t3d.R();
    if (rot == Rotation3D<>::identity())
        _impl = new PrismaticJointZeroRotationImpl(t3d.P());
    else if (t3d.P() == Vector3D<>(0, 0, 0))
        _impl = new PrismaticJointZeroOffsetImpl(t3d.R());
    else
        _impl = new PrismaticJointImplBasic(t3d);
    delete tmp;
}

//! @copydoc Joint::setJointMapping()
void PrismaticJoint::setJointMapping(rw::math::Function1Diff<>::Ptr function) {
	PrismaticJointImpl *tmp = _impl;
	_impl = new PrismaticJointWithQMapping(tmp->getFixedTransform(), function);
	delete tmp;
}

//! @copydoc Joint::removeJointMapping()
void PrismaticJoint::removeJointMapping() {
	Transform3D<> t3d = _impl->getFixedTransform();
	setFixedTransform(t3d);
}


rw::math::Transform3D<> PrismaticJoint::getTransform(double q) const{
    return _impl->getTransform( q );
}

Transform3D<> PrismaticJoint::getJointTransform(double q) const
{
    return rw::math::Transform3D<>( rw::math::Vector3D<>(0, 0, q) );
}

rw::math::Transform3D<> PrismaticJoint::getJointTransform(const rw::kinematics::State& state) const{
    const double q = getData(state)[0];
    return getJointTransform(q);
}

void PrismaticJoint::PrismaticJointImpl::getJacobian(size_t row,
						 size_t col,
						 const Transform3D<>& joint,
						 const Transform3D<>& tcp,
						 double q,
						 Jacobian& jacobian) const 
{
	Vector3D<> axis = joint.R().getCol(2);	
    jacobian.addPosition(axis, row, col);
}


PrismaticJoint::PrismaticJointWithQMapping::PrismaticJointWithQMapping(const Transform3D<>& t3d, const Function1Diff<>::Ptr mapping):	
	_mapping(mapping)
{
    const Rotation3D<>& rot = t3d.R();
    if (rot == Rotation3D<>::identity())
        _impl = new PrismaticJointZeroRotationImpl(t3d.P());
    else if (t3d.P() == Vector3D<>(0, 0, 0))
        _impl = new PrismaticJointZeroOffsetImpl(t3d.R());
    else
        _impl = new PrismaticJointImplBasic(t3d);
}

PrismaticJoint::PrismaticJointWithQMapping::~PrismaticJointWithQMapping() {
	delete _impl;
}

void PrismaticJoint::PrismaticJointWithQMapping::multiplyTransform(
	const rw::math::Transform3D<>& parent,
	double q,
	rw::math::Transform3D<>& result) const 
{
	/*double qnew = */_mapping->f(q);
	_impl->multiplyTransform(parent, q, result);
}

rw::math::Transform3D<> PrismaticJoint::PrismaticJointWithQMapping::getTransform(double q) {
	double qnew = _mapping->f(q);
	return _impl->getTransform(qnew);
}

rw::math::Transform3D<> PrismaticJoint::PrismaticJointWithQMapping::getFixedTransform() const {
	return _impl->getFixedTransform();
}


void PrismaticJoint::PrismaticJointWithQMapping::getJacobian(size_t row,
						 size_t col,
						 const Transform3D<>& joint,
						 const Transform3D<>& tcp,
						 double q,
						 Jacobian& jacobian) const 
{
	Vector3D<> axis = joint.R().getCol(2);
	axis *= _mapping->df(q);
    jacobian.addPosition(axis, row, col);
}
