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


#include "RevoluteJoint.hpp"

using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;






/*
    class RevoluteJointZeroOffsetImpl: public RevoluteJoint
    {
    public:
        RevoluteJointZeroOffsetImpl(const std::string& name,
                                    const Rotation3D<>& rotation) :
            RevoluteJoint(name),
            _transform(rotation)
        {
        }

    private:
        void doGetJointValueTransform(const Transform3D<>& parent, double q,
                                      Transform3D<>& result) const
        {
            const double a00 = parent.R()(0, 0);
            const double a01 = parent.R()(0, 1);
            const double a02 = parent.R()(0, 2);
            const double a10 = parent.R()(1, 0);
            const double a11 = parent.R()(1, 1);
            const double a12 = parent.R()(1, 2);
            const double a20 = parent.R()(2, 0);
            const double a21 = parent.R()(2, 1);
            const double a22 = parent.R()(2, 2);

            const double b00 = _transform.R()(0, 0);
            const double b01 = _transform.R()(0, 1);
            const double b02 = _transform.R()(0, 2);
            const double b10 = _transform.R()(1, 0);
            const double b11 = _transform.R()(1, 1);
            const double b12 = _transform.R()(1, 2);
            const double b20 = _transform.R()(2, 0);
            const double b21 = _transform.R()(2, 1);
            const double b22 = _transform.R()(2, 2);

            const double a00b00 = a00 * b00;
            const double a01b10 = a01 * b10;
            const double a01b11 = a01 * b11;
            const double a00b01 = a00 * b01;
            const double a02b21 = a02 * b21;
            const double a02b20 = a02 * b20;
            const double a10b00 = a10 * b00;
            const double a11b10 = a11 * b10;
            const double a11b11 = a11 * b11;
            const double a12b20 = a12 * b20;
            const double a12b21 = a12 * b21;
            const double a10b01 = a10 * b01;
            const double a20b00 = a20 * b00;
            const double a21b10 = a21 * b10;
            const double a22b20 = a22 * b20;
            const double a20b01 = a20 * b01;
            const double a21b11 = a21 * b11;
            const double a22b21 = a22 * b21;

            const double a20b01_a21b11_a22b21 = a20b01 + a21b11 + a22b21;
            const double a20b00_a21b10_a22b20 = a20b00 + a21b10 + a22b20;
            const double a10b01_a11b11_a12b21 = a10b01 + a11b11 + a12b21;
            const double a00b01_a01b11_a02b21 = a00b01 + a01b11 + a02b21;
            const double a10b00_a11b10_a12b20 = a10b00 + a11b10 + a12b20;
            const double a00b00_a01b10_a02b20 = a00b00 + a01b10 + a02b20;

            const double cq = cos(q);
            const double sq = sin(q);

            result.P() = parent.P();

            result.R() = Rotation3D<> (a00b00_a01b10_a02b20 * cq
                    + a00b01_a01b11_a02b21 * sq, a00b01_a01b11_a02b21 * cq
                    - a00b00_a01b10_a02b20 * sq, a00 * b02 + a01 * b12 + a02
                    * b22,

            a10b00_a11b10_a12b20 * cq + a10b01_a11b11_a12b21 * sq,
                                       a10b01_a11b11_a12b21 * cq
                                               - a10b00_a11b10_a12b20 * sq, a10
                                               * b02 + a11 * b12 + a12 * b22,

                                       a20b00_a21b10_a22b20 * cq
                                               + a20b01_a21b11_a22b21 * sq,
                                       a20b01_a21b11_a22b21 * cq
                                               - a20b00_a21b10_a22b20 * sq, a20
                                               * b02 + a21 * b12 + a22 * b22);
        }

    private:
        Transform3D<> _transform;
    };*/
//}

//----------------------------------------------------------------------
// RevoluteJoint
//----------------------------------------------------------------------


RevoluteJoint::RevoluteJoint(const std::string& name,
                             const Transform3D<>& transform):
    Joint(name, 1)
{
     if (transform.P() == Vector3D<>(0, 0, 0))
         _impl = new RevoluteJointZeroOffsetImpl(transform.R());
     else
         _impl = new RevoluteJointBasic(transform);
}

RevoluteJoint::~RevoluteJoint(){
    delete _impl;
}


void RevoluteJoint::multiplyJointTransform(const Transform3D<>& parent,
                                           const Q& q,
                                           Transform3D<>& result) const
{
    _impl->multiplyTransform(parent, q(0), result);
}

void RevoluteJoint::doMultiplyTransform(const Transform3D<>& parent,
                                        const State& state,
                                        Transform3D<>& result) const
{
    _impl->multiplyTransform(parent, getData(state)[0], result);
}

Transform3D<> RevoluteJoint::doGetTransform(const State& state) const
{
    return _impl->getTransform(getData(state)[0]);
}


void RevoluteJoint::getJacobian(size_t row, size_t col, const Transform3D<>& joint, const Transform3D<>& tcp, const State& state, Jacobian& jacobian) const {
    double q = getData(state)[0];
	_impl->getJacobian(row, col, joint, tcp, q, jacobian);
	//const Vector3D<> axis = joint.R().getCol(2);
	//if (_impl->getJacobianScale() != 1) {
	//	axis *= _impl->getJacobianScale();
	//}
	//const Vector3D<> p = cross(axis, tcp.P() - joint.P());

 //   jacobian.addPosition(p, row, col);
 //   jacobian.addRotation(axis,row, col);
}

rw::math::Transform3D<> RevoluteJoint::getFixedTransform() const{
	return _impl->getFixedTransform();
}


void RevoluteJoint::setFixedTransform( const rw::math::Transform3D<>& t3d) {
    // this might change the impl, os
    RevoluteJointImpl *tmp = _impl;
    if (t3d.P() == Vector3D<>(0, 0, 0))
        _impl = new RevoluteJointZeroOffsetImpl(t3d.R());
    else
        _impl = new RevoluteJointBasic(t3d);
    delete tmp;
}

void RevoluteJoint::setJointMapping(rw::math::Function1Diff<>::Ptr function) {
	RevoluteJointImpl *tmp = _impl;
	_impl = new RevoluteJointWithQMapping(tmp->getFixedTransform(), function);
	delete tmp;
}

void RevoluteJoint::removeJointMapping() {
    RevoluteJointImpl *tmp = _impl;
	Transform3D<> t3d = _impl->getFixedTransform();
    if (t3d.P() == Vector3D<>(0, 0, 0))
        _impl = new RevoluteJointZeroOffsetImpl(t3d.R());
    else
        _impl = new RevoluteJointBasic(t3d);
    delete tmp;	
}

rw::math::Transform3D<> RevoluteJoint::getTransform(double q) const{
    return _impl->getTransform(q);
}

Transform3D<> RevoluteJoint::getJointTransform(double q) const
{
    const double cq = cos(q);
    const double sq = sin(q);

    rw::math::Transform3D<> result;
    result(0,0) =  cq;
    result(0,1) = -sq;
    result(0,2) = 0;
    result(0,3) = 0;

    result(1,0) = sq;
    result(1,1) = cq;
    result(1,2) = 0;
    result(1,3) = 0;

    result(2,0) = 0;
    result(2,1) = 0;
    result(2,2) = 1;
    result(2,3) = 0;

    return result;
}

rw::math::Transform3D<> RevoluteJoint::getJointTransform(const rw::kinematics::State& state) const{
    const double q = getData(state)[0];
    return getJointTransform(q);
}


void RevoluteJoint::RevoluteJointImpl::getJacobian(size_t row,
  												   size_t col,
												   const Transform3D<>& joint,
												   const Transform3D<>& tcp,
												   double q,
												   Jacobian& jacobian) const 
{
	const Vector3D<> axis = joint.R().getCol(2);
	const Vector3D<> p = cross(axis, tcp.P() - joint.P());

    jacobian.addPosition(p, row, col);
    jacobian.addRotation(axis,row, col);
}


RevoluteJoint::RevoluteJointBasic::RevoluteJointBasic(const rw::math::Transform3D<>& transform) :
    _transform(transform)
{ 
	}

void RevoluteJoint::RevoluteJointBasic::multiplyTransform(const rw::math::Transform3D<>& parent,
                       double q,
                       rw::math::Transform3D<>& result) const
{
    const double a00 = parent.R()(0, 0);
    const double a01 = parent.R()(0, 1);
    const double a02 = parent.R()(0, 2);
    const double a10 = parent.R()(1, 0);
    const double a11 = parent.R()(1, 1);
    const double a12 = parent.R()(1, 2);
    const double a20 = parent.R()(2, 0);
    const double a21 = parent.R()(2, 1);
    const double a22 = parent.R()(2, 2);
    const double ax = parent.P()(0);
    const double ay = parent.P()(1);
    const double az = parent.P()(2);

    const double b00 = _transform.R()(0, 0);
    const double b01 = _transform.R()(0, 1);
    const double b02 = _transform.R()(0, 2);
    const double b10 = _transform.R()(1, 0);
    const double b11 = _transform.R()(1, 1);
    const double b12 = _transform.R()(1, 2);
    const double b20 = _transform.R()(2, 0);
    const double b21 = _transform.R()(2, 1);
    const double b22 = _transform.R()(2, 2);
    const double bx = _transform.P()(0);
    const double by = _transform.P()(1);
    const double bz = _transform.P()(2);

    const double a00b00 = a00 * b00;
    const double a01b10 = a01 * b10;
    const double a01b11 = a01 * b11;
    const double a00b01 = a00 * b01;
    const double a02b21 = a02 * b21;
    const double a02b20 = a02 * b20;
    const double a10b00 = a10 * b00;
    const double a11b10 = a11 * b10;
    const double a11b11 = a11 * b11;
    const double a12b20 = a12 * b20;
    const double a12b21 = a12 * b21;
    const double a10b01 = a10 * b01;
    const double a20b00 = a20 * b00;
    const double a21b10 = a21 * b10;
    const double a22b20 = a22 * b20;
    const double a20b01 = a20 * b01;
    const double a21b11 = a21 * b11;
    const double a22b21 = a22 * b21;

    const double a20b01_a21b11_a22b21 = a20b01 + a21b11 + a22b21;
    const double a20b00_a21b10_a22b20 = a20b00 + a21b10 + a22b20;
    const double a10b01_a11b11_a12b21 = a10b01 + a11b11 + a12b21;
    const double a00b01_a01b11_a02b21 = a00b01 + a01b11 + a02b21;
    const double a10b00_a11b10_a12b20 = a10b00 + a11b10 + a12b20;
    const double a00b00_a01b10_a02b20 = a00b00 + a01b10 + a02b20;

    const double cq = cos(q);
    const double sq = sin(q);

    result.P() = rw::math::Vector3D<> (ax + a00 * bx + a01 * by + a02 * bz, ay
            + a10 * bx + a11 * by + a12 * bz, az + a20 * bx + a21 * by
            + a22 * bz);

    result.R() = rw::math::Rotation3D<> (a00b00_a01b10_a02b20 * cq
            + a00b01_a01b11_a02b21 * sq, a00b01_a01b11_a02b21 * cq
            - a00b00_a01b10_a02b20 * sq, a00 * b02 + a01 * b12 + a02
            * b22,

    a10b00_a11b10_a12b20 * cq + a10b01_a11b11_a12b21 * sq,
                               a10b01_a11b11_a12b21 * cq
                                       - a10b00_a11b10_a12b20 * sq, a10
                                       * b02 + a11 * b12 + a12 * b22,

                               a20b00_a21b10_a22b20 * cq
                                       + a20b01_a21b11_a22b21 * sq,
                               a20b01_a21b11_a22b21 * cq
                                       - a20b00_a21b10_a22b20 * sq, a20
                                       * b02 + a21 * b12 + a22 * b22);
}

rw::math::Transform3D<> RevoluteJoint::RevoluteJointBasic::getTransform(double q) {
    const double b00 = _transform.R()(0, 0);
    const double b01 = _transform.R()(0, 1);
    const double b02 = _transform.R()(0, 2);
    const double b10 = _transform.R()(1, 0);
    const double b11 = _transform.R()(1, 1);
    const double b12 = _transform.R()(1, 2);
    const double b20 = _transform.R()(2, 0);
    const double b21 = _transform.R()(2, 1);
    const double b22 = _transform.R()(2, 2);
    const double bx = _transform.P()(0);
    const double by = _transform.P()(1);
    const double bz = _transform.P()(2);

    const double cq = cos(q);
    const double sq = sin(q);

    rw::math::Transform3D<> result;
    result(0,0) = b00*cq + b01*sq;
    result(0,1) = b01*cq - b00*sq;
    result(0,2) = b02;
    result(0,3) = bx;

    result(1,0) = b10*cq + b11*sq;
    result(1,1) = b11*cq - b10*sq;
    result(1,2) = b12;
    result(1,3) = by;

    result(2,0) = b20*cq + b21*sq;
    result(2,1) = b21*cq - b20*sq;
    result(2,2) = b22;
    result(2,3) = bz;

    return result;
}

rw::math::Transform3D<> RevoluteJoint::RevoluteJointBasic::getFixedTransform() const{
    return _transform;
}

////////////////////////////////////////////////////////////////////////////////////////
/////// RevoluteJointZeroOffsetImpl

RevoluteJoint::RevoluteJointZeroOffsetImpl::RevoluteJointZeroOffsetImpl(const rw::math::Rotation3D<>& rotation) :
    _transform(rotation)
{
}


void RevoluteJoint::RevoluteJointZeroOffsetImpl::multiplyTransform(const rw::math::Transform3D<>& parent,
                       double q,
                       rw::math::Transform3D<>& result) const
{
    const double a00 = parent.R()(0, 0);
    const double a01 = parent.R()(0, 1);
    const double a02 = parent.R()(0, 2);
    const double a10 = parent.R()(1, 0);
    const double a11 = parent.R()(1, 1);
    const double a12 = parent.R()(1, 2);
    const double a20 = parent.R()(2, 0);
    const double a21 = parent.R()(2, 1);
    const double a22 = parent.R()(2, 2);

    const double b00 = _transform.R()(0, 0);
    const double b01 = _transform.R()(0, 1);
    const double b02 = _transform.R()(0, 2);
    const double b10 = _transform.R()(1, 0);
    const double b11 = _transform.R()(1, 1);
    const double b12 = _transform.R()(1, 2);
    const double b20 = _transform.R()(2, 0);
    const double b21 = _transform.R()(2, 1);
    const double b22 = _transform.R()(2, 2);

    const double a00b00 = a00 * b00;
    const double a01b10 = a01 * b10;
    const double a01b11 = a01 * b11;
    const double a00b01 = a00 * b01;
    const double a02b21 = a02 * b21;
    const double a02b20 = a02 * b20;
    const double a10b00 = a10 * b00;
    const double a11b10 = a11 * b10;
    const double a11b11 = a11 * b11;
    const double a12b20 = a12 * b20;
    const double a12b21 = a12 * b21;
    const double a10b01 = a10 * b01;
    const double a20b00 = a20 * b00;
    const double a21b10 = a21 * b10;
    const double a22b20 = a22 * b20;
    const double a20b01 = a20 * b01;
    const double a21b11 = a21 * b11;
    const double a22b21 = a22 * b21;

    const double a20b01_a21b11_a22b21 = a20b01 + a21b11 + a22b21;
    const double a20b00_a21b10_a22b20 = a20b00 + a21b10 + a22b20;
    const double a10b01_a11b11_a12b21 = a10b01 + a11b11 + a12b21;
    const double a00b01_a01b11_a02b21 = a00b01 + a01b11 + a02b21;
    const double a10b00_a11b10_a12b20 = a10b00 + a11b10 + a12b20;
    const double a00b00_a01b10_a02b20 = a00b00 + a01b10 + a02b20;

    const double cq = cos(q);
    const double sq = sin(q);

    result.P() = parent.P();

    result.R() = rw::math::Rotation3D<> (a00b00_a01b10_a02b20 * cq
            + a00b01_a01b11_a02b21 * sq, a00b01_a01b11_a02b21 * cq
            - a00b00_a01b10_a02b20 * sq, a00 * b02 + a01 * b12 + a02
            * b22,

    a10b00_a11b10_a12b20 * cq + a10b01_a11b11_a12b21 * sq,
                               a10b01_a11b11_a12b21 * cq
                                       - a10b00_a11b10_a12b20 * sq, a10
                                       * b02 + a11 * b12 + a12 * b22,

                               a20b00_a21b10_a22b20 * cq
                                       + a20b01_a21b11_a22b21 * sq,
                               a20b01_a21b11_a22b21 * cq
                                       - a20b00_a21b10_a22b20 * sq, a20
                                       * b02 + a21 * b12 + a22 * b22);
}

rw::math::Transform3D<> RevoluteJoint::RevoluteJointZeroOffsetImpl::getTransform(double q) {
    const double b00 = _transform.R()(0, 0);
    const double b01 = _transform.R()(0, 1);
    const double b02 = _transform.R()(0, 2);
    const double b10 = _transform.R()(1, 0);
    const double b11 = _transform.R()(1, 1);
    const double b12 = _transform.R()(1, 2);
    const double b20 = _transform.R()(2, 0);
    const double b21 = _transform.R()(2, 1);
    const double b22 = _transform.R()(2, 2);

    const double cq = cos(q);
    const double sq = sin(q);

    rw::math::Transform3D<> result;
    result(0,0) = b00*cq + b01*sq;
    result(0,1) = b01*cq - b00*sq;
    result(0,2) = b02;
    result(0,3) = 0;

    result(1,0) = b10*cq + b11*sq;
    result(1,1) = b11*cq - b10*sq;
    result(1,2) = b12;
    result(1,3) = 0;

    result(2,0) = b20*cq + b21*sq;
    result(2,1) = b21*cq - b20*sq;
    result(2,2) = b22;
    result(2,3) = 0;
    return result;
}

rw::math::Transform3D<> RevoluteJoint::RevoluteJointZeroOffsetImpl::getFixedTransform() const {
    return _transform;
}


RevoluteJoint::RevoluteJointWithQMapping::RevoluteJointWithQMapping(const Transform3D<>& t3d, Function1Diff<>::Ptr mapping):	
	_mapping(mapping)
{
    if (t3d.P() == Vector3D<>(0, 0, 0))
        _impl = new RevoluteJointZeroOffsetImpl(t3d.R());
    else
        _impl = new RevoluteJointBasic(t3d);
}

RevoluteJoint::RevoluteJointWithQMapping::~RevoluteJointWithQMapping() {
	delete _impl;
}

void RevoluteJoint::RevoluteJointWithQMapping::multiplyTransform(
	const rw::math::Transform3D<>& parent,
	double q,
	rw::math::Transform3D<>& result) const 
{
	double qnew = _mapping->f(q);
	_impl->multiplyTransform(parent, qnew, result);
}

rw::math::Transform3D<> RevoluteJoint::RevoluteJointWithQMapping::getTransform(double q) {
	double qnew = _mapping->f(q);
	return _impl->getTransform(qnew);
}

rw::math::Transform3D<> RevoluteJoint::RevoluteJointWithQMapping::getFixedTransform() const {
	return _impl->getFixedTransform();
}

void RevoluteJoint::RevoluteJointWithQMapping::getJacobian(size_t row,
										size_t col,
										const Transform3D<>& joint,
										const Transform3D<>& tcp,
										double q,
										Jacobian& jacobian) const 
{	
	Vector3D<> axis = joint.R().getCol(2);
	//The axis is scaled with the first order derivative of the mapping to account for the mapping.
	axis *= _mapping->df(q);
	const Vector3D<> p = cross(axis, tcp.P() - joint.P());

    jacobian.addPosition(p, row, col);
    jacobian.addRotation(axis,row, col);	
}
