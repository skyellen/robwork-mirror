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

#include "Jacobian.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

#include "Math.hpp"

using namespace rw::math;
using namespace Eigen;

Jacobian::Jacobian(const Rotation3D<>& aRb) : _jac(6, 6)
{
	_jac.block<3,3>(0,0) = aRb.e();
	_jac.block<3,3>(0,3) = Matrix<double, 3, 3>::Zero();
	_jac.block<3,3>(3,0) = Matrix<double, 3, 3>::Zero();
	_jac.block<3,3>(3,3) = aRb.e();
}

Jacobian::Jacobian(const Vector3D<>& aPb) : _jac(6, 6)
{
	_jac.block<3,3>(0,0) = Matrix<double, 3, 3>::Identity();
	_jac.block<3,3>(0,3) = Math::skew(aPb);
	_jac.block<3,3>(3,0) = Matrix<double, 3, 3>::Zero();
	_jac.block<3,3>(3,3) = Matrix<double, 3, 3>::Identity();
}

Jacobian::Jacobian(const Transform3D<>& aTb) : _jac(6, 6)
{
    const Rotation3D<>& aRb = aTb.R();
    const Vector3D<>& aPb = aTb.P();

	_jac.block<3,3>(0,0) = aRb.e();
	_jac.block<3,3>(0,3) = Math::skew(aPb) * aRb.e();
	_jac.block<3,3>(3,0) = Matrix<double,3,3>::Zero();
	_jac.block<3,3>(3,3) = aRb.e();
}

const Jacobian rw::math::operator*(const Rotation3D<>& r, const Jacobian& jacobian)
{
    Jacobian::Base v(jacobian.e());
    Jacobian::Base rv(v.rows(), v.cols());
	Rotation3D<>::EigenMatrix3x3 rm = r.e();
    for(int row = 0; row < v.rows()-1; row += 3 ){
        for (int col = 0; col < v.cols(); col++) {
			rv.block<3,1>(row, col) = rm*v.block<3,1>(row, col);
        }
    }
    return Jacobian(rv);
}

void Jacobian::addPosition(const Vector3D<>& pos, size_t row, size_t col) {
    _jac(row, col) += pos(0);
    _jac(row+1, col) += pos(1);
    _jac(row+2, col) += pos(2);
}


void Jacobian::addRotation(const Vector3D<>& rot, size_t row, size_t col) {
    _jac(row+3, col) += rot(0);
    _jac(row+4, col) += rot(1);
    _jac(row+5, col) += rot(2);
}


template<>
void rw::common::serialization::read(Jacobian& tmp, InputArchive& iar, const std::string& id){
    std::vector<double> arr;
    size_t size1,size2;
    iar.readEnterScope(id);
    iar.read( size1, "size1" );
    iar.read( size2, "size2" );
    iar.read( arr, "data" );
    iar.readLeaveScope(id);
    tmp = Jacobian(size1,size2);
    Math::fromStdVectorToMat(arr, tmp, (int)tmp.size1(), (int)tmp.size2());
}

template<>
void rw::common::serialization::write(const Jacobian& tmp, OutputArchive& oar, const std::string& id){
    oar.writeEnterScope(id);
    oar.write( tmp.size1(), "size1" );
    oar.write( tmp.size2(), "size2" );
    oar.write( Math::toStdVector(tmp, (int)tmp.size1(), (int)tmp.size2()), "data" );
    oar.writeLeaveScope(id);
}

