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

#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Math.hpp>
#include <rw/common/macros.hpp>

using namespace rw::math;
using namespace boost::numeric::ublas;

typedef matrix_range<Jacobian::Base> Range;
typedef zero_matrix<double> ZeroMatrix;

Jacobian::Jacobian(const Rotation3D<>& aRb) : _jac(6, 6)
{
    Range(_jac, range(0, 3), range(0, 3)) = aRb.m();
    Range(_jac, range(0, 3), range(3, 6)) = ZeroMatrix(3, 3);
    Range(_jac, range(3, 6), range(0, 3)) = ZeroMatrix(3, 3);
    Range(_jac, range(3, 6), range(3, 6)) = aRb.m();
}

Jacobian::Jacobian(const Vector3D<>& aPb) : _jac(6, 6)
{
    Range(_jac, range(0, 3), range(0, 3)) = Rotation3D<>::identity().m();
    Range(_jac, range(0, 3), range(3, 6)) = Math::skew(aPb.m());
    Range(_jac, range(3, 6), range(0, 3)) = ZeroMatrix(3,3);
    Range(_jac, range(3, 6), range(3, 6)) = Rotation3D<>::identity().m();
}

Jacobian::Jacobian(const Transform3D<>& aTb) : _jac(6, 6)
{
    const Rotation3D<>& aRb = aTb.R();
    const Vector3D<>& aPb = aTb.P();
    Range(_jac, range(0, 3), range(0, 3)) = aRb.m();
    Range(_jac, range(0, 3), range(3, 6)) = prod(Math::skew(aPb.m()), aRb.m());
    Range(_jac, range(3, 6), range(0, 3)) = ZeroMatrix(3, 3);
    Range(_jac, range(3, 6), range(3, 6)) = aRb.m();
}

Jacobian rw::math::operator*(const Rotation3D<>& r, const Jacobian& jacobian)
{
    Jacobian::Base v(jacobian.m());
    Jacobian::Base rv(v.size1(), v.size2());

    for(size_t row = 0; row < v.size1()-1; row += 6 ){
        for (size_t i = 0; i < v.size2(); i++) {

            range col(i, i + 1);

            range first(row + 0, row + 3);
            Range(rv, first, col) = prod(r.m(), Range(v, first, col));

            range second(row + 3, row + 6);
            Range(rv, second, col) = prod(r.m(), Range(v, second, col));
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
