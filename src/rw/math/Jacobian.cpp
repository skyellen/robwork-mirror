/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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
    Range(_jac, range(0, 3), range(0, 3)) = Rotation3D<>::Identity().m();
    Range(_jac, range(0, 3), range(3, 6)) = Math::Skew(aPb.m());
    Range(_jac, range(3, 6), range(0, 3)) = ZeroMatrix(3,3);
    Range(_jac, range(3, 6), range(3, 6)) = Rotation3D<>::Identity().m();
}

Jacobian::Jacobian(const Transform3D<>& aTb) : _jac(6, 6)
{
    const Rotation3D<>& aRb = aTb.R();
    const Vector3D<>& aPb = aTb.P();
    Range(_jac, range(0, 3), range(0, 3)) = aRb.m();
    Range(_jac, range(0, 3), range(3, 6)) = prod(Math::Skew(aPb.m()), aRb.m());
    Range(_jac, range(3, 6), range(0, 3)) = ZeroMatrix(3,3);
    Range(_jac, range(3, 6), range(3, 6)) = aRb.m();
}

Jacobian rw::math::operator*(const Rotation3D<>& r, const Jacobian& jacobian)
{
    Jacobian::Base v(jacobian.m());
    Jacobian::Base rv(v.size1(), v.size2());

    //RW_ASSERT(v.size1() == 6);
    for(size_t row=0; row<v.size1()-1; row+=6 ){
        for (size_t i = 0; i < v.size2(); i++) {
            
            range col(i, i + 1);
            
            range first(row+0, row+3);
            Range(rv, first, col) = prod(r.m(), Range(v, first, col));
    
            range second(row+3, row+6);
            Range(rv, second, col) = prod(r.m(), Range(v, second, col));
        }
    }
    return Jacobian(rv);
}
