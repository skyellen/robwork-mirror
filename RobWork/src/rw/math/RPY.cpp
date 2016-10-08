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

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

#include "RPY.hpp"
#include "Constants.hpp"

using namespace rw::common;
using namespace rw::math;

template<class T>
RPY<T>::RPY(const Rotation3D<T>& R, T epsilon)
{
    const T r11 = R(0, 0);
    const T r12 = R(0, 1);
    const T r21 = R(1, 0);
    const T r22 = R(1, 1);
    const T r31 = R(2, 0);
    const T r32 = R(2, 1);
    const T r33 = R(2, 2);

    double sum = r11*r11 + r21*r21;

    // We check for rounding errors in input so that we don't take the sqrt of
    // some negative number.
    if (sum < 0) sum = 0;
    // TODO: Check that if sum < 0 then sum ~= 0 also.

    const double cos_beta = sqrt(sum);
    const double sin_beta = -r31;

    // If beta == 90 deg or beta == -90 deg:
    if (fabs(cos_beta) < epsilon) {

        // If beta == -90 deg:
        if (sin_beta < 0) {
            _rpy(0) = 0;
            _rpy(1) = static_cast<T>(-Pi / 2);
            _rpy(2) = - atan2(r12, r22);
        }

        // If beta == 90 deg:
        else {
            _rpy(0) = 0;
            _rpy(1) = static_cast<T>(Pi / 2);
            _rpy(2) = atan2(r12, r22);
        }

    } else {
        _rpy(1) = static_cast<T>(atan2(sin_beta, cos_beta));
        //_rpy(0) = static_cast<T>(atan2(r21 / cos_beta, r11 / cos_beta));
		_rpy(0) = static_cast<T>(atan2(r21, r11));
		//_rpy(2) = static_cast<T>(atan2(r32 / cos_beta, r33 / cos_beta));
        _rpy(2) = static_cast<T>(atan2(r32, r33));
    }
}

template<class T>
const Rotation3D<T> RPY<T>::toRotation3D() const
{
    const T a = _rpy(0);
    const T b = _rpy(1);
    const T c = _rpy(2);

    const T ca = cos(a);
    const T sa = sin(a);
    const T cb = cos(b);
    const T sb = sin(b);
    const T cc = cos(c);
    const T sc = sin(c);

    return Rotation3D<T>(
        ca * cb,
        ca * sb * sc-sa * cc,
        ca * sb * cc+sa * sc,

        sa * cb,
        sa * sb * sc+ca * cc,
        sa * sb * cc-ca * sc,

        -sb,
        cb * sc,
        cb * cc);
}

template class rw::math::RPY<double>;
template class rw::math::RPY<float>;

namespace rw{ namespace common { namespace serialization {

    template<class T>
    void writeImpl(const RPY<T>& tmp, OutputArchive& oar, const std::string& id){
        oar.writeEnterScope(id);
        oar.write( tmp[0]*Rad2Deg , "r" );
        oar.write( tmp[1]*Rad2Deg , "p" );
        oar.write( tmp[2]*Rad2Deg , "y" );
        oar.writeLeaveScope(id);
    }

    template<class T>
    void readImpl(RPY<T>& tmp, InputArchive& iar, const std::string& id){
        double r,p,y;
        iar.readEnterScope(id);
        iar.read( r , "r" );
        iar.read( p , "p" );
        iar.read( y , "y" );
        iar.readLeaveScope(id);
        tmp[0] = (T)(r*Deg2Rad);
        tmp[1] = (T)(p*Deg2Rad);
        tmp[2] = (T)(y*Deg2Rad);
    }

    // we need these to explicitly instantiate these functions
    template<> void write(const RPY<double>& tmp, OutputArchive& oar, const std::string& id ) { writeImpl(tmp,oar,id); }
    template<> void write(const RPY<float>& tmp, OutputArchive& oar, const std::string& id ) { writeImpl(tmp,oar,id); }
    template<> void read(RPY<double>& tmp, InputArchive& iar, const std::string& id) { readImpl(tmp,iar,id); }
    template<> void read(RPY<float>& tmp, InputArchive& iar, const std::string& id) { readImpl(tmp,iar,id); }

}}}
