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

#include "ProjectionMatrix.hpp"

#include <rw/math/Constants.hpp>

using namespace rw::math;

void ProjectionMatrix::setOrtho(double left, double right,
              double bottom, double top,
              double zNear, double zFar)
{

    double tx = -(right+left)/(right-left);
    double ty = -(top+bottom)/(top-bottom);
    double tz = -(zFar+zNear)/(zFar-zNear);
    // row 1
    _matrix(0,0) =  2.0/(right-left);
    _matrix(0,1) = 0.0;
    _matrix(0,2) = 0.0;
    _matrix(0,3) = tx;
    // row 2
    _matrix(1,0) = 0.0;
    _matrix(1,1) = 2.0/(top-bottom);
    _matrix(1,2) = 0.0;
    _matrix(1,3) = ty;
    // row 3
    _matrix(2,0) = 0.0;
    _matrix(2,1) = 0.0;
    _matrix(2,2) =  -2.0/(zFar-zNear);
    _matrix(2,3) = tz;

    _matrix(3,0) = 0.0;
    _matrix(3,1) = 0.0;
    _matrix(3,2) = 0.0;
    _matrix(3,3) = 1.0;
}

void ProjectionMatrix::setFrustum(double left, double right,
                         double bottom, double top,
                         double zNear, double zFar)
{
    double A = (right+left)/(right-left);
    double B = (top+bottom)/(top-bottom);
    double C = -(zFar+zNear)/(zFar-zNear);
    double D = -2.0*zFar*zNear/(zFar-zNear);

    // row 1
    _matrix(0,0) =  2.0*zNear/(right-left);
    _matrix(0,1) = 0.0;
    _matrix(0,2) = A;
    _matrix(0,3) = 0.0;
    // row 2
    _matrix(1,0) = 0.0;
    _matrix(1,1) = 2.0*zNear/(top-bottom);
    _matrix(1,2) = B;
    _matrix(1,3) = 0.0;
    // row 3
    _matrix(2,0) = 0.0;
    _matrix(2,1) = 0.0;
    _matrix(2,2) = C;
    _matrix(2,3) = D;

    _matrix(3,0) = 0.0;
    _matrix(3,1) = 0.0;
    _matrix(3,2) = -1.0;
    _matrix(3,3) = 0.0;
}

bool ProjectionMatrix::getFrustum(double& left, double& right,
                                       double& bottom, double& top,
                                       double& zNear, double& zFar) const
{
    if ( _matrix(3,0)!=0.0 || _matrix(3,1)!=0.0 || _matrix(3,2)!=-1.0 || _matrix(3,3)!=0.0){
        //std::cout << "TH: " << _matrix(0,0) << " " <<  _matrix(1,3) << " " << _matrix(2,3) << " " <<_matrix(3,3) << std::endl;
        //std::cout << "TX: " << _matrix(0,0) << " " << _matrix(3,1) << " " <<_matrix(3,2) << " " <<_matrix(3,3) << std::endl;
        return false;
    }


    zNear = _matrix(2,3)/ (_matrix(2,2)-1.0);
    zFar = _matrix(2,3) / (1.0+_matrix(2,2));

    left = zNear * (_matrix(0,2)-1.0) / _matrix(0,0);
    right = zNear * (1.0+_matrix(0,2)) / _matrix(0,0);

    top = zNear * (1.0+_matrix(1,2)) / _matrix(1,1);
    bottom = zNear * (_matrix(1,2)-1.0) / _matrix(1,1);

    return true;
}

std::pair<double,double> ProjectionMatrix::getClipPlanes() const {
    double zNear = _matrix(2,3)/ (_matrix(2,2)-1.0);
    double zFar = _matrix(2,3) / (1.0+_matrix(2,2));
    return std::make_pair(zNear,zFar);
}

bool ProjectionMatrix::getOrtho(double& left, double& right,
                      double& bottom, double& top,
                      double& zNear, double& zFar) const
{
    if (_matrix(3,0)!=0.0 || _matrix(3,1)!=0.0 || _matrix(3,2)!=0.0 || _matrix(3,3)!=1.0)
        return false;

    zNear = (_matrix(2,3)+1.0) / _matrix(2,2);
    zFar = (_matrix(2,3)-1.0) / _matrix(2,2);

    left = -(1.0+_matrix(0,3)) / _matrix(0,0);
    right = (1.0-_matrix(0,3)) / _matrix(0,0);

    bottom = -(1.0+_matrix(1,3)) / _matrix(1,1);
    top = (1.0-_matrix(1,3)) / _matrix(1,1);

    return true;
}


void ProjectionMatrix::setPerspective(double fovy, double aspectRatio, double zNear, double zFar)
{
    // calculate the appropriate left, right etc.
    double tan_fovy = tan( rw::math::Deg2Rad*(fovy*0.5) );
    double right  =  tan_fovy * aspectRatio * zNear;
    double left   = -right;
    double top    =  tan_fovy * zNear;
    double bottom =  -top;
    setFrustum(left,right,bottom,top,zNear,zFar);
    //double fovyt, aspectRatiot, zneart, zfart;
    //ProjectionMatrix::getPerspective(fovyt, aspectRatiot, zneart, zfart);
}

bool ProjectionMatrix::getPerspective(double& fovy,double& aspectRatio, double& zNear, double& zFar) const
{
    double right  =  0.0;
    double left   =  0.0;
    double top    =  0.0;
    double bottom =  0.0;
    if ( getFrustum(left,right,bottom,top,zNear,zFar) )
    {
        fovy = (atan(top/zNear)-atan(bottom/zNear))*Rad2Deg;
        aspectRatio = (right-left)/(top-bottom);
        return true;
    }
    return false;
}

ProjectionMatrix ProjectionMatrix::makePerspective(double fovy, double aspectRatio, double zNear, double zFar)
{
    ProjectionMatrix matrix;
    matrix.setPerspective(fovy,aspectRatio,zNear,zFar);
    return matrix;
}

ProjectionMatrix ProjectionMatrix::makePerspective(double fovy, double width, double height, double zNear, double zFar)
{
    ProjectionMatrix matrix;
    matrix.setPerspective(fovy,width/height,zNear,zFar);
    return matrix;
}


ProjectionMatrix ProjectionMatrix::makeOrtho(double left, double right,
                                  double bottom, double top,
                                  double zNear, double zFar)
{
    ProjectionMatrix matrix;
    matrix.setOrtho(left, right, bottom, top, zNear, zFar);
    return matrix;
}
