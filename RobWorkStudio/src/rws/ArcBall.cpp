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


#include <rwlibs/os/rwgl.hpp>

#include <math.h> // Needed for sqrtf
#include "ArcBall.hpp" // ArcBall header

// utility macros
// assuming IEEE-754(GLfloat), which i believe has max precision of 7 bits
#define Epsilon 1.0e-5

using namespace rw;
using namespace rw::math;
using namespace rws;

rw::math::Vector3D<float> ArcBall::mapToSphere(float x, float y) const
{
    //Adjust point coords and scale down to range of [-1 ... 1]
    float xTmp =        ( (x+_centerPt[0]) * _adjustWidth)  - 1.0f;
    float yTmp = 1.0f - ( (y+_centerPt[1]) * _adjustHeight);

    //Compute the square of the length of the vector to the point from the center
    float length = xTmp*xTmp + yTmp*yTmp;

    //If the point is mapped outside of the sphere... (length > radius squared)
    if (length > 1.0f) {
        //Compute a normalizing factor (radius / sqrt(length))
        float norm    = 1.0f / sqrtf(length);

        //Return the "normalized" vector, a point on the sphere
        return Vector3D<float>(xTmp*norm, yTmp*norm, 0.0f);
    }
    //Else it's on the inside
    //Return a vector to a point mapped inside the sphere sqrt(radius squared - length)
    return Vector3D<float>(xTmp, yTmp, sqrtf(1.0f - length));
}

//Create/Destroy
ArcBall::ArcBall(GLfloat NewWidth, GLfloat NewHeight):
	_centerPt(0.0f,0.0f),
    _stVec(0.0f,0.0f,0.0f),
    _enVec(0.0f,0.0f,0.0f)

{

    //Set initial bounds
    this->setBounds(NewWidth, NewHeight);
}

void ArcBall::click(float x, float y){
    //Map the point to the sphere
    _stVec = mapToSphere(x,y);
}

void ArcBall::draw()
{
    glBegin(GL_LINE_LOOP);
    for (float angle = 0; angle <= 2 * 3.142f; angle += 3.142f / 30)
    {
        float x =  cos (angle);
        float z =  sin (angle);
        glVertex3f( x, 0.0, z);
    }
    glEnd();

    glBegin(GL_LINE_LOOP);
    for (float angle = 0; angle <= 2 * 3.142f; angle += 3.142f / 30)
    {
        float x =  cos(angle);
        float y =  sin(angle);
        glVertex3f(x, y, 0);
    }
    glEnd();
}

//Mouse drag, calculate rotation
rw::math::Quaternion<float> ArcBall::drag(float x, float y)
{
    //Map the point to the sphere
	Vector3D<float> enVecTmp = this->mapToSphere(x, y);

    //Return the quaternion equivalent to the rotation
    //Compute the vector perpendicular to the begin and end vectors
    math::Vector3D<float>  perp = cross(_stVec, enVecTmp);

    //Compute the length of the perpendicular vector
    if ( perp.norm2() > Epsilon){ //if its non-zero
    	// update the enVec
    	_enVec = enVecTmp;

        //We're ok, so return the perpendicular vector as the transform after all
        //In the quaternion values, w is cosine (theta / 2), where theta is rotation angle
        math::Quaternion<float> tmpQuat(
            perp(0), perp(1), perp(2), dot(_stVec, _enVec));
        return tmpQuat;
    }
    //if its zero
    //The begin and end vectors coincide, so return an identity transform
    return Quaternion<float>(0.0f,0.0f,0.0f,0.0f);
}
