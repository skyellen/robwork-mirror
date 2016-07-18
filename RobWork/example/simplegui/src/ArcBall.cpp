#include <GL/gl.h>												// Header File For The OpenGL32 Library
#include <GL/glu.h>												// Header File For The GLu32 Library

#include <math.h>                                               // Needed for sqrtf
#include "ArcBall.hpp"                                            // ArcBall header

using namespace rw;
using namespace rw::math;

//Arcball sphere constants:
//Diameter is       2.0f
//Radius is         1.0f
//Radius squared is 1.0f

void ArcBall::mapToSphere(const std::pair<float,float>& newPt, math::Vector3D<float>& res) const {
    std::pair<float,float> tempPt;
    GLfloat length;

    //Copy paramter into temp point
    tempPt = newPt;

    //Adjust point coords and scale down to range of [-1 ... 1]
    tempPt.first  =        ( (tempPt.first+_centerPt.first) * _adjustWidth)  - 1.0f;
    tempPt.second = 1.0f - ( (tempPt.second+_centerPt.second) * _adjustHeight);

    //Compute the square of the length of the vector to the point from the center
    length      = (tempPt.first * tempPt.first) + (tempPt.second * tempPt.second);

    //If the point is mapped outside of the sphere... (length > radius squared)
    if (length > 1.0f) {
        GLfloat norm;
        //Compute a normalizing factor (radius / sqrt(length))
        norm    = 1.0f / sqrtf(length);

        //Return the "normalized" vector, a point on the sphere
        math::Vector3D<float> tmpV3d(tempPt.first * norm, tempPt.second * norm, 0.0f);
        res = tmpV3d;
    } else {   //Else it's on the inside
        //Return a vector to a point mapped inside the sphere sqrt(radius squared - length)
        math::Vector3D<float> tmpV3d(tempPt.first, tempPt.second, sqrtf(1.0f - length));
        res = tmpV3d;
    }

}

//Create/Destroy
ArcBall::ArcBall(GLfloat NewWidth, GLfloat NewHeight):
    _stVec(0.0f,0.0f,0.0f),_enVec(0.0f,0.0f,0.0f),_centerPt(0.0f,0.0f)
{
    //Set initial bounds
    this->setBounds(NewWidth, NewHeight);
    _sphere = gluNewQuadric();
}

//Mouse down
void ArcBall::click(const std::pair<float,float>& newPt)
{
    //Map the point to the sphere
    this->mapToSphere(newPt, _stVec);
}

void ArcBall::click(float x, float y){
    this->click(std::pair<float,float>(x,y));
}


//Mouse drag, calculate rotation
void ArcBall::drag(const std::pair<float,float>& newPt, math::Quaternion<float>& newRot)
{
    //Map the point to the sphere
    this->mapToSphere(newPt, _enVec);

    //Return the quaternion equivalent to the rotation
    math::Vector3D<float>  perp;

    //Compute the vector perpendicular to the begin and end vectors
//    Vector3fCross(&Perp, &this->StVec, &this->EnVec);
    perp = cross(_stVec,_enVec);

    //Compute the length of the perpendicular vector
    if ( perp.norm2() > Epsilon)    //if its non-zero
    {
        //We're ok, so return the perpendicular vector as the transform after all
        //In the quaternion values, w is cosine (theta / 2), where theta is rotation angle
        math::Quaternion<float> tmpQuat(perp(0),perp(1),perp(2),inner_prod(_stVec.m(),_enVec.m()));
        newRot = tmpQuat;
    }
    else //if its zero
    {
        //The begin and end vectors coincide, so return an identity transform
        math::Quaternion<float> tmpQuat(0.0f,0.0f,0.0f,0.0f);
        newRot = tmpQuat;
    }
}

