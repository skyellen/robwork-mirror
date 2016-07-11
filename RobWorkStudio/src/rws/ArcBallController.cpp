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

#include "ArcBallController.hpp" // ArcBallController header

#include <rwlibs/os/rwgl.hpp>

#include <math.h> // Needed for sqrtf
#include <rw/math/Math.hpp>

#include <QMouseEvent>

using namespace rw;
using namespace rw::math;
using namespace rws;

rw::math::Vector3D<> ArcBallController::mapToSphere(double x, double y) const
{
    //Adjust point coords and scale down to range of [-1 ... 1]
    float adjust = _adjustWidth;
    if(adjust<_adjustHeight)
        adjust = _adjustHeight;
    float xTmp = -( (_centerPt[0]-x) * adjust);
    float yTmp = (_centerPt[1]-y) * adjust;

    //std::cout << x << ";" << _centerPt[0] << ";" << _adjustWidth << ";" << xTmp << std::endl;
    //std::cout << y << ";" << _centerPt[1] << ";" << _adjustHeight << ";" << yTmp << std::endl;

    //Compute the square of the length of the vector to the point from the center
    float length = xTmp*xTmp + yTmp*yTmp;

    //If the point is mapped outside of the sphere... (length > radius squared)
    if (length > 1.0f) {
        //Compute a normalizing factor (radius / sqrt(length))
        float norm    = 1.0f / sqrtf(length);

        //Return the "normalized" vector, a point on the sphere
        return Vector3D<>(xTmp*norm, yTmp*norm, 0.0f);
    }
    //Else it's on the inside
    //Return a vector to a point mapped inside the sphere sqrt(radius squared - length)
    return Vector3D<>(xTmp, yTmp, sqrtf(1.0f - length));
}

//Create/Destroy
ArcBallController::ArcBallController(double NewWidth, double NewHeight):
	_centerPt(NewWidth/2.0,NewHeight/2.0),
    _stVec(0.0f,0.0f,0.0f),
    _enVec(0.0f,0.0f,0.0f)
{
    _viewTransform = Transform3D<>::makeLookAt(Vector3D<>(5,5,5),Vector3D<>::zero(),Vector3D<>::z());

    //Set initial bounds
    this->setBounds(NewWidth, NewHeight);
}

void ArcBallController::setBounds(double NewWidth, double NewHeight)
{
    //std::cout << "setBounds" << std::endl;
    _width = NewWidth;
    _height = NewHeight;

    // Set adjustment factor for width/height
    _adjustWidth  = 1.0f / ((NewWidth  - 1.0f));
    _adjustHeight = 1.0f / ((NewHeight - 1.0f));
    //std::cout << "setBounds" << std::endl;
}

void ArcBallController::click(float x, float y){
    //Map the point to the sphere
    _stVec = mapToSphere(x,y);
}

void ArcBallController::draw()
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
rw::math::Quaternion<double> ArcBallController::drag(float x, float y)
{
    //Map the point to the sphere
	Vector3D<> enVecTmp = this->mapToSphere(x, y);
    //Return the quaternion equivalent to the rotation
    //Compute the vector perpendicular to the begin and end vectors
    math::Vector3D<>  perp = cross(_stVec, enVecTmp);

    //Compute the length of the perpendicular vector
    if ( perp.norm2() > 1.0e-5){ //if its non-zero
    	// update the enVec
    	_enVec = enVecTmp;

        //We're ok, so return the perpendicular vector as the transform after all
        //In the quaternion values, w is cosine (theta / 2), where theta is rotation angle
        math::Quaternion<> tmpQuat(
            perp(0), perp(1), perp(2), dot(_stVec, _enVec));

        return tmpQuat;
    }
    //std::cout << "Quaternion<>(0.0f,0.0f,0.0f,0.0f)" << std::endl;

    //if its zero
    //The begin and end vectors coincide, so return an identity transform
    return Quaternion<>(0.0f,0.0f,0.0f,1.0f);
}


void ArcBallController::handleEvent(QEvent* e){
    //std::cout << "T: " << _viewTransform << "\n" ;
    if( e->type() == QEvent::MouseButtonPress){

        QMouseEvent *event = static_cast<QMouseEvent*>(e);

        _lastPos(0) = event->x();
        _lastPos(1) = event->y();

        click(event->x(), event->y());

    } else if(e->type() == QEvent::MouseMove){

        QMouseEvent *event = static_cast<QMouseEvent*>(e);
        if (event->buttons() == Qt::LeftButton) {
            if (event->modifiers() == Qt::ControlModifier) {
                //_viewPos(2) -= (event->y()-_lastPos(1))/_height*10;
                //std::cout << ((event->y()-_lastPos(1))/_adjustHeight*10) << std::endl;
                //std::cout << event->y() << "-" << _lastPos(1) << "/" << _adjustHeight*10 << " " << std::endl;
                Vector3D<> translateVector(0, 0, -(event->y()-_lastPos(1))/_height*10 );
                _viewTransform.P() -= _viewTransform.R()*translateVector;

            } else { // The mouse is being dragged
                double rx = (event->x());
                double ry = (event->y());

                // Update End Vector And Get Rotation As Quaternion
                Quaternion<double> quat = drag(rx, ry);
                EAA<> eaa = Math::quaternionToEAA(quat);
                Transform3D<> &wTc = _viewTransform;

                Transform3D<> wTp( _pivotPoint, Rotation3D<>::identity() );
                Transform3D<> pnTp(Vector3D<>(0,0,0), inverse((wTc.R()*eaa).toRotation3D()));

                //Transform3D<> pTc = inverse(wTp)*wTc;

                wTc = wTp * pnTp * inverse(wTp)*wTc;

                click(rx,ry);
            }
        }
        if (event->buttons() == Qt::RightButton) {
            Vector3D<> translateVector((event->x()-_lastPos(0))/_width*10, -((event->y()-_lastPos(1))/_height*10), 0 );
            _viewTransform.P() -= _viewTransform.R()*translateVector;
            _centerPt[0] += event->x()-_lastPos(0);
            _centerPt[1] += event->y()-_lastPos(1);
        }
        _lastPos(0) = event->x();
        _lastPos(1) = event->y();

    } else if( e->type() == QEvent::Wheel){

        QWheelEvent *event = static_cast<QWheelEvent*>(e);
        Vector3D<> translateVector(0, 0, event->delta()/(240.0) );
        _viewTransform.P() -= _viewTransform.R()*translateVector;
    }
}


void ArcBallController::setCenter(const rw::math::Vector3D<>& center,
                           const rw::math::Vector2D<>& screenCenter){
    //_viewTransform.P() -= _viewTransform.R()*_pivotPoint;
    _pivotPoint = center;
    _centerPt = screenCenter;
}

rw::math::Transform3D<> ArcBallController::getTransform() const{
    return _viewTransform;
}

void ArcBallController::setTransform(const rw::math::Transform3D<>& t3d){
    _viewTransform = t3d;
}

