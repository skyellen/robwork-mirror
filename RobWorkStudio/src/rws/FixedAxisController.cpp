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
#include "FixedAxisController.hpp"

using namespace rw;
using namespace rw::math;
using namespace rws;

//Create/Destroy
FixedAxisController::FixedAxisController(GLfloat NewWidth, GLfloat NewHeight):
	_centerPt(0.0f,0.0f),
    _stVec(0.0f,0.0f,0.0f),
    _enVec(0.0f,0.0f,0.0f)
{
    _viewTransform = inverse( Transform3D<>::makeLookAt(Vector3D<>(5,5,3),Vector3D<>::zero(),Vector3D<>::z()) );

    //Set initial bounds
    this->setBounds(NewWidth, NewHeight);
}

void FixedAxisController::click(float x, float y){
    //Map the point to the sphere
    _stVec = mapToSphere(x,y);
}

void FixedAxisController::draw()
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
rw::math::Quaternion<double> FixedAxisController::drag(float x, float y)
{
    // all rotation around

    //Map the point to the sphere
	Vector3D<float> enVecTmp = this->mapToSphere(x, y);

    //Return the quaternion equivalent to the rotation
    //Compute the vector perpendicular to the begin and end vectors
    math::Vector3D<float>  perp = cross(_stVec, enVecTmp);

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
    //if its zero
    //The begin and end vectors coincide, so return an identity transform
    return Quaternion<>(0.0f,0.0f,0.0f,0.0f);
}


void FixedAxisController::handleEvent(QEvent* e){
    std::cout << "T: " << _viewTransform << "\n" ;
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
                //Quaternion<double> quat = drag(rx, ry);

                // movement in x translate to rotation around world z-axis through pivot
                // movement in y translate to rotation around camera x-axis through pivot

                // Convert Quaternion Into Rotation3D
                Rotation3D<double> thisRot = quat.toRotation3D();

                // the rotation is defined around pivot center
                Transform3D<> wTp = Transform3D<>::identity();
                wTp.P() = _pivotPoint;
                Transform3D<> p1Tp2(Vector3D<>(0,0,0),thisRot);

                Transform3D<> pTc = /*inverse(wTp)* */_viewTransform;
                pTc = inverse(p1Tp2)*pTc;
                _viewTransform = /*wTp **/ pTc;

                click(rx,ry);
            }
        }
        if (event->buttons() == Qt::RightButton) {
            //_viewPos(0) += (event->x()-_lastPos(0))/_width*10;
            //_viewPos(1) -= (event->y()-_lastPos(1))/_height*10;
            Vector3D<> translateVector((event->x()-_lastPos(0))/_width*10, -((event->y()-_lastPos(1))/_height*10), 0 );
            _viewTransform.P() -= _viewTransform.R()*translateVector;
        }
        _lastPos(0) = event->x();
        _lastPos(1) = event->y();

    } else if( e->type() == QEvent::Wheel){

        QWheelEvent *event = static_cast<QWheelEvent*>(e);
        Vector3D<> translateVector(0, 0, event->delta()/(240.0) );
        _viewTransform.P() -= _viewTransform.R()*translateVector;

        //float distToPivot = norm_2( _pivotPoint - _viewPos );
        // somehow compensate when we are very close to objects
        //_zoomFactor += event->delta()/(120.0);

        //if(_zoomFactor>=0){
        //    _zoomScale = 1.0+0.5*_zoomFactor;
        //} else {
        //    _zoomScale = -1.0/(0.5*_zoomFactor-1);
        //}
    }
}

void FixedAxisController::updateCenter(double x, double y, double z){
    _viewTransform.P() -= _viewTransform.R()*_pivotPoint;
    _pivotPoint(0) = x;
    _pivotPoint(1) = y;
    _pivotPoint(2) = z;
    _viewTransform.P() += _viewTransform.R()*_pivotPoint;


    // update FixedAxisController center
    setCenter((float)x, (float)y);
}

rw::math::Transform3D<> FixedAxisController::getTransform() const{
    return _viewTransform;
}

void FixedAxisController::setTransform(const rw::math::Transform3D<>& t3d){
    _viewTransform = t3d;
}

void FixedAxisController::setBounds(GLfloat NewWidth, GLfloat NewHeight)
{
    _width = NewWidth;
    _height = NewHeight;

    // Set adjustment factor for width/height
    _adjustWidth  = 1.0f / ((NewWidth  - 1.0f) * 0.5f);
    _adjustHeight = 1.0f / ((NewHeight - 1.0f) * 0.5f);
}

