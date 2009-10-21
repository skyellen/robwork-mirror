#ifndef _ArcBall_h
#define _ArcBall_h

#include <GL/gl.h>												// Header File For The OpenGL32 Library
#include <GL/glu.h>												// Header File For The GLu32 Library

#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>

//utility macros
//assuming IEEE-754(GLfloat), which i believe has max precision of 7 bits
# define Epsilon 1.0e-5

class ArcBall {
    protected:
        inline void mapToSphere(const std::pair<float,float>& newPt, rw::math::Vector3D<float>& res) const;

    public:
        //Create/Destroy
        ArcBall(GLfloat NewWidth, GLfloat NewHeight);
        ~ArcBall() { /* nothing to do */ };

        //Set new bounds
        inline void setBounds(GLfloat NewWidth, GLfloat NewHeight)
        {
            //Set adjustment factor for width/height
            _adjustWidth  = 1.0f / ((NewWidth  - 1.0f) * 0.5f);
            _adjustHeight = 1.0f / ((NewHeight - 1.0f) * 0.5f);
        }

        void setCenter(const std::pair<float,float>& centerPt){
            _centerPt = centerPt;
        }

        //Mouse down
        void click(const std::pair<float,float>& newPt);
        void click(float x, float y);

        //Mouse drag, calculate rotation
        void drag(const std::pair<float,float>& newPt, rw::math::Quaternion<float>& newRot);

    protected:
        rw::math::Vector3D<float> _stVec;          //Saved click vector
        rw::math::Vector3D<float> _enVec;          //Saved drag vector
        std::pair<float,float> _centerPt; //Center of the ball
        GLfloat _adjustWidth;    //Mouse bounds width
        GLfloat _adjustHeight;   //Mouse bounds height
        GLUquadricObj* _sphere;

};

#endif

