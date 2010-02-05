#ifndef RENDERFORCE_HPP_
#define RENDERFORCE_HPP_

#include <rwlibs/drawable/Render.hpp>
#include <rwlibs/os/rwgl.hpp>
#include <rw/math/Vector3D.hpp>

class RenderForce : public rwlibs::drawable::Render
{
public:
	
    RenderForce():
        _pos(0,0,0),_nforce(0,0,0),_tforce(0.0)
    {
        _quadratic=gluNewQuadric();
    }
	
	virtual ~RenderForce(){};
	
    /**
     * @brief The point of contact
     */
    void setPosition(rw::math::Vector3D<> pos){
        _pos = rw::math::cast<float>( pos );
        //rw::math::Vector3D<> zAxis = pos;
        //rw::math::Vector3D<> xAxis(1,0,0);
    }
    
    /**
     * @brief the Force acting on the contact point
     */
    void setForce(rw::math::Vector3D<> nforce, double tForce, double staticFriction){
        _nforce = rw::math::cast<float>( nforce ) ;
        _tforce = tForce;
        //std::cout << "Force: " << force <<  std::endl;
    }
    
    /**
     * @copydoc Render::draw
     */
    void draw(Render::DrawType type, double alpha) const{
        //if( _force.norm2()<0.001 )
        //    return;

        glPushMatrix();
        
        glColor3f(1.0, 0.0, 0.0);
        glTranslatef(_pos(0),_pos(1),_pos(2));// Center The Cone
        gluSphere( _quadratic, 0.001, 32, 32);    // Draw Our Sphere

        glBegin(GL_LINES);
         glColor3f(1.0, 0.0, 0.0);
         glVertex3d(0,0,0);
         glVertex3d(_nforce(0),_nforce(1),_nforce(2));         
        glEnd();
        
        glPopMatrix();
        
/*    	double size = _nforce.norm2();
    	glPushMatrix();
        glColor3f(1.0, 0.0, 0.0);
        glTranslatef(_pos(0),_pos(1),_pos(2));// Center The Cone
        gluCylinder(
            _quadratic,
            0.001,
            size*_friction,
            size,
            32,
            32);    // Draw Our Cylinder
        
        glPopMatrix();
    	glPushMatrix();
        glColor3f(0.0, 0.0, 1.0);
        glTranslatef(_pos(0),_pos(1),_pos(2));// Center The Cone
        gluCylinder(
            _quadratic,
            0.001,
            _tforce,
            size,
            32,
            32);    // Draw Our Cylinder
        
        glPopMatrix();*/

    }
    	
private:
    rw::math::Vector3D<float> _pos, _nforce;
    double _tforce, _friction;
    GLUquadricObj *_quadratic;

};

#endif /*RenderForce_HPP_*/
