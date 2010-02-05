#include "RenderGhost.hpp"

#include <boost/foreach.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/drawable/Drawable.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::drawable;

namespace
{
    void GLTransform(const Transform3D<>& transform)
    {
        GLfloat gltrans[16];
        for (int j = 0; j<3; j++) {
            for (int k = 0; k<3; k++)
                gltrans[j+4*k] = (float)transform(j,k);
            gltrans[12+j] = (float)transform(j, 3);
        }
        gltrans[3] = gltrans[7] = gltrans[11] = 0;
        gltrans[15] = 1;
        glMultMatrixf(gltrans);
    }
}

RenderGhost::RenderGhost(rw::kinematics::Frame *frame, WorkCellGLDrawer *drawer):
	_drawer(drawer)
{
	_frames.push_back(frame);
	_drawFrame = new RenderFrame(0.2);
}

RenderGhost::RenderGhost(std::list<rw::kinematics::Frame*> frames, WorkCellGLDrawer *drawer):
	_frames(frames), _drawer(drawer)
{
	_drawFrame = new RenderFrame(0.2);
}


RenderGhost::~RenderGhost(){}

void RenderGhost::addState(rw::kinematics::State& state){
	_states.push_back(state);
}

void RenderGhost::draw(DrawType type, double alpha) const {
	BOOST_FOREACH(Frame *frame, _frames){
		const std::vector<Drawable*>& toDraw = _drawer->getDrawablesForFrame( frame );
		double alphaStep = 1.0/(double)_states.size();
		double alpha = 0;
		BOOST_FOREACH(Drawable *drawable, toDraw){
			alpha += alphaStep;
			drawable->setAlpha(alpha);
			//drawable->setDrawType(Drawable::WIRE);
    		for(size_t i=0; i<_states.size(); i++){
    			glPushMatrix();
    			Transform3D<> t3d = Kinematics::worldTframe(frame, _states[i]);
    			GLTransform(t3d);
    			glColor3f(alpha,0,0);
    			drawable->draw();
    			_drawFrame->draw(type, alpha);
    			glPopMatrix();
    		}
    		drawable->setAlpha(1.0);
    		//drawable->setDrawType(Drawable::SOLID);
		}
	}
}
