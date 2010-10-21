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

#include "RenderGhost.hpp"

#include <boost/foreach.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/drawable/Drawable.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::drawable;
using namespace rwsim::drawable;

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

RenderGhost::RenderGhost(rw::kinematics::Frame *frame,
		WorkCellGLDrawer *drawer,
		size_t N):
	_drawer(drawer),
	_states(N)
{
	_frames.push_back(frame);
	_drawFrame = new RenderFrame(0.2);
}

RenderGhost::RenderGhost(std::list<rw::kinematics::Frame*> frames, WorkCellGLDrawer *drawer, size_t N):
	_frames(frames), _drawer(drawer), _states(N)
{
	_drawFrame = new RenderFrame(0.2);
}


RenderGhost::~RenderGhost(){}

void RenderGhost::addState(const rw::kinematics::State& state){
	_states.push_back(state);
}

void RenderGhost::setMaxBufferSize(size_t size){
	_states.set_capacity(size);
}

void RenderGhost::draw(DrawType type, double alpha) const {
	BOOST_FOREACH(Frame *frame, _frames){
		const std::vector<Drawable::Ptr>& toDraw = _drawer->getDrawablesForFrame( frame );
		double alphaStep = 1.0/(double)_states.size();
		double alpha = 0;
		BOOST_FOREACH(Drawable::Ptr drawable, toDraw){
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


