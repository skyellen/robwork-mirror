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

#include <rw/graphics/DrawableNode.hpp>
#include <rw/graphics/WorkCellScene.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/opengl/RenderFrame.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::graphics;
using namespace rwsim::drawable;
using namespace rwlibs::opengl;

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
		WorkCellScene::Ptr drawer,
		size_t N):
	_drawer(drawer),
	_states(N)
{
	_frames.push_back(frame);
	_drawFrame = new RenderFrame(0.2f);
}

RenderGhost::RenderGhost(std::list<rw::kinematics::Frame*> frames, WorkCellScene::Ptr drawer, size_t N):
	_frames(frames), _drawer(drawer), _states(N)
{
	_drawFrame = new RenderFrame(0.2f);
}


RenderGhost::~RenderGhost(){}

void RenderGhost::addState(const rw::kinematics::State& state){
	_states.push_back(state);
}

void RenderGhost::setMaxBufferSize(size_t size){
	_states.set_capacity(size);
}

void RenderGhost::draw(const DrawableNode::RenderInfo& info, DrawType type, double alpha) const {

    BOOST_FOREACH(Frame *frame, _frames){
		const std::vector<DrawableNode::Ptr>& toDraw = _drawer->getDrawables( frame );
		double alphaStep = 1.0/(double)_states.size();
		double alpha = 0;
		BOOST_FOREACH(DrawableNode::Ptr drawable, toDraw){
			alpha += alphaStep;
			drawable->setTransparency((float)alpha);
			//drawable->setDrawType(Drawable::WIRE);
    		for(size_t i=0; i<_states.size(); i++){
    			glPushMatrix();
    			Transform3D<> t3d = Kinematics::worldTframe(frame, _states[i]);
    			GLTransform(t3d);
    			glColor3f((GLfloat)(alpha),0,0);
    			drawable->draw(info);
    			_drawFrame->draw(info, type, alpha);
    			glPopMatrix();
    		}
    		drawable->setTransparency(1.0);
    		//drawable->setDrawType(Drawable::SOLID);
		}
	}
}


