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


#include "WorkCellGLDrawer.hpp"

#include <rwlibs/os/rwgl.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/models/Accessor.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>

#include <boost/foreach.hpp>

#include "Render.hpp"
#include "Drawable.hpp"
#include "DrawableFactory.hpp"
#include "DrawableUtil.hpp"
#include <vector>

using namespace rw::math;
using namespace rwlibs::drawable;
using namespace rw::models;
using namespace rw::kinematics;

typedef std::vector<rwlibs::drawable::Drawable*> DrawableList;

WorkCellGLDrawer::~WorkCellGLDrawer()
{
    clearCache();
}

void WorkCellGLDrawer::clearCache()
{
    BOOST_FOREACH(FrameMap::const_reference entry, _frameMap) {
        BOOST_FOREACH(rwlibs::drawable::Drawable* da, entry.second) {
            delete da;
        }
    }
    _frameMap.clear();
}

void WorkCellGLDrawer::draw(const State& state, WorkCell* workcell, unsigned int dmask)
{
    Frame* world = workcell->getWorldFrame();
    draw(state, world, dmask);
}

DrawableList WorkCellGLDrawer::getAllDrawables(
    const State& state, WorkCell* workcell)
{
    DrawableList result;
    getAllDrawables(state, workcell->getWorldFrame(), result);
    return result;
}

void WorkCellGLDrawer::getAllDrawables(
    const State& state,
    const Frame* frame,
    DrawableList& result)
{
    const DrawableList& drawables = getDrawablesForFrame(frame);
    result.insert(result.end(), drawables.begin(), drawables.end());
    BOOST_FOREACH(const Frame& child, frame->getChildren(state)) {
        getAllDrawables(state, &child, result);
    }
}

void WorkCellGLDrawer::drawCameraView(const State& state, Frame* camera, unsigned int dmask)
{
    // Find the root:
    Frame* rootFrame = camera;
    while (rootFrame->getParent(state)) {
        rootFrame = rootFrame->getParent(state);
    }

    // Draw everything from the root and down at a new transform:
    glPushMatrix();
    DrawableUtil::multGLTransform( inverse(Kinematics::worldTframe(camera, state)) );
    draw(state, rootFrame, dmask);
    glPopMatrix();
}

void WorkCellGLDrawer::draw(const State& state, const Frame* frame, unsigned int dmask)
{
    glPushMatrix();

    DrawableUtil::multGLTransform( frame->getTransform(state) );
    const DrawableList& drawables = getDrawablesForFrame(frame);
    BOOST_FOREACH(Drawable* da, drawables) { da->draw(dmask); }

    BOOST_FOREACH(const Frame& child, frame->getChildren(state)) {
        draw(state, &child, dmask);
    }

    glPopMatrix();
}

void WorkCellGLDrawer::drawAndSelect(const rw::kinematics::State& state,
                   rw::models::WorkCell* workcell, unsigned int dmask){

    Frame* world = workcell->getWorldFrame();
    drawAndSelect(state, world, dmask);
}


void WorkCellGLDrawer::drawAndSelect(const rw::kinematics::State& state,
                   const Frame* frame, unsigned int dmask)
{


    glPushMatrix();
    DrawableUtil::multGLTransform(frame->getTransform(state));
    const DrawableList& drawables = getDrawablesForFrame(frame);

    glPushName( (GLuint) frame->getID() );
    BOOST_FOREACH(Drawable* da, drawables) { da->draw(dmask); }
    glPopName();

    BOOST_FOREACH(const Frame& child, frame->getChildren(state)) {
        drawAndSelect(state, &child, dmask);
    }

    glPopMatrix();

}


namespace
{
    DrawableList getFrameDrawables(const Frame& frame)
    {
    	DrawableList result;
        if (Accessor::drawableModelInfo().has(frame)) {
            // Load the drawable:
        	const std::vector<DrawableModelInfo> infos =
                Accessor::drawableModelInfo().get(frame);

        	BOOST_FOREACH(const DrawableModelInfo &info, infos) {
	            // TODO: handle multiple drawables
        	    rwlibs::drawable::Drawable* drawable = DrawableFactory::getDrawable(info.getId());

	            if (drawable) {
	                // Set various properties for the drawable:
	            	drawable->setTransform(info.getTransform());
	            	drawable->setScale((float)info.getGeoScale());

	                if (info.isHighlighted())
	                    drawable->setHighlighted(true);

	                if (info.isWireMode())
	                    drawable->setDrawType(Render::WIRE);

	                result.push_back(drawable);
	            } else {
	                RW_WARN(
	                    "NULL drawable returned by loadDrawableFile() for GeoID "
	                    << info.getId());
	            }
        	}
        }
        return result;
    }
}

const DrawableList& WorkCellGLDrawer::getDrawablesForFrame(const Frame* frame)
{
    RW_ASSERT(frame);

    DrawableList& seq = _frameMap[frame];
    if (seq.empty()) {
        DrawableList drawables = getFrameDrawables(*frame);
        if (!drawables.empty())
        	seq = drawables;
    }

    return seq;
}

void WorkCellGLDrawer::addDrawableToFrame(Frame* frame, Drawable* drawable)
{
    _frameMap[frame].push_back(drawable);
}

void WorkCellGLDrawer::removeDrawableFromFrame(Frame* frame, Drawable* drawable)
{
    DrawableList& seq = _frameMap[frame];
    seq.erase(
        std::remove(seq.begin(), seq.end(), drawable),
        seq.end());
}

void WorkCellGLDrawer::lock(){
	_mutex.lock();
}

void WorkCellGLDrawer::unlock(){
	_mutex.unlock();
}

