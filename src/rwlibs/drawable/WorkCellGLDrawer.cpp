/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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

#include <vector>

using namespace rw::math;
using namespace rwlibs::drawable;
using namespace rw::models;
using namespace rw::kinematics;

typedef std::vector<Drawable*> DrawableList;

namespace
{
    void GLTransform(const Transform3D<>& transform)
    {
        GLfloat gltrans[16];
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++)
                gltrans[j + 4 * k] = (float)transform(j, k);
            gltrans[12 + j] = (float)transform(j, 3);
        }
        gltrans[3] = gltrans[7] = gltrans[11] = 0;
        gltrans[15] = 1;
        glMultMatrixf(gltrans);
    }
}

WorkCellGLDrawer::~WorkCellGLDrawer()
{
    clearCache();
}

void WorkCellGLDrawer::clearCache()
{
    BOOST_FOREACH(FrameMap::const_reference entry, _frameMap) {
        BOOST_FOREACH(Drawable* da, entry.second) {
            delete da;
        }
    }
    _frameMap.clear();
}

void WorkCellGLDrawer::draw(const State& state, WorkCell* workcell)
{
    Frame* world = workcell->getWorldFrame();
    draw(state, world);
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

void WorkCellGLDrawer::drawCameraView(const State& state, Frame* camera)
{
    // Find the root:
    Frame* rootFrame = camera;
    while (rootFrame->getParent(state)) {
        rootFrame = rootFrame->getParent(state);
    }

    // Draw everything from the root and down at a new transform:
    glPushMatrix();
    GLTransform(inverse(Kinematics::worldTframe(camera, state)));
    draw(state, rootFrame);
    glPopMatrix();
}

void WorkCellGLDrawer::draw(const State& state, const Frame* frame)
{
    glPushMatrix();

    GLTransform(frame->getTransform(state));
    const DrawableList& drawables = getDrawablesForFrame(frame);
    BOOST_FOREACH(Drawable* da, drawables) { da->draw(); }

    BOOST_FOREACH(const Frame& child, frame->getChildren(state)) {
        draw(state, &child);
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
	            Drawable* drawable = DrawableFactory::getDrawable(info.getId());

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
