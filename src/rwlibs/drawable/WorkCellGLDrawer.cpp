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

WorkCellGLDrawer::~WorkCellGLDrawer()
{
    clearCache();
}

void WorkCellGLDrawer::clearCache()
{
    typedef FrameMap::const_iterator FI;
    for (FI fit = _frameMap.begin(); fit != _frameMap.end(); ++fit) {
        typedef DrawableList::const_iterator DI;
        for (DI dit = fit->second.begin(); dit != fit->second.end(); ++dit) {
            delete *dit;
        }
    }

    _frameMap.clear();
}

void WorkCellGLDrawer::draw(const State& state, WorkCell* workcell)
{
    Frame* world = workcell->getWorldFrame();
    draw(state, world);
}

DrawableList WorkCellGLDrawer::getAllDrawables(const State& state, WorkCell* workcell)
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

    Frame::const_iterator_pair children = frame->getChildren(state);
    for (Frame::const_iterator it = children.first; it != children.second; ++it) {
        getAllDrawables(state, &(*it), result);
    }
}

void WorkCellGLDrawer::drawCameraView(const State& state, Frame* camera)
{
    Frame* currentFrame = camera;
    while (currentFrame->getParent(state) != NULL) {
        currentFrame = currentFrame->getParent(state);
    }

    glPushMatrix();

    GLTransform(inverse(Kinematics::WorldTframe(camera, state)));
    draw(state, currentFrame);

    glPopMatrix();
}

void WorkCellGLDrawer::draw(const State& state, const Frame* frame)
{
    glPushMatrix();

    GLTransform(frame->getTransform(state));

    const DrawableList& drawables = getDrawablesForFrame(frame);
    typedef DrawableList::const_iterator DI;
    for (DI it = drawables.begin(); it != drawables.end(); ++it) {
        (*it)->draw();
    }

    const Frame::const_iterator_pair children = frame->getChildren(state);
    for (Frame::const_iterator it = children.first; it != children.second; ++it) {
        this->draw(state, &(*it));
    }

    glPopMatrix();
}

namespace
{
    Drawable* getFrameDrawableOrNull(const Frame& frame)
    {
        if (Accessor::DrawableID().has(frame)) {
            // Load the drawable:
            std::string drawableID = Accessor::DrawableID().get(frame);
            Drawable* drawable = DrawableFactory::GetDrawable(drawableID);

            if (drawable) {
                // Set various properties for the drawable:

                const double* scale = Accessor::GeoScale().getPtr(frame);
                if (scale) drawable->setScale((float)*scale);

                if (Accessor::DrawableHighlight().has(frame))
                    drawable->setHighlighted(true);

                if (Accessor::DrawableWireMode().has(frame))
                    drawable->setDrawType(Drawable::WIRE);

                return drawable;
            } else {
                RW_WARN(
                    "NULL drawable returned by loadDrawableFile() for GeoID "
                    << drawableID);
                return NULL;
            }
        } else {
            return NULL;
        }
    }
}

const DrawableList& WorkCellGLDrawer::getDrawablesForFrame(const Frame* frame)
{
    RW_ASSERT(frame);

    DrawableList& seq = _frameMap[frame];
    if (seq.empty()) {
        Drawable* drawable = getFrameDrawableOrNull(*frame);
        if (drawable) seq.push_back(drawable);
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
    seq.erase(std::remove(seq.begin(), seq.end(), drawable),
              seq.end());
}
