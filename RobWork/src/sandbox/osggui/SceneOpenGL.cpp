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


#include "SimpleGLScene.hpp"

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
#include <rw/geometry/GeometryFactory.hpp>

#include "Render.hpp"
#include "RenderGeometry.hpp"
#include "Drawable.hpp"
#include "DrawableFactory.hpp"
#include "DrawableUtil.hpp"
#include <vector>

using namespace rw::math;
using namespace rwlibs::drawable;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::common;
using namespace rw::sensor;

SimpleGLScene::~SimpleGLScene()
{
    clearCache();
}

void SimpleGLScene::clearCache()
{
    //BOOST_FOREACH(FrameMap::const_reference entry, _frameMap) {
    //    BOOST_FOREACH(rwlibs::drawable::Drawable::Ptr da, entry.second) {
    //        delete da;
    //    }
    //}
    _frameMap.clear();
}

void SimpleGLScene::draw(const State& state, WorkCell* workcell, unsigned int dmask)
{
    Frame* world = workcell->getWorldFrame();
    draw(state, world, dmask);
}

std::vector<Drawable::Ptr> SimpleGLScene::getAllDrawables(
    const State& state, WorkCell* workcell)
{
    std::vector<Drawable::Ptr> result;
    getAllDrawables(state, workcell->getWorldFrame(), result);
    return result;
}

void SimpleGLScene::getAllDrawables(
    const State& state,
    const Frame* frame,
    std::vector<Drawable::Ptr>& result)
{
    const std::vector<Drawable::Ptr>& drawables = getDrawablesForFrame(frame);
    result.insert(result.end(), drawables.begin(), drawables.end());
    BOOST_FOREACH(const Frame& child, frame->getChildren(state)) {
        getAllDrawables(state, &child, result);
    }
}

void SimpleGLScene::drawCameraView(const State& state, Frame* camera, unsigned int dmask)
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

void SimpleGLScene::draw(const State& state, const Frame* frame, unsigned int dmask)
{
    glPushMatrix();

    DrawableUtil::multGLTransform( frame->getTransform(state) );
    const std::vector<Drawable::Ptr>& drawables = getDrawablesForFrame(frame);
    BOOST_FOREACH(const Drawable::Ptr& da, drawables) { da->draw(dmask); }

    BOOST_FOREACH(const Frame& child, frame->getChildren(state)) {
        draw(state, &child, dmask);
    }

    glPopMatrix();
}

void SimpleGLScene::drawAndSelect(const rw::kinematics::State& state,
                   rw::models::WorkCell* workcell, unsigned int dmask){

    Frame* world = workcell->getWorldFrame();
    drawAndSelect(state, world, dmask);
}


void SimpleGLScene::drawAndSelect(const rw::kinematics::State& state,
                   const Frame* frame, unsigned int dmask)
{


    glPushMatrix();
    DrawableUtil::multGLTransform(frame->getTransform(state));
    const std::vector<Drawable::Ptr>& drawables = getDrawablesForFrame(frame);

    glPushName( (GLuint) frame->getID() );
    BOOST_FOREACH(const Drawable::Ptr& da, drawables) { da->draw(dmask); }
    glPopName();

    BOOST_FOREACH(const Frame& child, frame->getChildren(state)) {
        drawAndSelect(state, &child, dmask);
    }

    glPopMatrix();

}


namespace
{
    std::vector<Drawable::Ptr> getFrameDrawables(const Frame& frame)
    {
        std::vector<Drawable::Ptr> result;
        if (Accessor::drawableModelInfo().has(frame)  ) {
            // Load the drawable:
        	const std::vector<DrawableModelInfo> infos = Accessor::drawableModelInfo().get(frame);
        	BOOST_FOREACH(const DrawableModelInfo &info, infos) {
        		rwlibs::drawable::Drawable::Ptr drawable = NULL;
        		try {
        			drawable = DrawableFactory::getDrawable(info.getId());
        		} catch (const rw::common::Exception& exp){
        			RW_WARN(exp.getMessage());
        		}

	            if (drawable) {
	                // Set various properties for the drawable:
	            	drawable->setTransform(info.getTransform());
	            	drawable->setScale((float)info.getGeoScale());
	            	drawable->setMask( Drawable::DrawableObject | Drawable::Physical );

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
        if (Accessor::collisionModelInfo().has(frame) ) {

            const std::vector<CollisionModelInfo> cinfos = Accessor::collisionModelInfo().get(frame);
            BOOST_FOREACH(const CollisionModelInfo &info, cinfos) {
                rwlibs::drawable::Drawable::Ptr drawable = NULL;
                try {

                    //drawable = DrawableFactory::constructFromGeometry(info.getId());

                    Geometry::Ptr geometry = GeometryFactory::getGeometry(info.getId());
                    Render *render = new RenderGeometry(geometry);
                    drawable =  ownedPtr( new Drawable( ownedPtr(render) ) );

                } catch (const rw::common::Exception& exp){
                    RW_WARN(exp.getMessage());
                }

                if (drawable) {
                    // Set various properties for the drawable:
                    drawable->setMask( Drawable::CollisionObject );
                    drawable->setTransform(info.getTransform());
                    drawable->setScale((float)info.getGeoScale());

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

const std::vector<Drawable::Ptr>& SimpleGLScene::getDrawablesForFrame(const Frame* frame)
{
    RW_ASSERT(frame);

    std::vector<Drawable::Ptr>& seq = _frameMap[frame];
    if (seq.empty()) {
        std::vector<Drawable::Ptr> drawables = getFrameDrawables(*frame);
        if (!drawables.empty())
        	seq = drawables;
    }

    return seq;
}

void SimpleGLScene::addDrawableToFrame(Frame* frame, Drawable::Ptr drawable)
{
    _frameMap[frame].push_back(drawable);
}

void SimpleGLScene::removeDrawableFromFrame(Frame* frame, Drawable* drawable)
{
    std::vector<Drawable::Ptr>& seq = _frameMap[frame];
    seq.erase(
        std::remove(seq.begin(), seq.end(), drawable),
        seq.end());
}

void SimpleGLScene::lock(){
	_mutex.lock();
}

void SimpleGLScene::unlock(){
	_mutex.unlock();
}

std::pair<Drawable::Ptr,RenderFrame::Ptr> SimpleGLScene::addFrame(double size, Frame* frame, int dmask){
    //RW_THROW(frame!=NULL);
    RenderFrame::Ptr render = ownedPtr( new RenderFrame(size) );
    Drawable::Ptr drawable = ownedPtr( new Drawable(render,dmask) );
    addDrawableToFrame(frame, drawable);
    return std::make_pair(drawable, render);
}
std::pair<Drawable::Ptr,RenderGeometry::Ptr> SimpleGLScene::addGeometry(Geometry::Ptr geom, Frame* frame, int dmask){
    //RW_THROW(frame!=NULL);
    RenderGeometry::Ptr render = ownedPtr( new RenderGeometry(geom) );
    Drawable::Ptr drawable = ownedPtr( new Drawable(render,dmask) );
    addDrawableToFrame(frame, drawable);
    return std::make_pair(drawable, render);
}
std::pair<Drawable::Ptr, RenderModel3D::Ptr> SimpleGLScene::addModel3D(Model3D::Ptr model, Frame* frame, int dmask){
    //RW_THROW(frame!=NULL);
    RenderModel3D::Ptr render = ownedPtr( new RenderModel3D(model) );
    Drawable::Ptr drawable = ownedPtr( new Drawable(render,dmask) );
    addDrawableToFrame(frame, drawable);
    return std::make_pair(drawable, render);
}
std::pair<Drawable::Ptr,RenderImage::Ptr> SimpleGLScene::addImage(const Image& img, Frame* frame, int dmask){
    //RW_THROW(frame!=NULL);
    RenderImage::Ptr render = ownedPtr( new RenderImage(img) );
    Drawable::Ptr drawable = ownedPtr( new Drawable(render,dmask) );
    addDrawableToFrame(frame, drawable);
    return std::make_pair(drawable, render);
}
std::pair<Drawable::Ptr,RenderScan::Ptr> SimpleGLScene::addScan(const Scan2D& scan, Frame* frame, int dmask){
    //RW_THROW(frame!=NULL);
    RenderScan::Ptr render = ownedPtr( new RenderScan(scan) );
    Drawable::Ptr drawable = ownedPtr( new Drawable(render,dmask) );
    addDrawableToFrame(frame, drawable);
    return std::make_pair(drawable, render);
}
std::pair<Drawable::Ptr,RenderScan::Ptr> SimpleGLScene::addScan(const Image25D& scan, Frame* frame, int dmask){
    //RW_THROW(frame!=NULL);
    RenderScan::Ptr render = ownedPtr( new RenderScan(scan) );
    Drawable::Ptr drawable = ownedPtr( new Drawable(render,dmask) );
    addDrawableToFrame(frame, drawable);
    return std::make_pair(drawable, render);
}

std::pair<Drawable::Ptr,RenderLines::Ptr> SimpleGLScene::addLines(const std::vector<Line >& lines, Frame* frame, int dmask){
    //RW_THROW(frame!=NULL);
    RenderLines::Ptr render = ownedPtr( new RenderLines(lines) );
    Drawable::Ptr drawable = ownedPtr( new Drawable(render,dmask) );
    addDrawableToFrame(frame, drawable);
    return std::make_pair(drawable, render);
}


