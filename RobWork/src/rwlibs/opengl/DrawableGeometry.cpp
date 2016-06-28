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

#include "DrawableGeometry.hpp"

#include <boost/foreach.hpp>

using namespace rwlibs::opengl;
using namespace rw::common;
using namespace rw::math;

DrawableGeometry::DrawableGeometry(
    const std::string& name,
    unsigned int dmask):
    DrawableGeometryNode(name),
    _drawable(ownedPtr(new Drawable(name,dmask))),
    _alpha(1.0), _rgb(1,0,0)
{

}

DrawableGeometry::~DrawableGeometry(){}


void DrawableGeometry::setColor(double r, double g, double b, double alpha){
    _rgb = Vector3D<>(r,g,b);
    _alpha = alpha;
    if(_rlines)
        _rlines->setColor((float)r,(float)g,(float)b,(float)alpha);

}

void DrawableGeometry::setColor(const rw::math::Vector3D<>& rgb){
    _rgb = rgb;
    if(_rlines)
        _rlines->setColor((float)_rgb[0],(float)_rgb[1],(float)_rgb[2],(float)_alpha);
    BOOST_FOREACH(RenderGeometry::Ptr rg, _rgeoms){
        rg->setColor((float)_rgb[0],(float)_rgb[1],(float)_rgb[2]);
    }
}

void DrawableGeometry::setAlpha(double alpha){
    _alpha = alpha;
}

rw::math::Vector3D<> DrawableGeometry::getColor(){
    return _rgb;
}

double DrawableGeometry::getAlpha(  ){
    return _alpha;
}

void DrawableGeometry::addLines(const std::vector<rw::geometry::Line >& lines){
    initLines();
    _rlines->addLines(lines);
}

void DrawableGeometry::addLine(const rw::math::Vector3D<>& v1, const rw::math::Vector3D<>& v2){
    initLines();
    _rlines->addLine(v1,v2);
}

void DrawableGeometry::addGeometry(rw::geometry::Geometry::Ptr geom){
    _rgeoms.push_back( ownedPtr(new RenderGeometry(geom)) );
    _drawable->addRender(_rgeoms.back());
}

void DrawableGeometry::addFrameAxis(double size){
    _rframes.push_back(ownedPtr(new RenderFrame( (float) size )));
    _drawable->addRender(_rframes.back());
}


void DrawableGeometry::initLines(){
    if(_rlines==NULL){
        _rlines = ownedPtr( new RenderLines());
        _rlines->setColor((float)_rgb[0],(float)_rgb[1],(float)_rgb[2],(float)_alpha);
        _drawable->addRender(_rlines);
    }
}
