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


#ifdef DEPRECATED

#include "DrawableModelInfo.hpp"

using namespace rw::models;

namespace {

	void init(double &scale, bool &wire, bool &high){
		scale = 1.0;
		wire = false;
		high = false;
	}

}

DrawableModelInfo::DrawableModelInfo(const std::string& id,const std::string& name):
	_drawableId(id),
	_name(name),
	_transform(rw::math::Transform3D<>::identity())
{
	init(_geoScale,_wireMode,_highlighted);
}

DrawableModelInfo::DrawableModelInfo(const std::string& id, const std::string& name, rw::math::Transform3D<> t3d):
	_drawableId(id),
	_name(name),
	_transform(t3d)
{
	init(_geoScale,_wireMode,_highlighted);
}

DrawableModelInfo::DrawableModelInfo(const std::string& id,
                                     const std::string& name,
                                     rw::math::Transform3D<> t3d,
			 double scale, bool wire, bool high):
	_drawableId(id),
	_name(name),
	_transform(t3d),
	_geoScale(scale),
	_wireMode(wire),
	_highlighted(high)
{

}

std::vector<DrawableModelInfo> DrawableModelInfo::get(const rw::common::PropertyMap& pmap){
    return pmap.get<std::vector<DrawableModelInfo> >("DrawableModelInfo", std::vector<DrawableModelInfo>());
}

std::vector<DrawableModelInfo> DrawableModelInfo::get(rw::kinematics::Frame* frame){
    return get(frame->getPropertyMap());
}

void DrawableModelInfo::set(const std::vector<DrawableModelInfo>& data, rw::kinematics::Frame* frame){
    set(data, frame->getPropertyMap());
}

void DrawableModelInfo::set(const std::vector<DrawableModelInfo>& data, rw::common::PropertyMap& pmap){
    pmap.addForce<std::vector<DrawableModelInfo> >("DrawableModelInfo", "ID for the Drawable", data);
}


#endif //#ifdef DEPRECATED