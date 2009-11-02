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

#include "Render3DS.hpp"

#include <rw/common/macros.hpp>

using namespace rwlibs::drawable;

Render3DS::Render3DS(const std::string &filename)
{
    RW_ASSERT(!filename.empty());

    _model.Load(filename); // Load the model
}

void Render3DS::draw(DrawType type, double alpha) const
{
    switch (type) {
    case Render::SOLID:
    	glPolygonMode(GL_FRONT, GL_FILL);
    	glColor4f(0.7f, 0.7f, 0.7f, static_cast<float>(alpha));
    	_model.Draw();
    	break;
    case Render::OUTLINE: // Draw nice frame
    	glPolygonMode(GL_FRONT, GL_FILL);
    	glColor4f(0.7f, 0.7f, 0.7f, static_cast<float>(alpha));
    	_model.Draw();
    case Render::WIRE:
    	glPolygonMode(GL_FRONT, GL_LINE);
    	glColor4f(0.7f, 0.7f, 0.7f, static_cast<float>(alpha));
    	_model.Draw();
    	break;
    }
}
