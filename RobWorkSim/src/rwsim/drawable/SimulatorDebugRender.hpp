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

#ifndef RWSIM_DRAWABLE_SIMULATORDEBUGRENDER_HPP_
#define RWSIM_DRAWABLE_SIMULATORDEBUGRENDER_HPP_

#include <rw/graphics/Render.hpp>

namespace rwsim {
namespace drawable {

	class SimulatorDebugRender : public rw::graphics::Render {
	public:
	    typedef rw::common::Ptr<SimulatorDebugRender> Ptr;

		//virtual ~SimulatorDebugRender(){};

		static const unsigned int DRAW_NOTHING = 0;
		static const unsigned int DRAW_CONTACT_NORMAL = 2;
		static const unsigned int DRAW_FRICTION_CONE = 4;

		//virtual void draw(DrawType type, double alpha) = 0;

		virtual void setDrawMask(unsigned int mask) = 0;

	protected:
	    SimulatorDebugRender(){};

	};

} // dynamics
}
#endif /*RENDERSIMDEBUG_HPP_*/
