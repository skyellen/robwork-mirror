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

	//! @brief Render for debugging dynamic simulators.
	class SimulatorDebugRender : public rw::graphics::Render {
	public:
		//! @brief Smart pointer type for SimulatorDebugRender.
	    typedef rw::common::Ptr<SimulatorDebugRender> Ptr;

		//virtual ~SimulatorDebugRender(){};

	    //! @brief Draw mask for drawing nothing.
		static const unsigned int DRAW_NOTHING = 0;
	    //! @brief Draw mask for drawing contact normals.
		static const unsigned int DRAW_CONTACT_NORMAL = 1;
	    //! @brief Draw mask for drawing contact friction cones.
		static const unsigned int DRAW_FRICTION_CONE = 2;
	    //! @brief Draw mask for drawing body forces.
		static const unsigned int DRAW_BODY_FORCES = 4;
	    //! @brief Draw mask for drawing collision geometries.
		static const unsigned int DRAW_COLLISION_GEOMETRY = 8;


		//virtual void draw(DrawType type, double alpha) = 0;

		/**
		 * @brief Set the draw mask.
		 * @param mask [in] the draw mask.
		 */
		virtual void setDrawMask(unsigned int mask) = 0;

	protected:
		//! @brief Constructor.
	    SimulatorDebugRender(){}

	};

} // dynamics
}
#endif /*RENDERSIMDEBUG_HPP_*/
