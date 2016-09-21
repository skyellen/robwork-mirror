/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_BULLET_BTDEBUGRENDER_HPP_
#define RWSIMLIBS_BULLET_BTDEBUGRENDER_HPP_

/**
 * @file BtDebugRender.hpp
 *
 * \copydoc rwsimlibs::bullet::BtDebugRender
 */

#include <rwsim/drawable/SimulatorDebugRender.hpp>
//#include <OpenGl/GLDebugDrawer.h>

namespace rwsimlibs {
namespace bullet {
class BtSimulator;

//! @addtogroup rwsimlibs_bullet

//! @{
/**
 * @brief Debug render for the Bullet engine.
 */
class BtDebugRender: public rwsim::drawable::SimulatorDebugRender {
public:
	/**
	 * @brief Constructor.
	 * @param sim the simulator the debug render is associated to.
	 */
	BtDebugRender(BtSimulator *sim);

	//! @brief Destructor
	virtual ~BtDebugRender();

    //! @copydoc rw::graphics::Render::draw
	virtual void draw(const rw::graphics::DrawableNode::RenderInfo& info,
			DrawType type,
			double alpha) const;

    //! @copydoc rwsim::drawable::SimulatorDebugRender::setDrawMask
	virtual void setDrawMask(unsigned int mask);

private:
	BtSimulator *_sim;
	//GLDebugDrawer *_debugDrawer;
	unsigned int _drawMask;
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTDEBUGRENDER_HPP_ */
