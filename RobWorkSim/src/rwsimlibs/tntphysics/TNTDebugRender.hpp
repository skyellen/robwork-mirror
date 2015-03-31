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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTDEBUGRENDER_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTDEBUGRENDER_HPP_

/**
 * @file TNTDebugRender.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTDebugRender
 */

#include <rwsim/drawable/SimulatorDebugRender.hpp>

namespace rwsimlibs {
namespace tntphysics {

class TNTBodyConstraintManager;
class TNTIslandState;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief The debug render for TNT engines.
 */
class TNTDebugRender: public rwsim::drawable::SimulatorDebugRender {
public:
	//! @brief Construct new render.
	TNTDebugRender();

	//! @brief Destructor.
	virtual ~TNTDebugRender();

	//! @copydoc SimulatorDebugRender::setDrawMask
	virtual void setDrawMask(unsigned int mask);

	//! @copydoc Render::draw
    virtual void draw(const rw::graphics::DrawableNode::RenderInfo& info, rw::graphics::DrawableNode::DrawType type, double alpha) const;

    /**
     * @brief Update the state for the render.
     * @param state [in] the current state of the system.
     */
    void update(const TNTBodyConstraintManager* manager, const TNTIslandState* state, const rw::math::Vector3D<>& gravity);

private:
    unsigned int _drawMask;
    const TNTBodyConstraintManager* _manager;
    const TNTIslandState* _state;
    rw::math::Vector3D<> _gravity;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTDEBUGRENDER_HPP_ */
