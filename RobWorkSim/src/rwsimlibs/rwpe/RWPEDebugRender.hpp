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

#ifndef RWSIMLIBS_RWPE_RWPEDEBUGRENDER_HPP_
#define RWSIMLIBS_RWPE_RWPEDEBUGRENDER_HPP_

/**
 * @file RWPEDebugRender.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEDebugRender
 */

#include <rwsim/drawable/SimulatorDebugRender.hpp>

namespace rwsimlibs {
namespace rwpe {

class RWPEBodyConstraintGraph;
class RWPEIslandState;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief The debug render for RWPE engines.
 */
class RWPEDebugRender: public rwsim::drawable::SimulatorDebugRender {
public:
	//! @brief Construct new render.
	RWPEDebugRender();

	//! @brief Destructor.
	virtual ~RWPEDebugRender();

	//! @copydoc SimulatorDebugRender::setDrawMask
	virtual void setDrawMask(unsigned int mask);

	//! @copydoc Render::draw
    virtual void draw(const rw::graphics::DrawableNode::RenderInfo& info, rw::graphics::DrawableNode::DrawType type, double alpha) const;

    /**
     * @brief Update the state for the render.
     * @param manager [in] the manager.
     * @param state [in] the current state of the system.
     * @param gravity [in] the gravity vector.
     */
    void update(const RWPEBodyConstraintGraph* manager, const RWPEIslandState* state, const rw::math::Vector3D<>& gravity);

private:
    unsigned int _drawMask;
    const RWPEBodyConstraintGraph* _manager;
    const RWPEIslandState* _state;
    rw::math::Vector3D<> _gravity;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEDEBUGRENDER_HPP_ */
