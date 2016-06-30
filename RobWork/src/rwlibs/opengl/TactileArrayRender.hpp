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

#ifndef RWLIBS_OPENGL_TACTILESENSORRENDER_HPP
#define RWLIBS_OPENGL_TACTILESENSORRENDER_HPP

//! @file TactileArrayRender.hpp

#include <rw/graphics/Render.hpp>

namespace rw { namespace sensor { class TactileArrayModel; } }

namespace rwlibs {
namespace opengl {

//! @addtogroup opengl
// @{
    /**
     * @brief renders a TactileArray.
     */
    class TactileArrayRender : public rw::graphics::Render
    {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<TactileArrayRender> Ptr;

    	/**
    	 * @brief constructor
    	 * @param sensor [in] the tactile array that is to be rendered
    	 */
        TactileArrayRender(rw::common::Ptr<rw::sensor::TactileArrayModel> sensor):
            _sensor(sensor)
        {
        }

        /**
         * @brief destructor
         */
        virtual ~TactileArrayRender(){};

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;

    private:
        rw::common::Ptr<rw::sensor::TactileArrayModel> _sensor;
    };
    //! @}
}
}

#endif /*TactileSensorRender_HPP_*/
