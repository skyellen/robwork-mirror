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

#ifndef RWLIBS_DRAWABLE_TACTILESENSORRENDER_HPP
#define RWLIBS_DRAWABLE_TACTILESENSORRENDER_HPP

//! @file TactileArrayRender.hpp

#include <rw/math/Vector3D.hpp>
#include <rw/sensor/TactileArray.hpp>
#include <rwlibs/drawable/Render.hpp>
#include <rwlibs/os/rwgl.hpp>

namespace rwlibs {
namespace drawable {

//! @addtogroup drawable
// @{
    /**
     * @brief renders a TactileArray.
     */
    class TactileArrayRender : public rwlibs::drawable::Render
    {
    public:

    	/**
    	 * @brief constructor
    	 * @param sensor [in] the tactile array that is to be rendered
    	 */
        TactileArrayRender(rw::sensor::TactileArray *sensor):
            _sensor(sensor)
        {
        }

        /**
         * @brief destructor
         */
        virtual ~TactileArrayRender(){};


        //! @copydoc Render::draw
        void draw(Render::DrawType type, double alpha) const;

    private:
        rw::sensor::TactileArray *_sensor;
    };
    //! @}
}
}

#endif /*TactileSensorRender_HPP_*/
