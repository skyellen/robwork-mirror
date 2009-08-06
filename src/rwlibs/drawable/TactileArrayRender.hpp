/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/
#ifndef RWLIBS_DRAWABLE_TACTILESENSORRENDER_HPP
#define RWLIBS_DRAWABLE_TACTILESENSORRENDER_HPP

#include <rwlibs/drawable/Render.hpp>
#include <rwlibs/os/rwgl.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/sensor/TactileArray.hpp>

namespace rwlibs {
namespace drawable {
    /**
     * @brief renders a TactileArray.
     */
    class TactileArrayRender : public rwlibs::drawable::Render
    {
    public:

    	/**
    	 * @brief the tactile array that is to be rendered
    	 */
        TactileArrayRender(rw::sensor::TactileArray *sensor):
            _sensor(sensor)
        {
        }

        /**
         * @brief destructor
         */
        virtual ~TactileArrayRender(){};

        /**
         * @copydoc Render::draw
         */
        void draw(Render::DrawType type, double alpha) const;

    private:
        rw::sensor::TactileArray *_sensor;
    };

}
}

#endif /*TactileSensorRender_HPP_*/
