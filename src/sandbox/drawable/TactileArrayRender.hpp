#ifndef Drawable_TactileSensorRender_hpp_
#define Drawable_TactileSensorRender_hpp_

#include <rwlibs/drawable/Render.hpp>
#include <rwlibs/os/rwgl.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/sensor/TactileArray.hpp>

namespace rwlibs {
namespace drawable {
    /**
     * @brief renders a TactileArray
     */
    class TactileArrayRender : public rwlibs::drawable::Render
    {
    public:

        TactileArrayRender(rw::sensor::TactileArray *sensor):
            _sensor(sensor)
        {
        }

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
