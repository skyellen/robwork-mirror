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

#ifndef RWLIBS_SIMULATION_GLSCANNER3D_HPP
#define RWLIBS_SIMULATION_GLSCANNER3D_HPP

#include <rw/sensor/Scanner3D.hpp>

#include <rwlibs/drawable/WorkCellGLDrawer.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/PerspectiveTransform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

namespace rwlibs { namespace simulation {

    /** @addtogroup simulation */
    /* @{ */

    /**
     * @brief This simulated 3D scanner implementation use the OpenGL depth buffer for
     * doing depth tests. The scanner assimilates 3D range devices such as the Swissranger.
     */
    class GLScanner3D: public rw::sensor::Scanner3D {
    public:

        /**
         * @brief constructor
         * @param width [in] width of image
         * @param height [in] height of image
         * @param fov [in] the vertical field of view angle in degree
         * @param drawer [in] the WorkCellGLDrawer that draws the OpenGL scene
         * @param state [in] the state of the workcell
         */
        GLScanner3D(
            int width, int height, double fov,
            rw::kinematics::Frame* frame,
            const std::string& name,
            rwlibs::drawable::WorkCellGLDrawer *drawer)
            :
            rw::sensor::Scanner3D(frame,name),
            _fieldOfView(fov),_drawer(drawer),
            _img(width,height)
        {}

        /**
         * @brief destructor
         */
        virtual ~GLScanner3D();

        /**
         * @copydoc rw::sensor::Sensor::update
         */
        void update(double dt, const rw::kinematics::State& state);

        const rw::sensor::Image3D& getImage(){ return _img;};

        double getFrameRate(){ return _framerate; };

        void setFrameRate(double framerate){
            _framerate = framerate;
        }

        void acquire(){

        }

        bool isImageReady() = 0;
    private:

        double _fieldOfView; // in the y-axis
        rwlibs::drawable::WorkCellGLDrawer *_drawer;
        double _framerate;
        rw::sensor::Image3D _img;
    };

    /* @} */

}} // end namespaces

#endif // end include guard
