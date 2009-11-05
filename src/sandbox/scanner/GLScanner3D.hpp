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
