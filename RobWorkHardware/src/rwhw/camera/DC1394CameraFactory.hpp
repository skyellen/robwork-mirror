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

#ifndef RWHW_CAMERA_DC1394CAMERAFACTORY_HPP
#define RWHW_CAMERA_DC1394CAMERAFACTORY_HPP

#include <rw/kinematics/Frame.hpp>

#include <dc1394/dc1394.h>
#include <string>
#include <vector>

#include "DC1394Camera.hpp"

namespace rwhw {
namespace camera {
    /** @addtogroup camera */
    /* @{ */

    //class DC1394Camera : public rw::sensor::CameraFirewire {};

    /**
     * @brief Factory for DC1394 Cameras
     */
    class DC1394CameraFactory {
    public:
        /**
         * @brief Returns instance of CD1394CameraFactory
         */
        static DC1394CameraFactory* getInstance();

        /**
         * @brief Returns lists of cameras
         */
        const std::vector<std::string>& getCameraList();

        /**
         * @brief Returns a camera associated to a specific frame and a given index
         */
        DC1394Camera* getCamera(rw::kinematics::Frame* frame, unsigned int index);

        /**
         * @brief Free the given camera
         */
        void freeCamera(DC1394Camera* camera);


    protected:
        DC1394CameraFactory();
        virtual ~DC1394CameraFactory();
        void freeCameraByIndex(unsigned int index);

        void initialize();

        static DC1394CameraFactory* _instance;
        dc1394_t * _dc1394;
        std::vector<std::string> _cameraNames;
        std::vector<dc1394camera_t *> _cameraDriverList;
        std::vector<DC1394Camera*> _cameraList;
    };

/* @} */

}
} // End namespaces

#endif /* RWHW_CAMERA_DC1394CAMERAFACTORY_HPP */
