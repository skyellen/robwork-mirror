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

#ifndef RWHW_SWISSRANGER_HPP
#define RWHW_SWISSRANGER_HPP

/**
 * @file SwissRanger.hpp
 */

#include "SRConstants.hpp"
#include "SRCalibrationData.hpp"

#include <string>

#include <rw/sensor/Sensor.hpp>
#include <rw/math/Vector3D.hpp>

namespace rwhw {

    /** @addtogroup swissranger */
    /* @{ */

    /**
     * @brief Class for high-level communication with the SwissRanger Camera
     */
    class SwissRanger: public rw::sensor::Sensor {
    public:

        /**
         * @brief Constructs SwissRanger camera
         * @param name [in] name of sensor
         * @param maxrange [in] Maximal range of the sensor. Default value \b maxrange = 7.5meter
         */
        SwissRanger(const std::string& name, float maxrange = 7.5);

        /**
         * @brief Destructor. Closes camera connection is not already closed
         */
        virtual ~SwissRanger();

        /**
         * @brief Loads calibration data.
         *
         * This methods must be called before trying to obtain data with acquire
         *
         * @param calibFilename [in] full path and filename of the calibration data file
         */
        void loadCalibrationData(const std::string& calibFilename);

        /**
         * @brief Opens connection to the camera
         */
        void openCamera();

        /**
         * @brief Closes the connection to the camera
         */
        void closeCamera();

        /**
         * @brief Acquires raw data distance data from camera
         *
         * @param data [out] array of size IMG_SIZE into which to copy data
         */
        void acquireRaw(unsigned short* data);

        /**
         * @brief Acquires data, adjusts using calibratin data and estimates accuracies
         *
         * @param points [out] array of size 3*IMG_SIZE into which to copy the
         * 3D points. The is x1 y1 z1 x2 y2 z2....
         *
         * @param accuracies [out] array of size IMG_SIZE into which to copy the
         * estimated accuracies
         *
         * @param intensities [out] array of size IMG_SIZE into which to copy
         * the estimated intensities
         *
         * @param n [in] the number of images to average over
         */
        void acquire(
            float* points,
            float* accuracies,
            float* intensities,
            unsigned short n);

        /**
         * @brief Returns whether the camera has been opened
         *
         * @return true if camera is opened
         */
        virtual bool isOpen();

        /**
         * @brief Sets the camera integration time
         *
         * @param intTime [in] integration time within [0;255]
         */
        void setIntegrationTime(unsigned char intTime);

        /**
         * @brief Returns the integration time
         *
         * @return integration time
         */
        unsigned short getIntegrationTime() const;

        /**
         * @brief Sets the maximal range we will accept for measurements
         * @param maxrange [in] The value must be between 0 and 7.5 meter.
         */
        void setMaxRange(float maxrange);

        /**
         * @brief Returns the selected max range of the sensor
         * @return the max range
         */
        float getMaxRange();


        /**
         * @brief Image width in pixels
         */
        static const unsigned int IMG_WIDTH;

        /**
         * @brief Image height in pixels
         */
        static const unsigned int IMG_HEIGHT;

        /**
         * @brief Image size in pixels
         */
        static const unsigned int IMG_SIZE;

    private:
        int _srHandle;
        unsigned char _nIntTime;
        unsigned short _image[3 * swissranger::IMG_SIZE];

        unsigned long _distanceBuffer[swissranger::IMG_SIZE];
        unsigned long _intensityBuffer[swissranger::IMG_SIZE];
        unsigned long _amplitudeBuffer[swissranger::IMG_SIZE];

        bool _isOpen;
        swissranger::SRCalibrationData _calibrationData;

        float _maxrange;
    };

    /* @} */

} // end namespaces

#endif // end include guard
