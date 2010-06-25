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

#ifndef RWHW_SWISSRANGER_SRCALIBRATIONDATA_HPP
#define RWHW_SWISSRANGER_SRCALIBRATIONDATA_HPP

/**
 * @file SRCalibrationData.hpp
 */

#include "SRConstants.hpp"

#include <string>

namespace rwhw { namespace swissranger {
    /** @addtogroup swissranger */
    /* @{ */
    /**
     * @brief Class representing calibration data for the SwissRanger
     */
    class SRCalibrationData {
    public:
        /**
         * @brief Default constructor
         */
        SRCalibrationData();

        /**
         * @brief Destructor
         */
        ~SRCalibrationData();

        /**
         * @brief Loads file with calibration data
         * @param filename [in] file name
         * @return whether the file was successfully loaded
         */
        bool load(const std::string& filename);

        /**
         * @brief Saves file with calibration data
         * @param filename [in] file name
         * @return whether the file was successfully saved
         */
        bool save(const std::string& filename);

        /**
         * @brief Converts raw output distance to distance in meters
         * @param value [in] raw input distance from SwissRanger
         * @param index [in] index of pixel
         * @return distance in meter
         */
        float getDistance(unsigned short value, int index) {
            //return (7.5*value)/0xFFFC;
            return _gains[index]*value-_offsets[index];
        }

        /**
         * @brief Returns the rotation around x-axis for the pixel
         * @param index [in] index of pixel
         * @return x-axis rotation
         */
        float getAlpha(int index) {
            return _alphas[index];
        }

        /**
         * @brief Returns the rotation around y-axis for the pixel
         * @param index [in] index of pixel
         * @return y-axis rotation
         */
        float getBeta(int index) {
            return _betas[index];
        }

        /**
         * @brief Sets the offset of the pixel (only to be used when calibrating)
         * @param offset [in] offset of pixel
         * @param index [in] index of pixel
         */
        void setOffset(float offset, int index) {
            _offsets[index] = offset;
        }

        /**
         * @brief Sets the gain of the pixel (only to be used when calibrating)
         * @param gain [in] gain of pixel
         * @param index [in] index of pixel
         */
        void setGain(float gain, int index) {
            _gains[index] = gain;
        }

        /**
         * @brief Sets the x-axis rotation of the pixel (only to be used when calibrating)
         * @param alpha [in] x-axis rotation
         * @param index [in] index of pixel
         */
        void setAlpha(float alpha, int index) {
            _alphas[index] = alpha;
        }

        /**
         * @brief Sets the y-axis rotation of the pixel (only to be used when calibrating)
         * @param beta [in] y-axis rotation
         * @param index [in] index of pixel
         */
        void setBeta(float beta, int index) {
            _betas[index] = beta;
        }


    private:
        float _offsets[IMG_SIZE];
        float _gains[IMG_SIZE];
        float _alphas[IMG_SIZE];
        float _betas[IMG_SIZE];
    };

    /* @} */

}} // end namespaces

#endif // end include guard
