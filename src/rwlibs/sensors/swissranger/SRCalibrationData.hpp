/*********************************************************************
 * RobWork Version 0.2
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

#ifndef rwlibs_sensors_swissranger_SRCalibrationData_HPP
#define rwlibs_sensors_swissranger_SRCalibrationData_HPP

/**
 * @file SRCalibrationData.hpp
 */

#include "SRConstants.hpp"

#include <string>

namespace rwlibs { namespace sensors { namespace swissranger {
    /** @addtogroup sensors */
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
    
}}} // end namespaces

#endif // end include guard
