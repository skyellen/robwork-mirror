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


#ifndef RW_SENSOR_STEREOCAMERA_HPP
#define RW_SENSOR_STEREOCAMERA_HPP

/**
 * @file StereoCamera.hpp
 */

#include "Image.hpp"
#include "Sensor.hpp"
#include "CameraListener.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw { namespace sensor {
   
   /** @addtogroup sensor */
   /* @{ */
   
   /**
    * @brief The Camera class defines a general interface to a stereo camera.
    * A stereo camera consists of two cameras with the same intrinsic parameters,
    * but with different extrinsic parameters.
    * 
    * Since ideal cameras are assumed, the intrinsics are given as a horizontal/vertical
    * pixel resolution as well as a horizontal field of view (FOV).
    * 
    * The extrinsic parameters are given simply as two transformation matrices,
    * which give the pose of the cameras relative some external frame.
    *
    */
   class StereoCamera : public Sensor {
      public:
         //! @brief smart pointer type to this class
         typedef rw::common::Ptr<StereoCamera> Ptr;

         //! @brief output calibration file format for SaveCalibration()
         enum CalibrationFormat {
            OPENCV
         };
         
         //! @brief FOV direction
         enum FOVDirection {
            HORIZONTAL,
            VERTICAL
         };
         
      protected:
         /**
          * @brief constructor
          * @param name [in] name of sensor
          * @param modelInfo [in] info string
          */
         StereoCamera(const std::string& name,
                      const std::string& modelInfo);
         
         /**
          * @brief sets the camera model information
          * @param info [in] information of the camera
          */
         void setModelInfo(const std::string info) { _modelInfo = info; }
      
      public:
         /**
          * @brief destructor
          */
         virtual ~StereoCamera();
         
         /**
          * @brief returns the camera model information (version, type, size, etc.)
          * @return camera model information
          */
         virtual std::string getModelInfo() const { return _modelInfo;};
         
         /**
          * @brief utility function for saving a calibration file
          * @param filename [in] file to save to
          * @param fov [in] field of view (FOV) [rad]
          * @param wx [in] horizontal pixels
          * @param wy [in] vertical pixels
          * @param TL [in] transformation of left camera frame
          * @param TR [in] transformation of right camera frame
          * @param direction [in] the direction of the specified FOV
          * @param format [in] calibration file format to use
          * @return true if the file was successfully saved, false otherwise
          */
         static bool SaveCalibration(const std::string& filename,
                                     double fov, double wx, double wy,
                                     const rw::math::Transform3D<>& TL,
                                     const rw::math::Transform3D<>& TR,
                                     FOVDirection direction = HORIZONTAL,
                                     CalibrationFormat format = OPENCV);
      
      protected:
         //! name of camera model information
         std::string _modelInfo;
      
      private:
         StereoCamera(const StereoCamera&);
         StereoCamera& operator=(const StereoCamera&);
   };
   
   /* @} */
   
#ifdef RW_USE_DEPRECATED
   /**
    * @brief Smart pointer to StereoCamera
    */
   typedef rw::common::Ptr<StereoCamera> StereoCameraPtr;
#endif
}} // end namespaces

#endif // end include guard
