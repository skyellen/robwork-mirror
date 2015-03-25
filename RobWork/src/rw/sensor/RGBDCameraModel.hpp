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


#ifndef RW_SENSOR_RGBDCAMERAMODEL_HPP
#define RW_SENSOR_RGBDCAMERAMODEL_HPP

/**
 * @file RGBDCameraModel.hpp
 */

#include "SensorModel.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw { namespace sensor {
   
   /** @addtogroup sensor */
   /* @{ */
   
   /**
    * @brief
    *
    */
   class RGBDCameraModel : public SensorModel {
      public:
         //! @brief smart pointer type to this class
         typedef rw::common::Ptr<RGBDCameraModel> Ptr;
         
         /**
          * @brief constructor
          * @param name [in] name of sensor
          * @param modelInfo [in] info string
          */
         RGBDCameraModel(const std::string& name, rw::kinematics::Frame* frame, const std::string& modelInfo);
         
         /**
          * @brief destructor
          */
         virtual ~RGBDCameraModel();
         
   };
   
   /* @} */
   
}} // end namespaces

#endif // end include guard
