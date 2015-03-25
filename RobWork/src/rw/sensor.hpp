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
/**
 * @file rw/sensor.hpp
 *
 * this file includes all header files from the sensor namespace
 */

#ifndef RW_SENSOR_HPP_
#define RW_SENSOR_HPP_

#include "./sensor/Sensor.hpp"

#include "./sensor/Camera.hpp"
#include "./sensor/StereoCameraModel.hpp"
#include "./sensor/CameraFirewire.hpp"
#include "./sensor/CameraListener.hpp"
#include "./sensor/ImageUtil.hpp"

#include "./sensor/Scanner.hpp"
#include "./sensor/Scanner1D.hpp"
#include "./sensor/Scanner2D.hpp"
#include "./sensor/Scanner25D.hpp"

#include "./sensor/TactileArray.hpp"

#include "./sensor/SensorData.hpp"
#include "./sensor/Image.hpp"
#include "./sensor/Contact3D.hpp"

#endif /* SENSOR_HPP_ */
