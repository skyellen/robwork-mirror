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


#include "StereoCamera.hpp"

#include <algorithm>
#include <fstream>
#include <cmath>

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rw::math;

StereoCamera::StereoCamera(const std::string& name,
                           const std::string& modelInfo) : Sensor(name, modelInfo),
                                                           _modelInfo(modelInfo) {}

StereoCamera::~StereoCamera() {}

bool StereoCamera::SaveCalibration(const std::string& filename,
                                   double fov, double wx, double wy,
                                   const rw::math::Transform3D<>& TL,
                                   const rw::math::Transform3D<>& TR,
                                   FOVDirection direction,
                                   CalibrationFormat format) {
   // Open the file and check
   std::ofstream ofs(filename.c_str());
   if(ofs.fail())
      return false;
   
   // Switch the format
   switch(format) {
      case OPENCV: {
         // Calculate focal length based on FOV
         const double dim = direction == HORIZONTAL ? wx : wy;
         const double f = 0.5 * dim / std::tan(0.5*fov);
         // Output number of cameras
         ofs << 2 << std::endl;
         
         // Output resolutions
         ofs << wx << " " << wy << std::endl;
         // Output projection matrix
         ofs << f << " " << 0 << " " << 0.5*wx << std::endl;
         ofs << 0 << " " << f << " "  << 0.5*wy << std::endl;
         ofs << 0 << " " << 0 << " "  << 1 << std::endl;
         // Output distortion parameters
         ofs << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
         // Output rotation matrix
         const Rotation3D<>& RL = TL.R();
         for(unsigned int i = 0; i < 3; ++i)
            ofs << RL(i,0) << " " << RL(i,1) << " " << RL(i,2) << std::endl;
         // Output translation vector, scaled to mm
         const Vector3D<> PL = TL.P()*1000.0;
         ofs << PL[0] << " " << PL[1] << " " << PL[2] << std::endl;
         
         // Do the same stuff for the right camera
         ofs << wx << " " << wy << std::endl;
         ofs << f << " " << 0 << " " << 0.5*wx << std::endl;
         ofs << 0 << " " << f << " "  << 0.5*wy << std::endl;
         ofs << 0 << " " << 0 << " "  << 1 << std::endl;
         ofs << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
         const Rotation3D<>& RR = TR.R();
         for(unsigned int i = 0; i < 3; ++i)
            ofs << RR(i,0) << " " << RR(i,1) << " " << RR(i,2) << std::endl;
         const Vector3D<> PR = TR.P()*1000.0;
         ofs << PR[0] << " " << PR[1] << " " << PR[2] << std::endl;
         
         // Close file
         ofs.close();
         
         // Return file stream status
         return !ofs.fail();
      }
      default:
         RW_WARN("Unknown stereo calibration file format!");
         break;
   }
   
   return false;
}
