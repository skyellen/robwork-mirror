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


#include "StereoCameraModel.hpp"

#include <fstream>

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rw::math;

StereoCameraModel::StereoCameraModel(const std::string& name,
	 	 double fov, double width, double height,
        const rw::math::Transform3D<>& TL,
        const rw::math::Transform3D<>& TR,
	 	 rw::kinematics::Frame* frame,
	 	 const std::string& modelInfo):
		SensorModel(name, frame, modelInfo),
		_sdata(1, rw::common::ownedPtr( new StereoCameraModelCache()).cast<StateCache>()),
		_pmatrix( ProjectionMatrix::makePerspective(fov*height/width, width/height, 0.001, 30) ),
		_TR(TR), _TL(TL)
{
	add(_sdata);
}

StereoCameraModel::~StereoCameraModel() {}


Image::Ptr StereoCameraModel::getLeftImage(const rw::kinematics::State& state){
	return 	_sdata.getStateCache<StereoCameraModelCache>(state)->_leftImage;
}

void StereoCameraModel::setLeftImage(Image::Ptr img, rw::kinematics::State& state){
	_sdata.getStateCache<StereoCameraModelCache>(state)->_leftImage = img;
}

Image::Ptr StereoCameraModel::getRightImage(const rw::kinematics::State& state){
	return 	_sdata.getStateCache<StereoCameraModelCache>(state)->_rightImage;
}

void StereoCameraModel::setRightImage(Image::Ptr img, rw::kinematics::State& state){
	_sdata.getStateCache<StereoCameraModelCache>(state)->_rightImage = img;
}



bool StereoCameraModel::SaveCalibration(const std::string& filename,
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
         // Distortion parameters
         const std::vector<double> dist(4, 0.0);
         
         // Output number of cameras
         ofs << 2 << std::endl << std::endl;
         
         // Output left
         WriteCalibration(ofs, fov, wx, wy, TL, dist, direction, format);
         
         ofs << std::endl;

         // Output right
         WriteCalibration(ofs, fov, wx, wy, TR, dist, direction, format);
         
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

void StereoCameraModel::WriteCalibration(std::ostream& os,
                                    double fov, double wx, double wy,
                                    const rw::math::Transform3D<>& T,
                                    const std::vector<double>& dist,
                                    FOVDirection direction,
                                    CalibrationFormat format) {
   // Calculate focal length based on FOV
   const double dim = direction == HORIZONTAL ? wx : wy;
   const double f = 0.5 * dim / std::tan(0.5*fov);
   
   switch(format) {
      case OPENCV: {
         // Output resolutions
         os << wx << " " << wy << std::endl;
         
         // Output projection matrix
         os << f << " " << 0 << " " << 0.5*wx << std::endl;
         os << 0 << " " << f << " "  << 0.5*wy << std::endl;
         os << 0 << " " << 0 << " "  << 1 << std::endl;
         
         // Output distortion parameters
         if(!dist.empty()) {
            for(std::vector<double>::const_iterator it = dist.begin(); it != dist.end()-1; ++it)
               os << *it << " ";
            os << dist.back() << std::endl;
         }
         
         // Output rotation matrix
         const Rotation3D<>& R = T.R();
         for(unsigned int i = 0; i < 3; ++i)
            os << R(i,0) << " " << R(i,1) << " " << R(i,2) << std::endl;
         
         // Output translation vector, scaled to mm
         const Vector3D<> P = T.P()*1000.0;
         os << P[0] << " " << P[1] << " " << P[2] << std::endl;
         
         break;
      } default:
         RW_WARN("Unknown stereo calibration file format!");
         break;
   }
}
