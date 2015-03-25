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


#ifndef RW_SENSOR_STEREOCAMERAMODEL_HPP
#define RW_SENSOR_STEREOCAMERAMODEL_HPP

/**
 * @file StereoCameraModel.hpp
 */

#include "Image.hpp"

#include "SensorModel.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/ProjectionMatrix.hpp>

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
   class StereoCameraModel : public SensorModel {
      public:
         //! @brief smart pointer type to this class
         typedef rw::common::Ptr<StereoCameraModel> Ptr;

         //! @brief output calibration file format for SaveCalibration()
         enum CalibrationFormat {
            OPENCV
         };
         
         //! @brief FOV direction
         enum FOVDirection {
            HORIZONTAL,
            VERTICAL
         };
         
         /**
          * @brief constructor
          * @param name [in] name of sensor
          * @param fov [in] horizontal field of view
          * @param width [in] width of image
          * @param height [in] height of image
          * @param TL [in] transform from sensor frame to left camera frame
          * @param TR [in] transform from sensor frame to right camera frame
          * @param frame [in] sensor frame
          * @param modelInfo [in] info string
          */
         StereoCameraModel(const std::string& name,
        		 	 	 double fov, double width, double height,
                         const rw::math::Transform3D<>& TL,
                         const rw::math::Transform3D<>& TR,
        		 	 	 rw::kinematics::Frame* frame,
        		 	 	 const std::string& modelInfo="");
         /**
          * @brief destructor
          */
         virtual ~StereoCameraModel();

         //! get left image
         Image::Ptr getLeftImage(const rw::kinematics::State& state);

         //! set left image
         void setLeftImage(Image::Ptr img, rw::kinematics::State& state);

         //! get right image
         Image::Ptr getRightImage(const rw::kinematics::State& state);

         //! set right image
         void setRightImage(Image::Ptr img, rw::kinematics::State& state);
         
         /**
          * @brief utility function for saving a stereo calibration to a file
          * 
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
      
         /**
          * @brief utility function for writing a camera calibration to a stream
          * 
          * @param os the stream to write to
          * @param fov field of view (FOV) [rad]
          * @param wx horizontal pixels
          * @param wy vertical pixels
          * @param T [in] transformation of the camera frame
          * @param dist [in] distortion parameters
          * @param direction [in] the direction of the specified FOV
          * @param format [in] calibration file format to use
          */
         static void WriteCalibration(std::ostream& os,
                                     double fov, double wx, double wy,
                                     const rw::math::Transform3D<>& T,
                                     const std::vector<double>& dist = std::vector<double>(4, 0.0),
                                     FOVDirection direction = HORIZONTAL,
                                     CalibrationFormat format = OPENCV);
      private:

         //! cache to allow storing state information
         class StereoCameraModelCache: public rw::kinematics::StateCache {
     	public:
     		typedef rw::common::Ptr<StereoCameraModelCache> Ptr;
     		rw::common::Ptr<rw::sensor::Image> _leftImage,_rightImage;

     		//! constructor
     		StereoCameraModelCache()
     		{
     		};

     		//! @copydoc rw::kinematics::StateCache::size
     		size_t size() const{
     			size_t stmp = 0;
     			if(_leftImage!=NULL)
     				stmp+=_leftImage->getDataSize();
     			if(_rightImage!=NULL)
     				stmp+=_rightImage->getDataSize();
     			return stmp;
     		};

     		//! @copydoc rw::kinematics::StateCache::clone
     		virtual rw::common::Ptr<StateCache> clone() const{
     			StereoCameraModelCache::Ptr cache = rw::common::ownedPtr( new StereoCameraModelCache(*this) );
     			if(_leftImage!=NULL)
     				cache->_leftImage = rw::common::ownedPtr( new Image( *_leftImage ));
     			if(_rightImage!=NULL)
     				cache->_rightImage = rw::common::ownedPtr( new Image( *_rightImage ));
     			return cache;
     		};
     	};

         //! name of camera model information
         rw::math::ProjectionMatrix _pmatrix;
         rw::math::Transform3D<> _TR, _TL;
         rw::kinematics::StatelessData<int> _sdata;
      
   };
   
   /* @} */
   
}} // end namespaces

#endif // end include guard
