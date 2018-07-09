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


#ifndef RW_SENSOR_CAMERAMODEL_HPP
#define RW_SENSOR_CAMERAMODEL_HPP

/**
 * @file CameraModel.hpp
 */

#include "Image.hpp"
#include "SensorModel.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/math/ProjectionMatrix.hpp>

#include <string>

namespace rw { namespace kinematics { class State; } }

namespace rw { namespace sensor {

    /** @addtogroup sensor */
    /* @{ */

    /**
     * @brief The CameraModel class defines a generel pinhole camera model where
     * camera parameters and state values are stored.
     *
     */
    class CameraModel : public SensorModel
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<CameraModel> Ptr;
		//! @brief smart pointer type to this const class
		typedef rw::common::Ptr<const CameraModel> CPtr;

    public:

		/**
		 * constructor
		 * @param projection [in] pinhole projection model
		 * @param name [in] name of camera
		 * @param frame [in] frame that camera is attached/referenced to
		 * @param modelInfo [in] text description of the camera
		 */
		CameraModel(
        			const rw::math::ProjectionMatrix& projection,
        			const std::string& name,
        			rw::kinematics::Frame* frame,
        			const std::string& modelInfo = "");

        /**
         * @brief destructor
         */
        virtual ~CameraModel();

        /**
         * @brief returns the image if it has been saved in the State. Else null is
         * returned.
         * @return last image captured from camera.
         */
        Image::Ptr getImage(const  rw::kinematics::State& state);

        /**
         * @brief set the image in the state
         * @param img [in] image to set in state
         * @param state [in/out] the state in which to set the image.
         */
        void setImage(Image::Ptr img, rw::kinematics::State& state);

        //! get the camera projection matrix
        rw::math::ProjectionMatrix getProjectionMatrix() const;

        //!@brief get horisontal field of view in degrees.
        double getFieldOfViewX() const;

        //!@brief get vertical field of view in degrees.
        double getFieldOfViewY() const;

        ///// a list of features that most of the time is available
        //! @brief get far clipping plane
        double getFarClippingPlane() const;
        //! @brief get near clipping plane
        double getNearClippingPlane() const;


    protected:

        //! CameraModelCache that define data to store in the State
        class CameraModelCache: public rw::kinematics::StateCache {
    	public:
    		typedef rw::common::Ptr<CameraModelCache> Ptr;
    		rw::common::Ptr<rw::sensor::Image> _image;

    		//! constructor
    		CameraModelCache()
    		{
    		};

    		//! @copydoc rw::kinematics::StateCache::size
    		size_t size() const{
    			if(_image!=NULL)
    				return _image->getDataSize();
    			return 0;
    		};

    		//! @copydoc rw::kinematics::StateCache::clone
    		virtual rw::common::Ptr<StateCache> clone() const{
    			CameraModelCache::Ptr cache = rw::common::ownedPtr( new CameraModelCache(*this) );
    		    if(_image!=NULL)
    		    	cache->_image = rw::common::ownedPtr( new Image( *_image ));
    			return cache;
    		};
    	};

    private:
        //! name of camera model information
        rw::math::ProjectionMatrix _pmatrix;
        rw::kinematics::StatelessData<int> _sdata;
    };

    /* @} */

}} // end namespaces

#endif // end include guard
