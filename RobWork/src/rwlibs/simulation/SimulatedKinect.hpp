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

#ifndef RWLIBS_SIMULATION_SimulatedKinect_HPP_
#define RWLIBS_SIMULATION_SimulatedKinect_HPP_

//! @file SimulatedKinect.hpp

#include <rw/common/Ptr.hpp>
#include "SimulatedSensor.hpp"
#include <rw/graphics/SceneViewer.hpp>
#include <rw/sensor/CameraModel.hpp>
#include <rw/sensor/Scanner25DModel.hpp>

namespace rw { namespace sensor { class Scanner25D; } }

namespace rwlibs { namespace simulation {
    //! @addtogroup simulation
	// @{

    /**
     * @brief Simulates a Kinnect grabbing RGBD images.
     */
    class SimulatedKinect : public SimulatedSensor
    {
    public:
        /**
         * @brief constructor
         * @param name [in] name of this simulated scanner
         */
    	SimulatedKinect(const std::string& name, rw::kinematics::Frame *frame);

        /**
         * @brief constructor
         * @param name [in] name of this simulated scanner
         * @param desc [in] description of this scanner
         */
    	SimulatedKinect(const std::string& name, const std::string& desc, rw::kinematics::Frame *frame);

    	/**
    	 * @brief constructor
    	 * @param camModel [in] the camera model to use
    	 * @param scannerModel [in] the scanner model to use
    	 */
    	SimulatedKinect(rw::sensor::CameraModel::Ptr camModel, rw::sensor::Scanner25DModel::Ptr scannerModel);

    	/**
    	 * @brief destructor
    	 */
    	virtual ~SimulatedKinect();


    	void init(rw::graphics::SceneViewer::Ptr drawer);

    	/**
    	 * @brief set the framerate in frames per sec.
    	 * @param rate [in] frames per sec
    	 */
    	void setFrameRate(double rate);

    	///////////// below is inheritet functions form Scanner25D and Sensor

    	//! @copydoc Scanner25D::open
        void open();

        //! @copydoc Scanner25D::isOpen
        bool isOpen();

        //! @copydoc Scanner25D::close
        void close();

        //! @copydoc Scanner25D::acquire
        void acquire();

        //! @copydoc Scanner25D::isScanReady
        bool isDataReady();

        //! @copydoc Scanner25D::getRange
        std::pair<double,double> getRange() const;

        //! @copydoc Scanner25D::getFrameRate
        double getFrameRate() const;

        //! @copydoc Scanner25D::getImage
    	const rw::geometry::PointCloud& getScan();

    	const rw::sensor::Image& getImage();

    	//! @copydoc SimulatedSensor::update
        void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state);

    	//! @copydoc SimulatedSensor::reset
    	void reset(const rw::kinematics::State& state);

    	//! @copydoc SimulatedSensor::getSensor
        rw::sensor::Sensor::Ptr getSensorHandle(rwlibs::simulation::Simulator::Ptr simulator);

        /**
         * @brief set to true to enable realistic noise on the scan.
         * @param enabled [in]
         */
        void setNoiseEnabled(bool enabled){ _noiseEnabled = enabled; };

        /**
         * @brief returns the vertical field of view
         * @return the vertical field of view
         */
        double getVerticalFieldOfView() const { return _fieldOfView;}

        /**
         * @brief returns the width of the image
         * @return the width of the image
         */
        int getWidth() const { return _width; }

        /**
         * @brief returns the height of the image
         * @return the height of the image
         */
        int getHeight() const { return _height; }

        /**
         * @brief get the model of the camera of this kinect
         */
        rw::sensor::CameraModel::Ptr getCameraModel(){ return _camModel; }

        /**
         * @brief get the model of the range scannger of this kinect
         */
        rw::sensor::Scanner25DModel::Ptr getScannerModel(){ return _scannerModel; }

    private:
        rw::sensor::CameraModel::Ptr _camModel;
        rw::sensor::Scanner25DModel::Ptr _scannerModel;

        double _frameRate, _dtsum;
        bool _isAcquired,_isOpenned, _noiseEnabled;
        rw::common::Ptr<rw::sensor::Scanner25D> _rsensor;

        rw::graphics::SceneViewer::Ptr _drawer;
        rw::graphics::SceneViewer::View::Ptr _view;

        double _near, _far;

        double _fieldOfView; // in the y-axis
        bool _grabSingleFrame;
        int _width, _height;

        rw::sensor::Image::Ptr _img;
        rw::geometry::PointCloud::Ptr _scan;



    };

    //! @}
}
}

#endif /* SimulatedKinect_HPP_ */
