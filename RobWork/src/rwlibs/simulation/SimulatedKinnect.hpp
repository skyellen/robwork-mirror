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

#ifndef RWLIBS_SIMULATION_SimulatedKinnect_HPP_
#define RWLIBS_SIMULATION_SimulatedKinnect_HPP_

//! @file SimulatedKinnect.hpp

#include <rw/common/Ptr.hpp>
#include <rw/sensor/Scanner25D.hpp>
#include "SimulatedSensor.hpp"
#include "FrameGrabber25D.hpp"
#include "SimulatedScanner25D.hpp"
#include <rw/graphics/SceneViewer.hpp>


namespace rwlibs { namespace simulation {
    //! @addtogroup simulation
	// @{

    /**
     * @brief
     */
    class SimulatedKinnect : public SimulatedSensor
    {
    public:
        /**
         * @brief constructor
         * @param name [in] name of this simulated scanner
         */
    	SimulatedKinnect(const std::string& name,
    	                 rw::kinematics::Frame *frame);

        /**
         * @brief constructor
         * @param name [in] name of this simulated scanner
         * @param desc [in] description of this scanner
         */
    	SimulatedKinnect(const std::string& name,
                         const std::string& desc,
                         rw::kinematics::Frame *frame);

    	/**
    	 * @brief destructor
    	 */
    	virtual ~SimulatedKinnect();


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
        std::pair<double,double> getRange();

        //! @copydoc Scanner25D::getFrameRate
        double getFrameRate();

        //! @copydoc Scanner25D::getImage
    	const rw::sensor::Image25D& getScan();

    	const rw::sensor::Image& getImage();

    	//! @copydoc SimulatedSensor::update
        void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state);

    	//! @copydoc SimulatedSensor::reset
    	void reset(const rw::kinematics::State& state);

    	//! @copydoc SimulatedSensor::getSensor
        rw::sensor::Sensor::Ptr getSensor(){ return _rsensor;}

        /**
         * @brief set to true to enable realistic noise on the scan.
         * @param enabled [in]
         */
        void setNoiseEnabled(bool enabled){ _noiseEnabled = enabled; };

    private:
        double _frameRate, _dtsum;
        bool _isAcquired,_isOpenned, _noiseEnabled;
        rw::sensor::Scanner25D::Ptr _rsensor;

        rw::graphics::SceneViewer::Ptr _drawer;
        rw::graphics::SceneViewer::View::Ptr _view;

        double _near, _far;

        double _fieldOfView; // in the y-axis
        bool _grabSingleFrame;
        int _width, _height;

        rw::sensor::Image::Ptr _img;
        rw::sensor::Image25D::Ptr _scan;


    };

    //! @}
}
}

#endif /* SimulatedKinnect_HPP_ */
