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

#ifndef RW_STUDIO_SENSORVIEW
#define RW_STUDIO_SENSORVIEW

#include <QWidget>

#include <rw/common/Ptr.hpp>

namespace rw { namespace sensor { class Camera; } }
namespace rwlibs { namespace opengl { class RenderScan; } }
namespace rwlibs { namespace simulation { class SimulatedScanner2D; } }
namespace rwlibs { namespace simulation { class SimulatedScanner25D; } }

class ImageView;

namespace rws {

//! @brief A Qt widget for visualization of sensors.
class SensorView: public QWidget {
    Q_OBJECT
public:
	//! @brief Smart pointer type for SensorView.
	typedef rw::common::Ptr<SensorView> Ptr;

	/**
	 * @brief Construct new widget.
	 * @param parent [in] owner widget.
	 */
    SensorView(QWidget* parent = NULL):
        QWidget(parent)
    {
    }

    //! @brief Destructor.
    virtual ~SensorView() {}

    //! @brief Update the view.
    virtual void update() = 0;

    //! @brief Make the widget current.
    virtual void makeCurrent() = 0;
protected:
    /**
     * @brief Handle close event.
     * @param event [in] the event.
     */
    void closeEvent(QCloseEvent* event);

signals:
	/**
	 * @brief Signal emitted when view is closed.
	 * @param widget [in] the view just closed.
	 */
    void viewClosed(SensorView* widget);
};




//! @brief a view to visualize output of a camera
class CameraView: public SensorView {
public:
	/**
	 * @brief Create a camera view.
	 * @param camera [in] the camera sensor.
	 * @param parent [in] owner widget.
	 */
	CameraView(rw::common::Ptr<rw::sensor::Camera> camera, QWidget* parent = NULL);

	//! @copydoc SensorView::update
    virtual void update();

	//! @copydoc SensorView::makeCurrent
    virtual void makeCurrent() {}

private:
	rw::common::Ptr<rw::sensor::Camera> _camera;
    ImageView* _pImageView;    
};


//! @brief a view to visualize the output of 2.5D scanners.
class Scan25DView: public SensorView {
public:
	/**
	 * @brief Create a 2.5D scanner view.
	 * @param parent [in] owner widget.
	 */
    Scan25DView(QWidget* parent = NULL);

    /**
     * @brief Initialize view.
     * @param scanner [in] the simulated scanner.
     */
	virtual void initialize(rw::common::Ptr<rwlibs::simulation::SimulatedScanner25D> scanner);

	//! @copydoc SensorView::update
    virtual void update();

	//! @copydoc SensorView::makeCurrent
    virtual void makeCurrent();

private:
    rw::common::Ptr<rwlibs::simulation::SimulatedScanner25D> _scanner;
    rw::common::Ptr<rwlibs::opengl::RenderScan> _scanRender;
	ImageView* _pImageView;

};

//! @brief a view to visualize the output of 2D scanners.
class Scan2DView: public SensorView {
public:
	/**
	 * @brief Create a 2D scanner view.
	 * @param parent [in] owner widget.
	 */
    Scan2DView(QWidget* parent = NULL);

    /**
     * @brief Initialize view.
     * @param scanner [in] the simulated scanner.
     */
    virtual void initialize(rw::common::Ptr<rwlibs::simulation::SimulatedScanner2D> scanner);

	//! @copydoc SensorView::update
    virtual void update();

	//! @copydoc SensorView::makeCurrent
    virtual void makeCurrent();

private:
    rw::common::Ptr<rwlibs::simulation::SimulatedScanner2D> _scanner;
	rw::common::Ptr<rwlibs::opengl::RenderScan> _scanRender;
	ImageView* _pImageView;

};

} //end namespace rws;

#endif
