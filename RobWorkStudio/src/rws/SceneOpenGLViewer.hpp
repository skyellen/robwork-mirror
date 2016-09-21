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


#ifndef QTGUI_SceneOpenGLViewer_HPP
#define QTGUI_SceneOpenGLViewer_HPP

#ifdef __WIN32
#include <windows.h>
#endif

#include <vector>

#include <QObject>

#include <QGLWidget>

#include <rw/common/PropertyMap.hpp>
#include <rw/math/Vector3D.hpp>

#include <rwlibs/opengl/SceneOpenGL.hpp>

#include <rw/graphics/SceneCamera.hpp>

#include <boost/thread/mutex.hpp>

#include "CameraController.hpp"
#include "SceneViewerWidget.hpp"

namespace rw { namespace graphics { class Render; } }
namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace models { class WorkCell; } }
namespace rwlibs { namespace opengl { class SceneOpenGL; } }

class QMouseEvent;

namespace rws {


class RobWorkStudio;

/**
 * @brief Class representing an OpenGL based QT Widget for 3D visualization of the SceneGraph SceneOpenGL.
 *
 * One of the main responsibilities of the view is to define an opengl context in which
 * the scenegraph can operate and render.
 *
 * The view will add a "view" camera group and a "view" camera to the scene which it will control
 * through its gui events (mouse and keyboard).
 *
 * Multiple camera
 *
 */
class SceneOpenGLViewer: public QGLWidget, public SceneViewerWidget {
    Q_OBJECT

public:
	//! @brief Smart pointer type for SceneOpenGLViewer.
    typedef rw::common::Ptr<SceneOpenGLViewer> Ptr;

    /**
     * @brief Constructor.
     * @param parent [in] the parent widget (the owner of this widget).
     */
    SceneOpenGLViewer(QWidget* parent = 0);

    /**
     * @brief Constructs an OpenGL based QT Widget.
     * @param pmap [in] propertyies for the viewer.
     * @param parent [in] the parent widget (the owner of this widget).
     */
    SceneOpenGLViewer(rw::common::PropertyMap& pmap, QWidget* parent = 0);

    //! @brief Destructor.
    virtual ~SceneOpenGLViewer();

    // ----------------- SceneViewerWidget interface --------------------
    //! @copydoc SceneViewerWidget::getRenderInfo
    rw::graphics::SceneGraph::RenderInfo& getRenderInfo(){ return _renderInfo; }

    //! @copydoc SceneViewerWidget::getPivotDrawable
    rw::graphics::DrawableNode::Ptr getPivotDrawable(){ return _pivotDrawable; }

    //! @copydoc SceneViewerWidget::getWidget
    QWidget* getWidget(){ return this; }

    // ----------------- SceneViewer interface --------------------
    //! @copydoc SceneViewer::getScene
    rw::graphics::SceneGraph::Ptr getScene(){ return _scene; }

    //! @copydoc SceneViewer::getPropertyMap
    rw::common::PropertyMap& getPropertyMap(){return _pmap->getValue();}

    //! @copydoc SceneViewer::getViewCamera
    virtual rw::graphics::SceneCamera::Ptr getViewCamera() { return _mainCam; }

    //! @copydoc SceneViewer::setWorldNode
    void setWorldNode(rw::graphics::GroupNode::Ptr wnode);

    //! @copydoc SceneViewer::getWorldNode
    rw::graphics::GroupNode::Ptr getWorldNode(){ return _worldNode; }

    //! @copydoc SceneViewer::createView
    virtual View::Ptr createView(const std::string& name, bool enableBackground=false);

    //! @copydoc SceneViewer::getMainView
    virtual View::Ptr getMainView(){ return _mainView; }

    //! @copydoc SceneViewer::destroyView
    virtual void destroyView(View::Ptr view);

    //! @copydoc SceneViewer::selectView
    virtual void selectView(View::Ptr view);

    //! @copydoc SceneViewer::getCurrentView
    virtual View::Ptr getCurrentView(){ return _currentView; };

    //! @copydoc SceneViewer::getViews
    virtual std::vector<View::Ptr> getViews(){ return _views; };

    //! @copydoc SceneViewer::renderView
    void renderView(View::Ptr);

    //! @copydoc SceneViewer::updateState
    void updateState(const rw::kinematics::State& state);

    //! @copydoc SceneViewer::updateView
    void updateView(){
        updateGL();
    }

    //! @copydoc SceneViewer::getViewCenter
    rw::math::Vector3D<> getViewCenter(){
        return _cameraCtrl->getCenter();
    }

    //! @copydoc SceneViewer::setLogo
    void setLogo(const std::string& string) {
        _viewLogo = string;
        updateGL();
    }

    //! @copydoc SceneViewer::getLogo
    const std::string& getLogo() const{ return _viewLogo;}

    //! @copydoc SceneViewer::setTransform
    virtual void setTransform(const rw::math::Transform3D<>& t3d){
        _cameraCtrl->setTransform(t3d);
        getViewCamera()->setTransform(t3d);
        //updateGL();
    };

    //! @copydoc SceneViewer::pickDrawable(int,int)
    rw::graphics::DrawableNode::Ptr pickDrawable(int x, int y);

    //! @copydoc SceneViewer::pickDrawable(rw::graphics::SceneGraph::RenderInfo&,int,int)
    rw::graphics::DrawableNode::Ptr pickDrawable(rw::graphics::SceneGraph::RenderInfo& info, int x, int y);

    /**
     * @brief Saves the current 3D view to disk as either jpg, bmp or png.
     *
     * If failing a std::string is thrown with a detailed description of what
     * when wrong.
     *
     * @param filename [in] Path and name of the file. The filename extension
     * should be either ".jpg", ".bmp" or ".png" to specify which format to use.
     * @param fillR [in] Fill color if viewport is smaller than image, red component [0,255]
     * @param fillG [in] Fill color if viewport is smaller than image, green component [0,255]
     * @param fillB [in] Fill color if viewport is smaller than image, blue component [0,255]
     */
    void saveBufferToFile(const std::string& stdfilename,
                          const int fillR, const int fillG, const int fillB);

    //! @brief Clears the list of Drawables and WorkCells
    void clear();

    /**
     * @brief key pressed listener function. Key events in the opengl view
     * will trigger this method.
     * @param e [in] the event.
     */
    void keyPressEvent(QKeyEvent *e);

    /**
     * @brief set the camera view controller.
     * @param camController
     */
    void setCameraController(CameraController::Ptr camController){
        _cameraCtrl = camController;
    }

    /**
     * @brief Returns the camera controller
     * @brief Camera controller
     */
    CameraController::Ptr getCameraController(){
        return _cameraCtrl;
    }

    /**
     * @brief listener callback for property changed in getPropertyMap
     * @param base
     */
    void propertyChangedListener(rw::common::PropertyBase* base);
    
    /**
     * @brief picks the frame that has drawables that intersect the ray cast into the screen from
     * the screen coordinates \b x and \b y.
     * @param x [in] x coordinate
     * @param y [in] y coordinate
     * @return the frame that was selected, Null if no frames where selected.
     */
    rw::kinematics::Frame* pickFrame(int x, int y);

protected:
    //! Overridden from QGLWidget
    void initializeGL();

    //! Overridden from QGLWidget
    void paintGL();

    //! Overridden from QGLWidget
    void glDraw();

    //! Overridden from QGLWidget
    void resizeGL(int width, int height);

    //! Overridden from QGLWidget
    void mouseDoubleClickEvent(QMouseEvent* event);

    //! Overridden from QGLWidget
    void mousePressEvent(QMouseEvent* event);

    //! Overridden from QGLWidget
    void mouseMoveEvent(QMouseEvent* event);

    //! Overridden from QGLWidget
    void wheelEvent(QWheelEvent* event);

    //! @copydoc SceneViewer::setWorkCellScene
    void setWorkCellScene(rw::common::Ptr<rw::graphics::WorkCellScene> wcscene){
        _wcscene = wcscene;
    }

private:
    void init();

	void propertyUpdated(rw::common::PropertyBase* base);

	//void drawGLBackground();
	//void drawGLForeground();
    //void drawRWLogo();

    void setupCameraView(int camNr, bool setupViewport = true);

private:

	rwlibs::opengl::SceneOpenGL::Ptr _scene;
	rw::common::Ptr<rw::graphics::WorkCellScene> _wcscene;

	// the main camera which is controlled by the gui
    rw::graphics::SceneCamera::Ptr _mainCam, _backCam, frontCam;
    rw::graphics::CameraGroup::Ptr _mainCamGroup;

    View::Ptr _mainView, _currentView;
    std::vector<View::Ptr> _views;

    rw::common::Ptr<rw::kinematics::State> _state;

    rw::common::Ptr<rw::models::WorkCell> _wc;

    QFont _logoFont;
    std::string _viewLogo;

    int _width, _height;
    // Background Color definitions

    rw::common::Property<rw::common::PropertyMap>::Ptr _pmap;
    rw::common::Property<bool>::Ptr _viewBackground;
    rw::common::Property<rw::math::Vector3D<> >::Ptr _backgroundColorTop, _backgroundColorBottom;

    CameraController::Ptr _cameraCtrl;
    rw::common::Ptr<rw::graphics::Render> _backgroundRender;
    rw::graphics::DrawableNode::Ptr _backgroundnode;
    rw::graphics::DrawableGeometryNode::Ptr _pivotDrawable;
    rw::graphics::GroupNode::Ptr _worldNode;
    rw::graphics::SceneGraph::RenderInfo _renderInfo;
    boost::mutex _renderMutex;

};

}

#endif //#ifndef QTGUI_SceneOpenGLViewer_HPP
