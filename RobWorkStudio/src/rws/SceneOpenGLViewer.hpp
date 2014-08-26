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
#include <QString>

#include <QGLWidget>

#include <QMouseEvent>
#include <QMessageBox>
#include <QFileDialog>
#include <QImage>
#include <QAction>
#include <QToolBar>
#include <QMenu>

#include <rw/common/PropertyMap.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/proximity/CollisionDetector.hpp>

#include <rwlibs/opengl/SceneOpenGL.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwlibs/opengl/DrawableUtil.hpp>

#include <rw/graphics/Render.hpp>
#include <rw/graphics/SceneCamera.hpp>

#include <boost/shared_ptr.hpp>

#include "CameraController.hpp"
#include "RobWorkStudioPlugin.hpp"
#include "SceneViewerWidget.hpp"

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
    Q_OBJECT;

public:

    typedef rw::common::Ptr<SceneOpenGLViewer> Ptr;

    SceneOpenGLViewer(QWidget* parent = 0);

    /**
     * @brief Constructs an OpenGL based QT Widget
     * @param rwStudio [in] robworkstudio
     * @param parent [in] Parent widget
     */
    SceneOpenGLViewer(rw::common::PropertyMap& pmap, QWidget* parent = 0);

    /**
     * @brief destructor
     */
    virtual ~SceneOpenGLViewer();

    // ----------------- WorkCellViewer interface --------------------
    rw::graphics::SceneGraph::Ptr getScene(){ return _scene; }

    rw::common::PropertyMap& getPropertyMap(){return _pmap->getValue();}

    virtual rw::graphics::SceneCamera::Ptr getViewCamera() { return _mainCam; }

    void setWorldNode(rw::graphics::GroupNode::Ptr wnode);

    rw::graphics::GroupNode::Ptr getWorldNode(){ return _worldNode; }


    virtual View::Ptr createView(const std::string& name, bool enableBackground=false);
    virtual View::Ptr getMainView(){ return _mainView; }
    virtual void destroyView(View::Ptr view);
    virtual void selectView(View::Ptr view);
    virtual View::Ptr getCurrentView(){ return _currentView; };
    virtual std::vector<View::Ptr> getViews(){ return _views; };
    void renderView(View::Ptr);

    rw::graphics::DrawableNode::Ptr getPivotDrawable(){
        return _pivotDrawable;
    }

    // get/create a slave camera
    //virtual rwlibs::drawable::SceneCamera::Ptr getSlaveCamera(const std::string& name){ return _cameraViews[0]; }
    //virtual int getNrSlaveCameras(){ return _cameraViews.size(); }
    //virtual std::vector<std::string> getSlaveCameraNames(){ return std::vector<std::string>(); }

    //virtual rwlibs::drawable::SceneCamera::Ptr addSlaveCamera(const std::string& name){ return NULL;}
    //virtual void removeSlaveCamera(const std::string& name){ }

    //virtual rwlibs::drawable::SceneCamera::Ptr addSlaveCamera(const std::string& name){ return NULL;}
    /**
     * @brief the current camera can be either the view camera or one of the slave cameras
     * @return
     */
    //virtual rwlibs::drawable::SceneCamera::Ptr getCurrentCamera(){ return _cameraViews[0];}
    //virtual void setCurrentCamera(const std::string& name){};

    void updateState(const rw::kinematics::State& state) {
        if(_state==NULL)
            _state = rw::common::ownedPtr(new rw::kinematics::State());
        *_state = state;
        _renderInfo._state = _state.get();
    }

    void updateView(){
        updateGL();
    }

    QWidget* getWidget(){ return this; }


    /**
     * @brief Clears the list of Drawables and WorkCells
     */
    void clear();

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

    /**
     * @brief key pressed listener function. Key events in the opengl view
     * will trigger this method.
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

    const std::string& getLogo() const{ return _viewLogo;}

    rw::math::Vector3D<> getViewCenter(){
        return _cameraCtrl->getCenter();
    }

    void setLogo(const std::string& string) {
        _viewLogo = string;
        updateGL();
    }

    virtual void setTransform(const rw::math::Transform3D<>& t3d){
        _cameraCtrl->setTransform(t3d);
        getViewCamera()->setTransform(t3d);
        //updateGL();
    };


    rw::graphics::DrawableNode::Ptr pickDrawable(int x, int y);
    rw::graphics::DrawableNode::Ptr pickDrawable(rw::graphics::SceneGraph::RenderInfo& info, int x, int y);
    rw::graphics::SceneGraph::RenderInfo& getRenderInfo(){ return _renderInfo; };
    
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

    void setWorkCellScene(rw::graphics::WorkCellScene::Ptr wcscene){
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
	rw::graphics::WorkCellScene::Ptr _wcscene;

	// the main camera which is controlled by the gui
    rw::graphics::SceneCamera::Ptr _mainCam, _backCam, frontCam;
    rw::graphics::CameraGroup::Ptr _mainCamGroup;

    View::Ptr _mainView, _currentView;
    std::vector<View::Ptr> _views;

    rw::common::Ptr<rw::kinematics::State> _state;

    rw::models::WorkCell::Ptr _wc;

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
