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


#ifndef QTGUI_VIEWGL_HPP
#define QTGUI_VIEWGL_HPP

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

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/use_robwork_namespace.hpp>

#include <rwlibs/drawable/WorkCellGLDrawer.hpp>
#include <rwlibs/drawable/Drawable.hpp>
#include <rwlibs/drawable/Render.hpp>
#include <rwlibs/drawable/DrawableUtil.hpp>

#include <boost/shared_ptr.hpp>

#include "CameraController.hpp"
#include "RobWorkStudioPlugin.hpp"


class RobWorkStudio;

/**
 * @brief Class representing an OpenGL based QT Widget for 3D visualization of Drawables
 */
class ViewGL: public QGLWidget {
    Q_OBJECT;

public:
    /**
     * @brief container struct for keeping track of camera parameters
     */
	struct GLCameraView {
		GLCameraView(double fov, int w, int h, rw::kinematics::Frame* f):
			fovy(fov),height(h),width(w),vnear(0.1),vfar(100),frame(f)
		{}
		double fovy; // in degree
		int height; // in pixels
		int width; // in pixels
		double vnear,vfar; // near clipping plane
		rw::kinematics::Frame *frame; // camera frame
	};

	struct GLLightSource {
	    GLenum light;
	    float pos[4];
	    float ambient[4];
	    float diffuse[4];
	    void init(){
            glLightfv(light, GL_AMBIENT, ambient);
            glLightfv(light, GL_DIFFUSE, diffuse);
            //glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
            glLightfv(light, GL_POSITION, pos);
	    }
	    void enable(){ glEnable(light);}
	    void disable(){ glDisable(light);}
	};

    /**
     * @brief Constructs an OpenGL based QT Widget
     * @param rwStudio [in] robworkstudio
     * @param parent [in] Parent widget
     */
    ViewGL(RobWorkStudio* rwStudio, QWidget* parent = 0);

    /**
     * @brief destructor
     */
    ~ViewGL();

    /**
     * @brief sets up the toolbar
     */
    void setupToolBar(QToolBar* toolbar);

    /**
     * @brief sets up menu
     */
    void setupMenu(QMenu* menu);

    /**
     * @brief adds a camera view to the scene
     */
    void addCameraView(const GLCameraView& view){
    	_cameraViews.push_back(view);
    }

    /**
     * @brief sets the active camera
     * @param i [in] the index of the camera to activate. When i==0 the main
     * free flying user camera s selected
     */
    bool setCamera(int i){
    	if(i<0 || i>=(int)_cameraViews.size()){
    		return false;
    	}
    	_cameraNr = i;
    	_cameraViewChanged = true;
    }

    /**
     * @brief Adds a Drawable to the view component.
     *
     * The ViewGL does not take control of the drawable, therefore it is the
     * callers reponsibility to destroy the Drawable when not being used
     * anymore.
     *
     * Likewise it is the callers reponsibility not to delete the Drawable
     * before being removed from the ViewGL Widget.
     *
     * @param drawable [in] Pointer to drawable to be visualized
     */
    void addDrawable(rwlibs::drawable::Drawable* drawable);

    /**
     * @brief init the ViewGL with a workcell
     * @param workcell [in] the workcell to be drawn
     * @param state [in] the initial state of the workcell
     * @param collisionDetector [in] used to detect collisions in the scene
     */
    void addWorkCell(rw::models::WorkCell* workcell,
                     rw::kinematics::State* state,
                     rw::proximity::CollisionDetector* collisionDetector);

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
     */
    void saveBufferToFile(const QString& filename);

    /**
     * @brief Specified whether to visualize the pivot point
     * @param visible [in] True for showing pivot point, false otherwise.
     */
    void showPivotPoint(bool visible);

    /**
     * @brief Sets whether to visualize as SOLID, WIRE or BOTH.
     * @param drawType [in] The drawtype to use
     */
    void setDrawType(rwlibs::drawable::Render::DrawType drawType);

    /**
     * @brief key pressed listener function. Key events in the opengl view
     * will trigger this method.
     */
    void keyPressEvent(QKeyEvent *e);

    bool isCheckForCollisionEnabled(){ return _checkForCollision->isChecked();};

    void setCameraController(CameraControllerPtr camController){
        _cameraCtrl = camController;
    }

    CameraControllerPtr getCameraController(){
        return _cameraCtrl;
    }

    /**
     * @brief Draws GL stuff...
     * @param showPivot [in] True for showing pivot point, false otherwise.
     */
    void drawGLStuff(bool showPivot);

    /**
     * @brief Returns the list of camera views
     */
    std::vector<GLCameraView>& getCameraViews(){return _cameraViews;}

    /**
     * @brief Returns the camera number
     */
    int getCameraNr(){return _cameraNr;}

    /**
     * @brief Returns the zoom scale
     */
    float getZoomScale(){return _zoomScale;}

public slots:
    /**
     * @brief Sets whether to check for collision
     * @param check [in] whether to check collision or not
     */
    void setCheckForCollision(bool check);

protected:
    /* Overridden methods from QGLWidget
     */
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    void mouseDoubleClickEvent(QMouseEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);

private slots:
    void setDrawTypeSlot();
    void setTransparentSlot();
    void showPivotPointSlot();

    // Save buffer dialog.
    void saveBufferToFileQuery();

private:
	void drawGLBackground();
	void setupCameraView(int camNr, bool setupViewport = true);

	rw::kinematics::Frame* pickFrame(int x, int y);

    std::vector<rwlibs::drawable::Drawable*> _drawables;

    std::vector<GLCameraView> _cameraViews;
    std::vector<GLLightSource> _lights;

    struct Cell
    {
        // And we need a collision detector also.
        Cell(
            robwork::WorkCell* workcell,
            robwork::State* state,
            rw::proximity::CollisionDetector* detector)
            :
            workcell(workcell),
            state(state),
            detector(detector)
        {}

        Cell() :
            workcell(),
            state(),
            collisionPairs(),
            detector()
        {}

        operator bool () { return workcell && state && detector; }

        robwork::WorkCell* workcell;
        robwork::State* state;
        rw::kinematics::FramePairSet collisionPairs;
        rw::proximity::CollisionDetector* detector;
    };

    std::vector<rwlibs::drawable::Drawable*> getAllDrawables(const Cell& cell);

    Cell _cell;

    robwork::Rotation3D<float> _viewRotation;
    robwork::Vector3D<float> _viewPos;
    robwork::Vector3D<float> _lastViewPos;
    robwork::Vector3D<float> _pivotPoint;
    robwork::Vector3D<float> _lastPos;

    rwlibs::drawable::Render::DrawType _drawType;
    float _alpha;

    GLUquadricObj* _sphereObj;

    bool _showPivotPoint;
    int _width, _height;
    CameraControllerPtr _cameraCtrl;

    float _zoomFactor;
    float _zoomScale;

    QAction* _showSolidAction;
    QAction* _showWireAction;
    QAction* _showOutlineAction;
    QAction* _showTransparentAction;
    QAction* _showPivotPointAction;
    QAction* _checkForCollision;
    QAction* _saveBufferToFileAction;

    RobWorkStudio* _rwStudio;
    rwlibs::drawable::WorkCellGLDrawer* _workcellGLDrawer;
    int _cameraNr;
    QFont _logoFont;
    const QString _viewLogo;
    bool _cameraViewChanged;
};

#endif //#ifndef QTGUI_VIEWGL_HPP
