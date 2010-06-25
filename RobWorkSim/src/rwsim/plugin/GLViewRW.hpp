/************************************************************************
 * RobWorkStudio Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * This Software is developed using the Qt Open Source Edition, and is
 * therefore only available under the GNU General Public License (GPL).
 *
 * RobWorkStudio can be used, modified and redistributed freely.
 * RobWorkStudio is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWorkStudio relies on RobWork, which has a different
 * license. For more information goto your RobWork directory and read license.txt.
 ************************************************************************/

#ifndef GLVIEWRW_HPP
#define GLVIEWRW_HPP

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

#include <rws/ArcBall.hpp>
#include <rws/RobWorkStudioPlugin.hpp>


class RobWorkStudio;

/**
 * @brief Class representing an OpenGL based QT Widget for 3D visualization of Drawables
 */
class GLViewRW: public QGLWidget {
    Q_OBJECT;

public:
    /**
     * @brief Constructs an OpenGL based QT Widget
     * @param rwStudio [in] robworkstudio
     * @param parent [in] Parent widget
     */
    GLViewRW(QWidget* parent = 0);

    /**
     * @brief destructor
     */
    ~GLViewRW();

    /**
     * @brief sets up the toolbar
     */
    void setupToolBar(QToolBar* toolbar);

    /**
     * @brief sets up menu
     */
    void setupMenu(QMenu* menu);

    /**
     * @brief Adds a Drawable to the view component.
     *
     * The GLViewRW does not take control of the drawable, therefore it is the
     * callers reponsibility to destroy the Drawable when not being used
     * anymore.
     *
     * Likewise it is the callers reponsibility not to delete the Drawable
     * before being removed from the GLViewRW Widget.
     *
     * @param drawable [in] Pointer to drawable to be visualized
     */
    void addDrawable(rwlibs::drawable::Drawable* drawable);

    /**
     * @brief Clears the list of Drawables
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
     * @brief
     */
    void keyPressEvent(QKeyEvent *e);

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
	void drawGLStuff(bool showPivot);

	rw::kinematics::Frame* pickFrame(int x, int y);

    std::vector<rwlibs::drawable::Drawable*> _drawables;

    robwork::Rotation3D<float> _viewRotation;
    robwork::Vector3D<float> _viewPos;
    robwork::Vector3D<float> _lastViewPos;
    robwork::Vector3D<float> _pivotPoint;
    robwork::Vector3D<float> _lastPos;

    rwlibs::drawable::Render::DrawType _drawType;
    float _alpha;

    GLUquadricObj* _sphereObj;
    rws::ArcBall _arcBall;
    bool _showPivotPoint;
    int _width, _height;

    float _zoomFactor;
    float _zoomScale;

    QAction* _showSolidAction;
    QAction* _showWireAction;
    QAction* _showOutlineAction;
    QAction* _showTransparentAction;
    QAction* _showPivotPointAction;
    QAction* _checkForCollision;
    QAction* _saveBufferToFileAction;

    rwlibs::drawable::WorkCellGLDrawer* _workcellGLDrawer;
    QFont _logoFont;
    const QString _viewLogo;
    bool _cameraViewChanged;
};

#endif //#ifndef QTGUI_GLViewRW_HPP
