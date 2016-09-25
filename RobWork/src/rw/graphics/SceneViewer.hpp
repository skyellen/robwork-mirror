
#ifndef RW_GRAPHICS_SCENEVIEWER_HPP_
#define RW_GRAPHICS_SCENEVIEWER_HPP_

#include <rw/math/Vector3D.hpp>

#include "SceneGraph.hpp"
#include "SceneCamera.hpp"

#include <rw/common/PropertyMap.hpp>

namespace rw {
namespace graphics {

    /**
     * @brief interface for viewing a scene graph.
     *
     * The scene graph viewer
     */
    class SceneViewer {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<SceneViewer> Ptr;

        //! @brief Destructor.
        virtual ~SceneViewer(){}

        //! @brief get the current scene graph
        virtual SceneGraph::Ptr getScene() = 0;

        //! @brief get the logo that is displayed in the 3d scene
        virtual const std::string& getLogo() const = 0;

        //! @brief set the logo that is displayed in the 3d scene
        virtual void setLogo(const std::string& string) = 0;

        //! @brief get propertymap
        virtual rw::common::PropertyMap& getPropertyMap() = 0;


        //! @brief Update the view.
        virtual void updateView() = 0;

        /**
         * @brief Set a new state.
         * @param state [in] new state.
         */
        virtual void updateState(const rw::kinematics::State& state) = 0;

        /**
         * @brief Set the world group node.
         * @param wnode [in] the world node.
         */
        virtual void setWorldNode(rw::graphics::GroupNode::Ptr wnode) = 0;
        
        /**
         * @brief Set the WorkCell scene.
         * @param wcscene [in] the workcell scene.
         */
        virtual void setWorkCellScene(rw::common::Ptr<class WorkCellScene> wcscene) { _wcscene = wcscene; }

        /**
         * @brief Get the world node.
         * @return the world node.
         */
        virtual rw::graphics::GroupNode::Ptr getWorldNode() = 0;

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
        virtual void saveBufferToFile(const std::string& filename,
                                      const int fillR, const int fillG, const int fillB) = 0;

        //// ---------------- View functions
        //! @copydoc A view.
        struct View {
        	//! @brief Smart pointer for a View.
            typedef rw::common::Ptr<View> Ptr;

            /**
             * @brief Construct new view.
             * @param name [in] name of the view.
             */
            View(const std::string& name):
                _name(name),
                _drawType(DrawableNode::SOLID),
                _drawMask(DrawableNode::Physical | DrawableNode::DrawableObject)
                {};

            //! @brief Name of the view.
            std::string _name;
            //! @brief The draw type.
            rw::graphics::DrawableNode::DrawType _drawType;
            //! @brief The draw mask.
            int _drawMask;
            //! @brief The scene camera for the view.
            SceneCamera::Ptr _viewCamera;
            //! @brief The camera group.
            CameraGroup::Ptr _camGroup;
        };



        /* A view always has one camera attached, this is the getSceneCamera(). Besides that a number
         * of slave cameras can be attached. These are manipulated through getSlaveSceneCamera()
         */

        /**
         * @brief Get the view camera.
         * @return a scene camera.
         */
        virtual SceneCamera::Ptr getViewCamera() = 0;

        /**
         * @brief Get the view center.
         * @return the center.
         */
        virtual rw::math::Vector3D<> getViewCenter() = 0;
        //! @copydoc SceneGraph::pickDrawable(int,int)
        virtual DrawableNode::Ptr pickDrawable(int x, int y) = 0;
        //! @copydoc SceneGraph::pickDrawable(SceneGraph::RenderInfo&,int,int)
        virtual DrawableNode::Ptr pickDrawable(SceneGraph::RenderInfo& info, int x, int y) = 0;

        // get/create a slave camera
        /**
         * @brief Create a new view.
         * @param name [in] name of view.
         * @param enableBackground [in] (optional) enable the background. Default is false.
         * @return the new view.
         */
        virtual View::Ptr createView(const std::string& name, bool enableBackground=false) = 0;

        /**
         * @brief Get the main view.
         * @return main view.
         */
        virtual View::Ptr getMainView() = 0;

        /**
         * @brief Destroy view.
         * @param view [in] the view to destroy.
         */
        virtual void destroyView(View::Ptr view) = 0;

        /**
         * @brief Select a view.
         * @param view [in] the view to select.
         */
        virtual void selectView(View::Ptr view) = 0;

        /**
         * @brief Get the currently selected view.
         * @return the view.
         */
        virtual View::Ptr getCurrentView() = 0;

        /**
         * @brief Get all views.
         * @return the views.
         */
        virtual std::vector<View::Ptr> getViews() = 0;

        /**
         * @brief Render a view.
         * @param view [in] the view to render
         */
        virtual void renderView(View::Ptr view) = 0;

        //virtual SceneCamera::Ptr getSlaveCamera(const std::string& name) = 0;
        //virtual int getNrSlaveCameras() = 0;
        //virtual std::vector<std::string> getSlaveCameraNames() = 0;

        //virtual SceneCamera::Ptr addSlaveCamera(const std::string& name) = 0;
        //virtual void removeSlaveCamera(const std::string& name) = 0;

        /**
         * @brief the current camera can be either the view camera or one of the slave cameras
         * @return
         */
        //virtual SceneCamera::Ptr getCurrentCamera() = 0;
        //virtual void setCurrentCamera(const std::string& name) = 0;

        /**
         * @brief Defines a listener for position change events
         *
         * These listeners are called when user has selected a point in the 3D view.
         *
         * To send an event double clicking left mouse button on the point of interest while pressing shift.
         *
         * Example use in a plugin: See RobWorkStudio::StateChangedListener
         */
        typedef boost::function<void(const rw::math::Vector3D<>&)> PositionSelectedListener;

        /**
         * @brief Defines event for PositionChanged.
         */
        typedef rw::common::Event<PositionSelectedListener, const rw::math::Vector3D<>&> PositionSelectedEvent;

        /**
         * @brief Returns PositionChangedEvent object needed for subscription to and firing of event
         * @return REference to the PositionSelectedEvent
         */
        PositionSelectedEvent& positionSelectedEvent() {
            return _positionSelectedEvent;
        }

        /// ----------------- ABSTRACT view manipulation Functions, use getViewCamera()
        /**
         * @brief set the orientation of the view. The view will look in the
         * positive direction of the z-axis, with x-axis as the width and
         * the y-axis as the height. Origin of view is in the center of the
         * image.
         * @param rot [in] rotation relative to world
         */
        virtual void setTransform(const rw::math::Transform3D<>& t3d){ getViewCamera()->setTransform(t3d); };

        /**
         * @brief get the current rotation of the view
         * @return orientation of the view
         */
        virtual rw::math::Transform3D<> getTransform() { return getViewCamera()->getTransform(); };

        //virtual void setWorkCellScene(rw::graphics::WorkCellScene::Ptr wcscene) = 0;


    protected:
        //! @brief Event for selection of a position.
        PositionSelectedEvent _positionSelectedEvent;
        //! @brief The WorkCell scene.
        rw::common::Ptr<class WorkCellScene> _wcscene;
        
        //View::Ptr _mainView;
        //std::vector<View::Ptr> _views;
    };
}
}
#endif /* SceneViewer_HPP_ */
