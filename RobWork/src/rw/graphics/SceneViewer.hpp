
#ifndef SCENEVIEWER_HPP_
#define SCENEVIEWER_HPP_

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

#include "SceneGraph.hpp"
#include "SceneCamera.hpp"

#include <rw/common/PropertyMap.hpp>
#include <rw/proximity/CollisionDetector.hpp>


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

        virtual ~SceneViewer(){};

        //! @brief get the current scene graph
        virtual SceneGraph::Ptr getScene() = 0;

        //! @brief get the logo that is displayed in the 3d scene
        virtual const std::string& getLogo() const = 0;

        //! @brief set the logo that is displayed in the 3d scene
        virtual void setLogo(const std::string& string) = 0;

        //! @brief get propertymap
        virtual rw::common::PropertyMap& getPropertyMap() = 0;

        virtual void updateView() = 0;

        virtual void updateState(const rw::kinematics::State& state) = 0;

        virtual void setWorldNode(rw::graphics::GroupNode::Ptr wnode) = 0;
        virtual rw::graphics::GroupNode::Ptr getWorldNode() = 0;

        //// ---------------- SceneCamera functions
        /* A view allways has one camera attached, this is the getSceneCamera(). Besides that a number
         * of slave cameras can be attached. These are manipulated through getSlaveSceneCamera()
         */
        // get view camera
        virtual SceneCamera::Ptr getViewCamera() = 0;

        virtual rw::math::Vector3D<> getViewCenter() = 0;

        virtual DrawableNode::Ptr pickDrawable(int x, int y) = 0;

        // get/create a slave camera
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

    private:
        PositionSelectedEvent _positionSelectedEvent;
    };
}
}
#endif /* SceneViewer_HPP_ */
