
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/common.hpp>
#include <rw/graphics.hpp>
#ifndef RWLUA_RW_GRAPHICS_HPP
#define RWLUA_RW_GRAPHICS_HPP

namespace rwlua {
namespace rw {
    //! @addtogroup lua
    // @{
    typedef ::rw::graphics::DrawableGeometryNode DrawableGeometryNode;
    typedef ::rw::graphics::DrawableNode DrawableNode;
    typedef ::rw::graphics::GroupNode GroupNode;
    typedef ::rw::graphics::Model3D Model3D;
    //typedef ::rw::graphics::ProjectionMatrix ProjectionMatrix;
    typedef ::rw::graphics::Render Render;
    typedef ::rw::graphics::SceneCamera SceneCamera;
    typedef ::rw::graphics::SceneGraph SceneGraph;
    typedef ::rw::graphics::SceneNode SceneNode;
    typedef ::rw::graphics::SceneViewer SceneViewer;
    typedef ::rw::graphics::WorkCellScene WorkCellScene;

    typedef ::rw::graphics::DrawableGeometryNode::Ptr DrawableGeometryNodePtr;
    typedef ::rw::graphics::DrawableNode::Ptr DrawableNodePtr;
    typedef ::rw::graphics::GroupNode::Ptr GroupNodePtr;
    typedef ::rw::graphics::Model3D::Ptr Model3DPtr;
    typedef ::rw::graphics::Render::Ptr RenderPtr;
    typedef ::rw::graphics::SceneCamera::Ptr SceneCameraPtr;
    typedef ::rw::graphics::SceneGraph::Ptr SceneGraphPtr;
    typedef ::rw::graphics::SceneNode::Ptr SceneNodePtr;
    typedef ::rw::graphics::SceneViewer::Ptr SceneViewerPtr;
    typedef ::rw::graphics::WorkCellScene::Ptr WorkCellScenePtr;

    // @}
}}

#endif
