#ifndef RW_GRAPHICS_WORKCELLSCENE_HPP_
#define RW_GRAPHICS_WORKCELLSCENE_HPP_

#include "SceneGraph.hpp"

#include <vector>
#include <map>
#include <boost/thread/mutex.hpp>

#include <rw/kinematics/FKTable.hpp>
#include <rw/sensor/Image.hpp>
#include <rw/sensor/Scan2D.hpp>
#include <rw/sensor/Image25D.hpp>

#include "SceneGraph.hpp"

namespace rw { namespace models { class WorkCell; }}
namespace rw { namespace kinematics { class Frame; class State; }}

namespace rw {
namespace graphics {

    /**
     * @brief  class for wrapping the SceneGraph interface such that it extends the scene-graph
     * functionality to work on frames and workcells.
     *
     * The scene graph is composed of nodes which are related to Frames. Each frame
     * can have several Drawables attached which can be considered as leafs. Beside keeping track
     * of the mapping from frames to scenenodes, this class also adds settings such as highlighted,
     * adding of frameaxis(per Frame), visibility and so on.
     */
    class WorkCellScene {
    public:
        //! @brief smart pointer to this class
        typedef rw::common::Ptr<WorkCellScene> Ptr;

        /**
         * @brief constructor - creates a workcell with a current SceneGraph
         * @param scene [in] scene graph to use.
         */
        WorkCellScene(SceneGraph::Ptr scene);

        //! @brief destructor
        virtual ~WorkCellScene();

        /**
         * @brief renders the complete scene using the settings in \b info.
         *
         * This basically forwards the call to SceneGraph::draw
         *
         * @param info [in] render settings
         */
        void draw(SceneGraph::RenderInfo& info);

        /**
         * @brief set the workcell to render
         * @param wc [in] the workcell that is to be rendered
         */
        void setWorkCell(rw::models::WorkCell::Ptr wc);


        //! @brief get the workcell that is currently being rendered.
        rw::models::WorkCell::Ptr getWorkCell();

        /**
         * @brief state changes are updated by calling this method. That includes
         * transformations between frames, joints and stuff.
         * @param state [in] the new state that is to be rendered
         */
        void setState(const rw::kinematics::State& state);

        /**
         * @brief gets the node of the scene graph that maps to the world frame
         * of the workcell.
         * @return scene node
         */
        rw::graphics::GroupNode::Ptr getWorldNode();

        /**
         * @brief updates the state of the scenegraph to that of \b state
         * @param state
         */
        void updateSceneGraph(rw::kinematics::State& state);

        /**
         * @brief any cached drawables or scene nodes are deleted
         */
        void clearCache();

        // the frame interface
        /**
         * @brief sets the visibility of a frame and its drawables.
         * @param f [in] the frame.
         * @param visible [in] true if frame should be visible, false otherwise
         */
        void setVisible(rw::kinematics::Frame* f, bool visible);

        /**
         * @brief test if a frame is visible or not
         * @param f [in] the frame
         * @return true if frame is visible, false if not
         */
        bool isVisible(rw::kinematics::Frame* f);

        /**
         * @brief sets a frame to be highlighted or not.
         * @param f [in] the frame.
         * @param highlighted [in] true if frame should be highlighted, false otherwise
         */
        void setHighlighted( rw::kinematics::Frame* f, bool highlighted);

        /**
         * @brief test if a frame is highlighted or not
         * @param f [in] the frame
         * @return true if frame is highlighted, false if not
         */
        bool isHighlighted( rw::kinematics::Frame* f);

        /**
         * @brief enables the drawing of the frame-axis of a frame.
         * @param f [in] the frame
         * @param visible [in] true if frame axis should be drawn, false otherwise
         */
        void setFrameAxisVisible( rw::kinematics::Frame* f, bool visible);

        /**
         * @brief test if frame-axis is visible
         * @param f [in] the frame
         * @return true if frame axis of frame is set to be drawn, false otherwise
         */
        bool isFrameAxisVisible( rw::kinematics::Frame* f);

		/**
		 * @brief Sets the draw type for frame and drawables associated to a frame
		 * @param f [in] Frame to which the drawables are associated
		 * @param type [in] New draw type
		 */
        void setDrawType( rw::kinematics::Frame* f, DrawableNode::DrawType type);

		/**
		 * @brief Returns the draw type associated to a frame
		 * @param f [in] Frame for which to return the draw type
		 */
        DrawableNode::DrawType getDrawType( rw::kinematics::Frame* f);


        void setDrawMask(rw::kinematics::Frame* f, unsigned int mask);
        unsigned int getDrawMask(rw::kinematics::Frame* f);

        void setTransparency(rw::kinematics::Frame* f, double alpha);

        // interface for adding drawables
        DrawableGeometryNode::Ptr addLines(const std::string& name, const std::vector<rw::geometry::Line >& lines, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);
        DrawableGeometryNode::Ptr addGeometry(const std::string& name, rw::geometry::Geometry::Ptr geom, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);
        DrawableNode::Ptr addFrameAxis(const std::string& name, double size, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);
        DrawableNode::Ptr addModel3D(const std::string& name, Model3D::Ptr model, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);
        DrawableNode::Ptr addImage(const std::string& name, const rw::sensor::Image& img, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);
        DrawableNode::Ptr addScan(const std::string& name, const rw::sensor::Scan2D& scan, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);
        DrawableNode::Ptr addScan(const std::string& name, const rw::sensor::Image25D& scan, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);
        DrawableNode::Ptr addRender(const std::string& name, rw::graphics::Render::Ptr render, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);

        DrawableNode::Ptr addDrawable(const std::string& filename, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);
        void addDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame*);

        std::vector<DrawableNode::Ptr> getDrawables();
        std::vector<DrawableNode::Ptr> getDrawables(rw::kinematics::Frame*);
        std::vector<DrawableNode::Ptr> getDrawablesRec(rw::kinematics::Frame*, rw::kinematics::State& state);

        DrawableNode::Ptr findDrawable(const std::string& name);
        DrawableNode::Ptr findDrawable(const std::string& name, rw::kinematics::Frame* frame);
        std::vector<DrawableNode::Ptr> findDrawables(const std::string& name);

        bool removeDrawables(rw::kinematics::Frame*);
        bool removeDrawables(const std::string& name);
        bool removeDrawable(DrawableNode::Ptr drawable);
        bool removeDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame*);
        bool removeDrawable(const std::string& name);
        bool removeDrawable(const std::string& name, rw::kinematics::Frame*);

        rw::kinematics::Frame* getFrame(DrawableNode::Ptr d);

        void workCellChangedListener(int);
    private:
        SceneGraph::Ptr _scene;
        rw::models::WorkCell::Ptr _wc;
        rw::kinematics::FKTable _fk;

        typedef std::map<rw::kinematics::Frame*, GroupNode::Ptr> FrameNodeMap;
        typedef std::map<GroupNode::Ptr, rw::kinematics::Frame*> NodeFrameMap;

        //! the mapping from frame to group nodes 1:1
        FrameNodeMap _frameNodeMap;
        //! the mapping from group nodes to frames 1:1
        NodeFrameMap _nodeFrameMap;

        /**
         * @brief struct for keeping track of the visual state of each frame
         */
        struct FrameVisualState {
            FrameVisualState():visible(true),highlighted(false),alpha(1.0),frameAxisVisible(false),dtype(DrawableNode::SOLID){}
            bool visible;
            bool highlighted;
            double alpha;
            bool frameAxisVisible;
            DrawableNode::DrawType dtype;
            unsigned int dmask;
        };

        //! mapping from frame to visualization state, 1:1
        std::map<rw::kinematics::Frame*, FrameVisualState> _frameStateMap;
        //! mapping from frame to all its drawables, 1:many
        std::map<rw::kinematics::Frame*, std::vector<DrawableNode::Ptr> > _frameDrawableMap;

        //! the drawable used to draw the frame axis
        DrawableNode::Ptr _frameAxis;

        GroupNode::Ptr _worldNode;
    };
}
}

#endif /* RWSCENEGRAPH_HPP_ */
