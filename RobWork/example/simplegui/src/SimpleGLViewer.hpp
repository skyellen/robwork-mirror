#ifndef SIMPLEGLVIEWER_H_
#define SIMPLEGLVIEWER_H_

#include <rw/graphics/SceneViewer.hpp>

namespace rw { namespace models { class WorkCell; } }
namespace rwlibs { namespace opengl { class SceneOpenGL; } }

class EventListener;
class Menu;

#include <map>

/**
 * @brief simple viewer for rendering robwork workcell
 */
class SimpleGLViewer: public rw::graphics::SceneViewer {
    public:
    	//! @brief smart pointer type to this class
    	typedef rw::common::Ptr<SimpleGLViewer> Ptr;

        //! @brief Constructor.
		SimpleGLViewer();

    	//! @brief Destructor.
        virtual ~SimpleGLViewer();

        //! @copydoc SceneViewer::getScene
        virtual rw::graphics::SceneGraph::Ptr getScene();

        //! @copydoc SceneViewer::getLogo
        virtual const std::string& getLogo() const;

        //! @copydoc SceneViewer::setLogo
        virtual void setLogo(const std::string& string);

        //! @copydoc SceneViewer::getPropertyMap
        virtual rw::common::PropertyMap& getPropertyMap();

        //! @copydoc SceneViewer::updateView
        virtual void updateView();

        //! @copydoc SceneViewer::updateState
        virtual void updateState(const rw::kinematics::State& state);

        //! @copydoc SceneViewer::setWorldNode
        virtual void setWorldNode(rw::graphics::GroupNode::Ptr wnode);

        //! @copydoc SceneViewer::getWorldNode
        virtual rw::graphics::GroupNode::Ptr getWorldNode();

        //! @copydoc SceneViewer::saveBufferToFile
        virtual void saveBufferToFile(const std::string& stdfilename,
                                      const int fillR, const int fillG, const int fillB);

        //! @copydoc SceneViewer::getViewCamera
        virtual rw::graphics::SceneCamera::Ptr getViewCamera();

        //! @copydoc SceneViewer::getViewCenter
        virtual rw::math::Vector3D<> getViewCenter();

        //! @copydoc SceneViewer::pickDrawable(int,int)
        virtual rw::graphics::DrawableNode::Ptr pickDrawable(int x, int y);

        //! @copydoc SceneViewer::pickDrawable(SceneGraph::RenderInfo&, int, int)
        virtual rw::graphics::DrawableNode::Ptr pickDrawable(rw::graphics::SceneGraph::RenderInfo& info, int x, int y);

        //! @copydoc SceneViewer::createView
        virtual View::Ptr createView(const std::string& name, bool enableBackground=false);

        //! @copydoc SceneViewer::getMainView
        virtual View::Ptr getMainView();

        //! @copydoc SceneViewer::destroyView
        virtual void destroyView(View::Ptr view);

        //! @copydoc SceneViewer::selectView
        virtual void selectView(View::Ptr view);

        //! @copydoc SceneViewer::getCurrentView
        virtual View::Ptr getCurrentView();

        //! @copydoc SceneViewer::getViews
        virtual std::vector<View::Ptr> getViews();

        //! @copydoc SceneViewer::renderView
        virtual void renderView(View::Ptr);

        //! @copydoc SceneViewer::zoom
        virtual void zoom(double amount);

        //! @copydoc SceneViewer::autoZoom
        virtual void autoZoom();

        void init(int argc, char** argv);

        void addMenu(Menu *menu);

        void setKeyListener(EventListener *listener);

        void setWorkcell(rw::common::Ptr<rw::models::WorkCell> workcell);

        const rw::kinematics::State& getState();

        void resize(int width, int height);

        void setPosition(int x, int y);

        bool start();

        bool stop();

    private:
        void addSubMenus(Menu *menu);
        void initGlut(int,int,int,int);
        void initLights();
        void initMenu();

    public:
        struct InternalData;
        InternalData* const _data;

    private:
        rw::common::Ptr<rwlibs::opengl::SceneOpenGL> _scene;
        rw::graphics::GroupNode::Ptr _worldNode;
        rw::common::Ptr<rw::kinematics::State> _state;

        rw::common::Ptr<rw::models::WorkCell> _wc;

        std::map<int,Menu*> _menuMap;
};


#endif /*SIMPLEGLVIEWER_H_*/
