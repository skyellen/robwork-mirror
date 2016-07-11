#ifndef RWSIM_DRAWABLE_RENDERGHOST_HPP_
#define RWSIM_DRAWABLE_RENDERGHOST_HPP_

//! @file RenderGhost.hpp

#include <list>
//#include <vector>

#include <rw/graphics/Render.hpp>
#include <rw/kinematics/State.hpp>

#include <boost/circular_buffer.hpp>

namespace rw { namespace graphics { class WorkCellScene; } }
namespace rwlibs { namespace opengl { class RenderFrame; } }

namespace rwsim {
namespace drawable {
	//! @addtogroup rwsim_drawable
	//! @{

	/**
	 * @brief This render implementation will render a set of previous
	 * configurations/states (ghosts) of a set of specific frames.
	 *
	 * A fixed size \b N circular or ring buffer of states is maintained such
	 * that the last \b N added states will be rendered.
	 */
	class RenderGhost: public rw::graphics::Render
	{
	public:

		/**
		 * @brief constructor - for rendering a single frame
		 * @param frame [in] frame that is to be rerendered
		 * @param drawer [in] the workcell drawer
		 * @param N [in] max nr of states that is to be rendered
		 */
		RenderGhost(rw::kinematics::Frame *frame,
				rw::common::Ptr<rw::graphics::WorkCellScene> drawer,
				size_t N);
	
		/**
		 * @brief constructor - for rendering multiple frames
		 * @param frames [in] all frames that are to be rendered
		 * @param drawer [in] the workcell drawer
		 * @param N [in] max nr of states that is to be rendered
		 */
		RenderGhost(std::list<rw::kinematics::Frame*> frames,
				rw::common::Ptr<rw::graphics::WorkCellScene> drawer,
				size_t N);

		/**
		 * @brief destructor
		 */
		virtual ~RenderGhost();

		/**
		 * @brief add new state that is to be rendered
		 * @param state [in] state that is to be rendered
		 */
		void addState(const rw::kinematics::State& state);

		/**
		 * @brief clear all states
		 */
		void clear();
	
		/**
		 * @brief sets the max number of states that is rendered
		 * @param size [in] max number of states to render
		 * @note be carefull setting this too high, since rendering typically
		 * is performance wise quite expensive.
		 */
		void setMaxBufferSize(size_t size);
	
        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;

	
	private:
		std::list<rw::kinematics::Frame*> _frames;
		rw::common::Ptr<rw::graphics::WorkCellScene> _drawer;
		//std::vector<rw::kinematics::State> _states;
		rwlibs::opengl::RenderFrame *_drawFrame;

		boost::circular_buffer<rw::kinematics::State> _states;
	};
	//! @}
}
}

#endif /*RenderGhost_HPP_*/
