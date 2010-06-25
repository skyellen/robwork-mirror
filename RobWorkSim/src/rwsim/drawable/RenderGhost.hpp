#ifndef RWSIM_DRAWABLE_RENDERGHOST_HPP_
#define RWSIM_DRAWABLE_RENDERGHOST_HPP_

//! @file RenderChost.hpp

#include <list>
#include <vector>

#include <rw/kinematics/State.hpp>

#include <rwlibs/drawable/Render.hpp>
#include <rwlibs/drawable/RenderFrame.hpp>
#include <rwlibs/drawable/WorkCellGLDrawer.hpp>

#include <boost/circular_buffer.hpp>

namespace rwsim {
namespace drawable {
	//! @addtogroup drawable @{

	/**
	 * @brief This render implementation will render a set of previous
	 * configurations/states (ghosts) of a set of specific frames.
	 *
	 * A fixed size \b N circular or ring buffer of states is maintained such
	 * that the last \b N added states will be rendered.
	 */
	class RenderGhost: public rwlibs::drawable::Render
	{
	public:

		/**
		 * @brief constructor
		 * @param frame [in] frame that is to be rerendered
		 * @param drawer [in] the workcell drawer
		 * @param N [in] max nr of states that is to be rendered
		 */
		RenderGhost(rw::kinematics::Frame *frame,
					  rwlibs::drawable::WorkCellGLDrawer *drawer,
					  size_t N);
	
		/**
		 * @brief constructor
		 * @param frame [in] frame that is to be rerendered
		 * @param drawer [in] the workcell drawer
		 * @param N [in] max nr of states that is to be rendered
		 */
		RenderGhost(std::list<rw::kinematics::Frame*> frames,
					  rwlibs::drawable::WorkCellGLDrawer *drawer,
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
	
		//! @copydoc Render::draw
		virtual void draw(DrawType type, double alpha) const;
	
	private:
		std::list<rw::kinematics::Frame*> _frames;
		rwlibs::drawable::WorkCellGLDrawer * _drawer;
		//std::vector<rw::kinematics::State> _states;
		rwlibs::drawable::RenderFrame *_drawFrame;

		boost::circular_buffer<rw::kinematics::State> _states;
	};
	//! @}
}
}

#endif /*RenderGhost_HPP_*/
