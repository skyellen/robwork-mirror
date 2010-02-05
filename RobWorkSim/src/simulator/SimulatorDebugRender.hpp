#ifndef SIMULATORDEBUGRENDER_HPP_
#define SIMULATORDEBUGRENDER_HPP_

#include <rwlibs/drawable/Render.hpp>

namespace drawable {

	class SimulatorDebugRender : public rwlibs::drawable::Render {
	public:

		//virtual ~SimulatorDebugRender(){};

		static const unsigned int DRAW_NOTHING = 0;
		static const unsigned int DRAW_CONTACT_NORMAL = 2;
		static const unsigned int DRAW_FRICTION_CONE = 4;

		//virtual void draw(DrawType type, double alpha) = 0;

		virtual void setDrawMask(unsigned int mask) = 0;

	protected:
	    SimulatorDebugRender(){};

	};

} // dynamics

#endif /*RENDERSIMDEBUG_HPP_*/
