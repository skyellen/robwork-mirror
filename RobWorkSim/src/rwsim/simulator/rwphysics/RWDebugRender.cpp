#include "RWDebugRender.hpp"

using namespace rwsim::drawable;
using namespace rwsim::simulator;


namespace {

	void drawContactNormals(){
		// run through all contacts
			// for each contact draw the contact normal
	}
}

// methods inherited from RenderSimDebug
void RWDebugRender::draw(DrawType type, double alpha) const {

	if( _mask & SimulatorDebugRender::DRAW_NOTHING )
		return;

	if( _mask & SimulatorDebugRender::DRAW_CONTACT_NORMAL ){
		// draw contact normals
		drawContactNormals();
	}

	if( _mask & SimulatorDebugRender::DRAW_FRICTION_CONE){
		// draw friction cones
	}

}

void RWDebugRender::setDrawMask(unsigned int mask){
	_mask = mask;
}
