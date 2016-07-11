#ifndef RWSIM_SIMULATOR_RWDEBUGRENDER_HPP_
#define RWSIM_SIMULATOR_RWDEBUGRENDER_HPP_

#include <rwsim/drawable/SimulatorDebugRender.hpp>

namespace rwsim { namespace dynamics { class DynamicWorkCell; } }

namespace rwsim {
namespace simulator {


	class RWDebugRender : public rwsim::drawable::SimulatorDebugRender {
	public:
		RWDebugRender(dynamics::DynamicWorkCell &dwc):_dwc(dwc){};

		virtual ~RWDebugRender(){};

		// methods inherited from RenderSimDebug
		void draw(const rw::graphics::DrawableNode::RenderInfo& info, DrawType type, double alpha) const;

		void setDrawMask(unsigned int mask);

	private:
		dynamics::DynamicWorkCell &_dwc;
		unsigned int _mask;
	};

}
}

#endif /*RWDEBUGRENDER_HPP_*/
