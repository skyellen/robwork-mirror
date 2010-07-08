#ifndef RWSIM_SIMULATOR_RWDEBUGRENDER_HPP_
#define RWSIM_SIMULATOR_RWDEBUGRENDER_HPP_

#include <rwsim/dynamics/DynamicWorkcell.hpp>
#include <rwsim/drawable/SimulatorDebugRender.hpp>

namespace rwsim {
namespace simulator {


	class RWDebugRender : public rwsim::drawable::SimulatorDebugRender {
	public:
		RWDebugRender(dynamics::DynamicWorkcell &dwc):_dwc(dwc){};

		virtual ~RWDebugRender(){};

		// methods inherited from RenderSimDebug
		void draw(DrawType type, double alpha) const;

		void setDrawMask(unsigned int mask);

	private:
		dynamics::DynamicWorkcell &_dwc;
		unsigned int _mask;
	};

}
}

#endif /*RWDEBUGRENDER_HPP_*/
