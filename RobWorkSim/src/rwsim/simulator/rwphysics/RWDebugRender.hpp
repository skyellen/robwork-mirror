#ifndef RWDEBUGRENDER_HPP_
#define RWDEBUGRENDER_HPP_

#include <dynamics/DynamicWorkcell.hpp>
#include <simulator/SimulatorDebugRender.hpp>

namespace rwsim {
namespace simulator {


	class RWDebugRender : public SimulatorDebugRender {
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
