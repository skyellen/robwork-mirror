#ifndef RWSIM_SIMULATION_BTDEBUGRENDER_HPP_
#define RWSIM_SIMULATION_BTDEBUGRENDER_HPP_

#include <rwsim/drawable/SimulatorDebugRender.hpp>

#include <OpenGl/GLDebugDrawer.h>



namespace rwsim {
namespace simulator {
    class BtSimulator;

    class BtDebugRender: public rwsim::drawable::SimulatorDebugRender
    {
        BtSimulator *_sim;
        GLDebugDrawer *_debugDrawer;
        unsigned int _drawMask;
    public:

        BtDebugRender(BtSimulator *sim);

        virtual ~BtDebugRender();

        virtual void draw(DrawType draw, double alpha) const;

        virtual void setDrawMask(unsigned int mask);

    };
}
}
#endif /*BTDEBUGRENDER_HPP_*/
