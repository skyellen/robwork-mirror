#ifndef RWSIM_SIMULATION_BTDEBUGRENDER_HPP_
#define RWSIM_SIMULATION_BTDEBUGRENDER_HPP_

#include <rwsim/simulator/SimulatorDebugRender.hpp>

#include <OpenGl/GLDebugDrawer.h>

class BtSimulator;

namespace rwsim {
namespace simulator {


    class BtDebugRender: public SimulatorDebugRender
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
#endif /*BTDEBUGRENDER_HPP_*/
