/*
 * ODEDebugRender.hpp
 *
 *  Created on: 07-07-2008
 *      Author: jimali
 */

#ifndef ODEDEBUGRENDER_HPP_
#define ODEDEBUGRENDER_HPP_

#include <simulator/SimulatorDebugRender.hpp>

class ODESimulator;

namespace drawable {

    class ODEDebugRender: public SimulatorDebugRender
    {
        ODESimulator *_sim;
        unsigned int _drawMask;
    public:

        ODEDebugRender(ODESimulator * sim):
            _sim(sim)
        {
        }

        virtual ~ODEDebugRender(){}

        virtual void draw(DrawType draw, double alpha) const;

        virtual void setDrawMask(unsigned int mask){
            _drawMask = mask;
        }
    };

}


#endif /* ODEDEBUGRENDER_HPP_ */
