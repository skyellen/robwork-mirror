#ifndef EULERINTEGRATOR_HPP_
#define EULERINTEGRATOR_HPP_

#include "BodyIntegrator.hpp"

#include <rw/math/EAA.hpp>

#include "RWBody.hpp"

namespace rwsim {
namespace simulator {


    class EulerIntegrator : public BodyIntegrator
    {
    public:

        EulerIntegrator(RWBody *body);

    	virtual ~EulerIntegrator(){};

        void updatePosition(double h, rw::kinematics::State& state);

        void updateVelocity(double h, rw::kinematics::State& state);

    private:
        RWBody *_body;
    };

}
}

#endif /*EULERINTEGRATOR_HPP_*/
