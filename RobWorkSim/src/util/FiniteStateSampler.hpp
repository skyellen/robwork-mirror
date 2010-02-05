/*
 * FiniteStateSampler.hpp
 *
 *  Created on: 23-09-2009
 *      Author: jimali
 */

#ifndef FINITESTATESAMPLER_HPP_
#define FINITESTATESAMPLER_HPP_

#include "StateSampler.hpp"
#include <vector>

class FiniteStateSampler: public StateSampler
{
public:

    /**
     * @brief
     * @param state
     * @param n
     * @return
     */
    FiniteStateSampler(const rw::kinematics::State& state, int n=1);

    /**
     * @brief
     * @param state
     * @return
     */
    FiniteStateSampler(const std::vector<rw::kinematics::State>& states, int n=1);

    virtual ~FiniteStateSampler();


    bool sample(rw::kinematics::State& state);

    bool empty() const;

    typedef enum{SINGLE_STATE, MULTIPLE_STATE} SamplerType;

private:

    SamplerType _type;
    bool _empty;
    int _n,_cidx;
    std::vector<rw::kinematics::State> _states;
};

#endif /* FINITESTATESAMPLER_HPP_ */
