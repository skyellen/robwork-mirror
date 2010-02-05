/*
 * MovingAverage.hpp
 *
 *  Created on: 09-12-2008
 *      Author: jimali
 */

#ifndef MOVINGAVERAGE_HPP_
#define MOVINGAVERAGE_HPP_

#include <vector>

/**
 * @brief
 */
class MovingAverage {
public:
    MovingAverage(int len):
        _len(len),_invLen(1.0/len),_cb(_len,0.0),_sum(0.0),_idx(0)
    {}

    void addSample(double sample){
        _sum -= _cb[_idx];
        _sum += sample;
        _cb[_idx] = sample;
        _idx++;
        if(_idx == _len )
            _idx = 0;
    }

    double getAverage(){
        return _sum*_invLen;
    }

private:
    const int _len;
    double _invLen;
    std::vector<double> _cb;
    double _sum;
    int _idx;

};


#endif /* MOVINGAVERAGE_HPP_ */
