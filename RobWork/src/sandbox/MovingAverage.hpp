/*
 * MovingAverage.hpp
 *
 *  Created on: 09-12-2008
 *      Author: jimali
 */

#ifndef MOVINGAVERAGE_HPP_
#define MOVINGAVERAGE_HPP_

#include <vector>
#include <rw/common/macros.hpp>

/**
 * @brief calculates the moving average over a continues input of samples.
 *
 * A circular buffer is maintained so that the last \b N samples can be inspected.
 * the sum of values in this buffer is also maintained such that the average can be
 * efficiently calculated.
 *
 */
class MovingAverage {
public:
    /**
     * @brief constructor creates a MovingAverage filter over a window of
     * \b N samples.
     * @param N [in] the size of the window of samples
     * @return
     */
    MovingAverage(size_t N):
        _len(N),_invLen(1.0),_cb(_len,0.0),_sum(0.0),_idx(0)
    {
        RW_ASSERT(N!=0);
        _invLen = 1.0/N;
    }

    /**
     * @brief adds a sample
     * @param sample
     */
    void addSample(double sample){
        _sum -= _cb[_idx];
        _sum += sample;
        _cb[_idx] = sample;
        _idx++;
        if(_idx == _len )
            _idx = 0;
    }

    /**
     * @brief returns the current average
     * @return
     */
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
