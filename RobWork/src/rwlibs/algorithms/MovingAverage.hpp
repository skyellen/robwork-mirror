/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWLIBS_ALGORITHMS_MOVINGAVERAGE_HPP_
#define RWLIBS_ALGORITHMS_MOVINGAVERAGE_HPP_

#include <vector>

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
     */
    MovingAverage(std::size_t N);

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
