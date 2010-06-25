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

#include "SwissRanger.hpp"

#include <rw/common/macros.hpp>

extern "C" {
#include <swissranger/linux/swissranger.h>
}

using namespace std;

using namespace rwhw;
using namespace rwhw::swissranger;
using namespace rw::math;
using namespace rw::sensor;

const unsigned int SwissRanger::IMG_WIDTH = 160;
const unsigned int SwissRanger::IMG_HEIGHT = 124;
const unsigned int SwissRanger::IMG_SIZE = IMG_WIDTH*IMG_HEIGHT;

SwissRanger::SwissRanger(const std::string& name, float maxrange) :
    Sensor(NULL, name, "swissranger sensor", 3),
    _nIntTime(30),
    _isOpen(false),
    _maxrange(maxrange)
{}

SwissRanger::~SwissRanger()
{
    if (_isOpen)
        swissranger_close(_srHandle);
}

bool SwissRanger::isOpen()
{
    return _isOpen;
}

void SwissRanger::loadCalibrationData(const std::string& calibFilename)
{
    std::cout<<"Load CalibraData "<<std::endl;
    _calibrationData.load(calibFilename);
    std::cout<<"Calib Data Loaded "<<std::endl;
}

void SwissRanger::openCamera()
{
    if (!_isOpen) {
        _srHandle = swissranger_open();
        if (_srHandle < 0)
            RW_THROW("Could not open SwissRanger camera.");

        _isOpen = true;

        // Set output to be distance, intensity and amplitude
        if (swissranger_send(_srHandle, (char)0x2, (char)0xC0) < 0)
            RW_THROW(
                "Could not initialize swissranger camera!!!");

        setIntegrationTime(_nIntTime);
    }
}

void SwissRanger::closeCamera()
{
    if (isOpen()) {
        if (swissranger_close(_srHandle) < 0)
            RW_THROW("Could not close swissranger Camera!!!");
        _isOpen = false;
    }
}

void SwissRanger::acquireRaw(unsigned short* data)
{
    if (!isOpen()) {
        RW_WARN("Could not Acquire image, Open Camera first.");
        return;
    }

    if (swissranger_acquire(_srHandle, data, IMG_SIZE*3*sizeof(unsigned short))<0)
        RW_THROW("Could not Acquire Data from Camera");
}

void SwissRanger::acquire(float* points,
                          float* accuracies,
                          float* intensities,
                          unsigned short n)
{
    if (!isOpen()) {
        RW_WARN("Could not Acquire image, Open Camera first.");
        return;
    }

    Vector3D<float> pos;
    float dist, alpha, beta;

    //Reset content of distance, intensity and amplitude buffers
    memset(_distanceBuffer, 0, sizeof(unsigned long) * IMG_SIZE);
    memset(_intensityBuffer, 0, sizeof(unsigned long) * IMG_SIZE);
    memset(_amplitudeBuffer, 0, sizeof(unsigned long) * IMG_SIZE);

    for (unsigned short imgIndex = 0; imgIndex<n; imgIndex++) {
        acquireRaw(_image);
        for (unsigned short i = 0; i<IMG_SIZE; i++) {
            _distanceBuffer[i] += _image[i];
            _intensityBuffer[i] += _image[i+IMG_SIZE];
            _amplitudeBuffer[i] += _image[i+2*IMG_SIZE];
        }
    }

    size_t index = 0;
    unsigned long avgI = 0;
    double avgacc = 0;
    int cnt = 0;
    const float L = 7.5f;
    float accuracy, z;
    unsigned int d;
    for (unsigned short i = 0; i<IMG_SIZE; i++) {

        accuracy = L / sqrt(8.0f) * sqrt((float)_intensityBuffer[i]/n) / (2 * _amplitudeBuffer[i] / n);

        avgacc += accuracy;
        d = _distanceBuffer[i];
        dist = _calibrationData.getDistance(d / n, i);

        alpha = _calibrationData.getAlpha(i);
        beta = _calibrationData.getBeta(i);

        z = sqrt(dist*dist/(1+alpha*alpha+beta*beta));
        points[index++] = (beta*z);
        points[index++] = -(alpha*z);
        points[index++] = z;

        accuracies[i] = accuracy;
        intensities[i] = (((_intensityBuffer[i]/n)>>2))/16384.0f;

        if (dist<_maxrange) {
            avgI += _intensityBuffer[i]/n;
            cnt++;
        }

    }

    std::cout<<"Average Accuracy = "<<avgacc/IMG_SIZE<<std::endl;
    std::cout<<"Pixels OK = "<<cnt<<std::endl;
    avgI /= cnt;

    // According to "3D time-of-flight cameras for mobile robotics" it should be
    // between 15000 and 18000
    const unsigned long Ia = 14000;

    // Gain constant for adjusting integration time
    const double Vp = 1.0/1000;

    const double diff = (double)avgI-(double)Ia;
    std::cout<<"Avg T = "<<avgI<<std::endl;
    std::cout<<"dI = "<<Vp*diff<<std::endl;
    std::cout<<"Integration Time Before = "<<_nIntTime<<std::endl;
    setIntegrationTime((unsigned char)(_nIntTime - Vp * diff));
    std::cout<<"Integration Time After = "<<_nIntTime<<std::endl;
}

void SwissRanger::setIntegrationTime(unsigned char intTime)
{
    _nIntTime = intTime;
    if (isOpen()) {
        if (swissranger_send(_srHandle, 0x9, intTime)<0)
            RW_THROW(
                "Could not send integration time to camera");
    }
}

unsigned short SwissRanger::getIntegrationTime() const
{
    return _nIntTime;
}
