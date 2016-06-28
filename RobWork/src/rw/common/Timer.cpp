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


#include "Timer.hpp"
#include "TimerUtil.hpp"

#include <sstream>

using namespace rw::common;

Timer::Timer()
{
    reset();
}

Timer::Timer(long timems):
        _totalTime(timems),
        _isPaused(true)
{

}

Timer::Timer(int hh, int mm, int ss, int ms):
        _totalTime( ((hh*60+mm)*60+ss)*1000 + ms ),
        _isPaused(true)
{

}

Timer::~Timer()
{

}

bool Timer::isPaused() {
	return _isPaused;
}

void Timer::reset()
{
    _relativeTime = (long)TimerUtil::currentTimeMs();
    _totalTime = 0;
    _isPaused = false;
}

void Timer::resetAndPause() {    
    _totalTime = 0;
    _isPaused = true;
}

void Timer::resetAndResume() {
    _totalTime = 0;
    _relativeTime = (long)TimerUtil::currentTimeMs();
    _isPaused = false;
}

void Timer::pause()
{
    if (!_isPaused) {
        const long now = (long)TimerUtil::currentTimeMs();
        _totalTime += now - _relativeTime;
        _relativeTime = now;
        _isPaused = true;
    }
}

void Timer::resume()
{
    if (_isPaused) {
        _relativeTime = (long)TimerUtil::currentTimeMs();
        _isPaused = false;
    }
}



double Timer::getTime() const
{
	return getTimeMs()*0.001;
}

long Timer::getTimeSec() const{
	return getTimeMs()/1000;
}

long Timer::getTimeMs() const
{
    if (_isPaused)
        return _totalTime; // convert to sec
    else
        return _totalTime + (long)TimerUtil::currentTimeMs() - _relativeTime;
}


std::string Timer::toString(const std::string& format){
    char line[64];
    long timems = _totalTime;
    if(!_isPaused)
        timems += (long)TimerUtil::currentTimeMs() - _relativeTime;
    int dd = timems/(1000*60*60*24);
    int hh = timems/(1000*60*60)-dd*24;
    int mm = timems/(1000*60)-dd*24*60-hh*60;
    int ss = timems/1000-dd*24*60*60-hh*60*60-mm*60;
    int ms = timems-(((dd*24+hh)*60+mm)*60+ss)*1000;

    std::stringstream sstr(format);
    std::stringstream output;
    while(!sstr.eof()){
         sstr.getline(line, 64, ':');
        if(line[0]=='h'){
            if(line[1]=='h' && hh<10)
                output << "0";
            output << hh << ":";
        } else if(line[0]=='m'){
            if(line[1]=='m' && mm<10)
                output << "0";
            output << mm << ":";
        } else if(line[0]=='s' ){
            if(line[1]=='s' && ss<10)
                output << "0";
            output << ss << ":";
        } else if(line[0]=='z' ){
            if(line[1]=='z' && line[2]=='z' && ms<100 ){
                if(ms<10)
                    output << "00";
                else
                    output << "0";
            }
            output << ms << ":";
        } else {

        }
    }
    std::string outstr = output.str();
    return  outstr.substr(0,outstr.size()-1);
}

