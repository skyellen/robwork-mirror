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

#include "Player.hpp"

#include <rw/trajectory/Trajectory.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>
#include <rw/common/macros.hpp>

using namespace robwork;
using namespace rw::trajectory;
using namespace rws;
namespace {
    const int RECORD_NUM_OF_DIGITS = 6;
}

Player::Player(
    TimedStatePath path,
    StateDrawPtr drawer,
    double tickInterval,
    RobWorkStudio* rwstudio)
    :
    _path(path),
    _trajectory( TrajectoryFactory::makeLinearTrajectory(path) ),
    _drawer(drawer),
    _tickInterval(tickInterval),
    _rwstudio(rwstudio),
    _record(false),
    _recNo(0),
    _now(0),
    _direction(1),
    _velocityScale(1),
    _timer(this),
    _loop(false),
    _interpolate(true)
{
    RW_ASSERT(_tickInterval > 0);
    RW_ASSERT(drawer);

    // Connect the timer:
    connect(&_timer, SIGNAL(timeout()), this, SLOT(tick()));
}


void Player::setTickInterval(double interval) {
    _tickInterval = interval;
    if (_timer.isActive()) {
        stopTimer();
        startTimer();
    }
}


void Player::setupRecording(const QString filename, const QString& type) {
    _recordFilename = filename;
    _recordType = type;
}

void Player::startRecording() {
    _record = true;
    _recNo = 0;
}

void Player::stopRecording() {
    _record = false;
}

void Player::tick()
{
    const double end = getEndTime();

    // Make sure that we do show the robot at the position at the start or end
    // of the path.
    bool outside = false;
    if (_now < 0) {
        _now = 0;
        outside = true;
    }
    else if (end < _now) {
        _now = end;
        outside = true;
    }

    // Draw the work cell.
    draw();

    if (_record && _rwstudio != NULL) {
        //Create Filename
        QString number = QString::number(_recNo++);
        while (number.length() < RECORD_NUM_OF_DIGITS)
            number.prepend("0");
        QString filename = _recordFilename + number + "." + _recordType;
        _rwstudio->saveViewGL(filename);
    }

    // If we reached the end and we are looping, then move the cursor to the
    // start or end.
    if (outside && _loop) {
        if (_direction < 0)
            _now = end;
        else
            _now = 0;
    }

    // If the range is empty or if we outside of the range and not looping:
    if (end <= 0 || (outside && !_loop)) {
        // then stop the player.
        stopTimer();
    }

    // Otherwise,
    else {
        // increment the time.
        _now +=
            _direction *
            _velocityScale *
            _tickInterval;
    }
}

// Forward, backward and pause.

void Player::forward()
{
    _direction = 1;
    runTimer();
}

void Player::backward()
{
    _direction = -1;
    runTimer();
}

void Player::pauseOrResume()
{
    if (timerIsRunning())
        stopTimer();
    else
        startTimer();
}

// Move to start and end.

void Player::toStart()
{
    _now = 0;
    stopTimer();
    draw();
}

void Player::toEnd()
{
    _now = getEndTime();
    stopTimer();
    draw();
}

// Move to a specific time.

void Player::setRelativePosition(double relative)
{
    const double time = relative * getEndTime();

    if (time < 0) toStart();
    else if (getEndTime() < time) toEnd();
    else {
        _now = time;
        // Should we stop the timer also?
        stopTimer();
        draw();
    }
}

// Change the speed.

void Player::setRelativeSpeed(double speed)
{
    _velocityScale = speed;
}

// Change the loop flag.
void Player::setLoopPlayback(bool loop)
{
    _loop = loop;
}



// Timer accessors.

void Player::runTimer()
{
    if (!timerIsRunning())
        startTimer();
}

bool Player::timerIsRunning()
{
    return _timer.isActive();
}

void Player::startTimer()
{
    const int ms = (int)(_tickInterval * 1000);
    _timer.start(ms);
}

void Player::stopTimer()
{
    _timer.stop();
}

void Player::draw()
{
    // This takes care of empty trajectories.
    if(_interpolate){
        if (0 <= _now && _now <= getEndTime()) {
            _drawer->draw(_trajectory->x(_now));
        }
    } else {
        if (0 <= _now && _now <= getEndTime()) {
            for(unsigned int i=0;i<_path.size()-1;i++){
                if( _path[i].getTime()<= _now && _now<=_path[i+1].getTime() ){
                    _drawer->draw(_path[i].getValue());
                    break;
                }
            }
        }
    }
    relativePositionChanged(_now / getEndTime());

}

std::string Player::getInfoLabel() const
{
    if (getEndTime() < 0)
        return "No trajectory loaded";
    else {
        char buf[40];
        sprintf(buf, "%.2f", getEndTime());

        return
            "Trajectory of length " +
            std::string(buf) +
            " s loaded";
    }
}

// Constructors.

PlayerPtr makeEmptyPlayer()
{
    return ownedPtr(new Player(
                               TimedStatePath(),
                               makeEmptyStateDraw(),
                               1,
                               (RobWorkStudio*)NULL));
}

PlayerPtr makePlayer(const TimedStatePath& path,
                     StateDrawPtr drawer,
                     double tickInterval,
                     RobWorkStudio* rwstudio)
{
    return ownedPtr(new Player(path, drawer, tickInterval, rwstudio));
}
