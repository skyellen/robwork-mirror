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

#ifndef RW_STUDIO_PLAYER_MODULE_H
#define RW_STUDIO_PLAYER_MODULE_H

#include "StateDraw.hpp"

#include <rws/RobWorkStudio.hpp>

#include <rw/common/Ptr.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

#include <boost/shared_ptr.hpp>

#include <QTimer>
#include <QObject>

// The controls of the playback interface are forwarded to this utility.
class Player : public QObject
{
    Q_OBJECT

public:
    // We will need a player that is not parameterized by just a sequence of
    // states but by a sequence of states together with the instant in time of
    // each state. We will take care of that later.
    Player(rw::trajectory::TimedStatePath statePath,
           StateDrawPtr drawer,
           double tickInterval,
           rws::RobWorkStudio* rwstudio);

    // used for recording only
    Player(
           double tickInterval,
           rws::RobWorkStudio* rwstudio);

    // Start the playback in forward direction.
    void forward();

    // Start the playback in backward direction.
    void backward();

    // Pause or resume the playback.
    void pauseOrResume();

    // Move to start of path.
    void toStart();

    // Move to end of path.
    void toEnd();

    /**
     * @brief Take a single step.
     * @param forward [in] true to move one step forward, false to move backwards.
     */
    void step(bool forward);

    // Move to a relative position.
    void setRelativePosition(double relative);

    // Scale the speed.
    void setRelativeSpeed(double speed);

    // Something to write in an info label.
    std::string getInfoLabel() const;

    // Whether to loop the playback or not.
    void setLoopPlayback(bool loop);

    //Set up the interval between ticks
    void setTickInterval(double interval);


    /**
     * @brief Starts recording.
     */
    void startRecording();

    /**
     * @brief Sets up recording to save files as \b filename of type \b type
     */
    void setupRecording(const QString filename, const QString& type);

    /**
     * @brief Stops recording
     */
    void stopRecording();

    void setInterpolate(bool interpolate){
        _interpolate = interpolate;
    }

private slots:
    // Increment the current time by tickInterval.
    void tick();

signals:
    void relativePositionChanged(double val);

private:
    void stopTimer();
    void startTimer();
    bool timerIsRunning();
    void runTimer();

    double getEndTime() const { return _trajectory->duration(); }
    void draw();

public:
    // Where to do the drawing.
	rw::trajectory::StateTrajectory::Ptr _trajectory;
	rw::trajectory::TimedStatePath _path;
private:
    // How to do the drawing.
    StateDrawPtr _drawer;

    // The time between calls to tick().
    double _tickInterval;

    rws::RobWorkStudio* _rwstudio;

    bool _record;
    int _recNo;
    QString _recordFilename;
    QString _recordType;

    double _now; // The current time.
    int _direction; // The sign of direction of traversal. +1 or -1.
    double _velocityScale; // Scaling of the step size. A non-negative number.

    QTimer _timer; // The timer yielding the callbacks.

    // Whether to loop the playback or not.
    bool _loop;

    bool _interpolate;
public:
    bool _recordingOnly;
private:
    Player(const Player&);
    Player& operator=(const Player&);
};

typedef rw::common::Ptr<Player> PlayerPtr;

PlayerPtr makeEmptyPlayer();

PlayerPtr makePlayer(const rw::trajectory::TimedStatePath& path,
                     StateDrawPtr drawer,
                     double tickInterval,
                     rws::RobWorkStudio* rwstudio);



#endif
