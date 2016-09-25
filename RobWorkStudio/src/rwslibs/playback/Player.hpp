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

#include <rw/common/Ptr.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/trajectory/Trajectory.hpp>

#include <QTimer>
#include <QObject>

class StateDraw;

namespace rws { class RobWorkStudio; }

// The controls of the playback interface are forwarded to this utility.
class Player : public QObject
{
    Q_OBJECT

public:
	//! @brief Smart pointer type for Player.
	typedef rw::common::Ptr<Player> Ptr;

	/**
	 * @brief Construct a new player.
	 * @param statePath [in] the path of timed states.
	 * @param drawer [in] the drawer to invoke for displaying a new state in the path.
	 * @param tickInterval [in] the rendering rate.
	 * @param rwstudio [in] the RobWorkStudio instance (used for saving to file during recordings).
	 */
    Player(rw::trajectory::TimedStatePath statePath,
           rw::common::Ptr<StateDraw> drawer,
           double tickInterval,
           rws::RobWorkStudio* rwstudio);

    /**
     * @brief Construct a new player that can be used only for recording.
     *
     * Notice that the state in RobWorkStudio must be updated through other means, for instance by the user.
     *
	 * @param tickInterval [in] the recording rate.
	 * @param rwstudio [in] the RobWorkStudio instance (used for saving to file during recordings).
     */
    Player(
           double tickInterval,
           rws::RobWorkStudio* rwstudio);

    //! @brief Start the playback in forward direction.
    void forward();

    //! @brief Start the playback in backward direction.
    void backward();

    //! @brief Pause or resume the playback.
    void pauseOrResume();

    //! @brief Move to start of path.
    void toStart();

    //! @brief Move to end of path.
    void toEnd();

    /**
     * @brief Take a single step.
     * @param forward [in] true to move one step forward, false to move backwards.
     */
    void step(bool forward);

    /**
     * @brief Move to a relative position.
     * @param relative [in] a value between 0 and 1.
     */
    void setRelativePosition(double relative);

    /**
     * @brief Scale the speed.
     * @param speed [in] the velocity scale.
     */
    void setRelativeSpeed(double speed);

    /**
     * @brief Get the current status of the player.
     * @return a status string.
     */
    std::string getInfoLabel() const;

    /**
     * @brief Whether to loop the playback or not.
     * @param loop [in] true will loop the playback.
     */
    void setLoopPlayback(bool loop);

    /**
     * @brief Set up the interval between ticks
     * @param interval [in] the interval.
     */
    void setTickInterval(double interval);

    //! @brief Starts recording.
    void startRecording();

    /**
     * @brief Sets up recording to save files as \b filename of type \b type.
     * @param filename [in] the filename to save to.
     * @param type [in] the filetype.
     */
    void setupRecording(const QString filename, const QString& type);

    //! @brief Stops recording
    void stopRecording();

    /**
     * @brief Enable/disable interpolation for a more smooth playback.
     * @param interpolate [in] true to do interpolation.
     */
    void setInterpolate(bool interpolate){
        _interpolate = interpolate;
    }

    /**
     * @brief Construct an empty player.
     * @return empty player.
     */
    static Player::Ptr makeEmptyPlayer();

    //! @copydoc Player
    static Player::Ptr makePlayer(const rw::trajectory::TimedStatePath& path,
    					rw::common::Ptr<StateDraw> drawer,
                         double tickInterval,
                         rws::RobWorkStudio* rwstudio);

private slots:
    // Increment the current time by tickInterval.
    void tick();

signals:
	/**
	 * @brief Emitted during playback.
	 * @param val [in] the current playback position as a value between 0 and 1.
	 */
    void relativePositionChanged(double val);

private:
    void stopTimer();
    void startTimer();
    bool timerIsRunning();
    void runTimer();

    double getEndTime() const { return _trajectory->duration(); }
    void draw();

public:
    //! @brief The interpolated trajectory.
	rw::trajectory::StateTrajectory::Ptr _trajectory;
    //! @brief The original trajectory.
	rw::trajectory::TimedStatePath _path;
private:
    // How to do the drawing.
	rw::common::Ptr<StateDraw> _drawer;

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
    //! @brief Indicates whether a trajectory is loaded, or the Player is in recording-only mode.
    bool _recordingOnly;
private:
    Player(const Player&);
    Player& operator=(const Player&);
};
#endif
