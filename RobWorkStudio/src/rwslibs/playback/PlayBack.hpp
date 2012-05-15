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

#ifndef RW_STUDIO_PLAYBACK_MODULE_H
#define RW_STUDIO_PLAYBACK_MODULE_H

#include <QTabWidget>
#include <QTextEdit>
#include <QSlider>
#include <QtGui>
#include <QCheckBox>

#include <list>
#include <vector>
#include <memory>

#include "Player.hpp"
#include "PlayBackSettings.hpp"

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/trajectory/Path.hpp>

namespace rws {

class PlayBack : public RobWorkStudioPlugin
{
    Q_OBJECT
#ifndef RWS_USE_STATIC_LINK_PLUGINS
    Q_INTERFACES(rws::RobWorkStudioPlugin)
#endif
public:
    PlayBack();

    virtual ~PlayBack();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    void initialize();

private:
	void stateTrajectoryChangedListener(const rw::trajectory::TimedStatePath& trajectory);
private slots:
    void openPath();
    void savePath();
    void forwardPlay();
    void backwardPlay();
    void pauseOrResumePlay();
    void toStartPlay();
    void toEndPlay();
    void reloadPlay();
    void sliderSetPosition(int val);
    void relativePositionChanged(double relative);
    void speedValueChanged(double percent);
    void loopPlaybackChanged(int state);
    void interpolateChanged(int state);

    void record(bool record);
    void showSettings();

private:
    class MyStateDraw;
    rw::common::Ptr<StateDraw> makeMyStateDraw();
    void draw(const rw::kinematics::State& state);

    void rawOpenPlayFile(const std::string& file);
    void openPlayFile(const std::string& file);
    void setInfoLabel();

private:
    rw::models::WorkCell* _workcell;
    //rwlibs::drawable::WorkCellGLDrawer* _workcellGLDrawer;

    std::string _previousOpenSaveDirectory;

    PlayerPtr _player;

    QLabel* _info;

    QSlider* _slider;
    bool _inSliderSet;
    bool _inRelativePositionChanged;

    QDoubleSpinBox* _speed;
    QCheckBox* _loop,*_interpolate;

    std::string _file; // The currently opened file.

    PlayBackSettings _settings;
};

}

#endif
