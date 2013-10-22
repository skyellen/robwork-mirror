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

#ifndef RW_STUDIO_LOG_MODULE_H
#define RW_STUDIO_LOG_MODULE_H

#include <QTextEdit>

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/TreeState.hpp>
#include <rw/kinematics/StateSetup.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/common/Message.hpp>

class WriterWrapper;

namespace rws {
/**
 * @brief This plugin registers a number of LogWriters at the Log component
 * such that any output written to info, warning or error logs will be routed
 * to the window of this plugin.
 */
class ShowLog : public RobWorkStudioPlugin
{
    Q_OBJECT
#ifndef RWS_USE_STATIC_LINK_PLUGINS
    Q_INTERFACES(rws::RobWorkStudioPlugin)
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
#endif
public:
    /**
     * @brief constructor
     */
	ShowLog();

    /**
     * @brief destructor
     */
    ~ShowLog();

	/**
	 * @copydoc RobWorkStudioPlugin::open
	 */
    void open(rw::models::WorkCell* workcell);

	/**
	 * @copydoc RobWorkStudioPlugin::close
	 */
    void close();

	/**
	 * @copydoc RobWorkStudioPlugin::frameSelectedListener
	 */
    void frameSelectedListener(rw::kinematics::Frame* frame);

	/**
	 * @copydoc RobWorkStudioPlugin::listener
	 */
    void initialize();

    /**
     * @brief listener for messegers from other plugins
     * @param plugin
     * @param id
     * @param msg
     */
    void receiveMessage(
        const std::string& plugin,
        const std::string& id,
        const rw::common::Message& msg);

    void write(const std::string& str, const QColor& color);

    void flush(){ _editor->clear(); };

    bool event(QEvent *event);

private:
    QIcon getIcon();

private:
    QTextEdit* _editor;
    QTextCursor *_endCursor;
    std::vector<rw::common::Ptr<WriterWrapper> > _writers;

    //rw::kinematics::State _state;
};

}

#endif
