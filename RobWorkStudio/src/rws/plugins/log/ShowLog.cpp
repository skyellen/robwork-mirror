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

#include "ShowLog.hpp"

#include <rws/RobWorkStudio.hpp>
#include <sstream>
#include <rw/common/Log.hpp>

using namespace robwork;
using namespace rwlibs::drawable;
using namespace rw::common;
using namespace rws;
namespace
{
}

//----------------------------------------------------------------------
// Standard plugin methods

class WriterWrapper: public LogWriter {
public:
	WriterWrapper(ShowLog* slog, QColor color, Log::LogLevel id):
		_slog(slog),_color(color),_id(id)
	{

	}

	virtual ~WriterWrapper(){}

    virtual void flush(){
    	_slog->flush();
    }

    /**
     * @brief Writes \b str to the log
     * @param str [in] message to write
     */
    virtual void write(const std::string& str){
    	/*    	std::stringstream buf;


        if(_isNewLine){
            buf	<< "["<<_id<<"] : ";
        }
        size_t lpos = 0;
        size_t pos = str.find("\n");
        while(pos!=std::string::npos){
        	buf << str.substr(pos,pos-lpos) << "\n    ";
        	lpos = pos;
        	pos = str.find("\n", pos+1);
        }
        buf << str.substr(lpos) << "\n";

        _slog->write(buf.str(),_color);
        */
    	_slog->write(str,_color);
        _isNewLine = false;
    }

    virtual void writeln(const std::string& str){
    	_slog->write(str,_color);
    	_isNewLine = true;
    }
private:
	ShowLog *_slog;
	QColor _color;
	Log::LogLevel _id;
	bool _isNewLine;
};

QIcon ShowLog::getIcon() {
  //  Q_INIT_RESOURCE(resources);
    return QIcon(":/log.png");
}

ShowLog::ShowLog():
    RobWorkStudioPlugin("Log", getIcon() )
{
	this->setWindowFlags(Qt::CustomizeWindowHint);

    _editor = new QTextEdit();
    _editor->setReadOnly(true);
    _editor->setCurrentFont( QFont("Courier New", 10) );

    _endCursor = new QTextCursor( );
    *_endCursor = _editor->textCursor();

    setWidget(_editor);  // Sets the widget on the QDockWidget

    _writers.push_back( new WriterWrapper(this, Qt::black, Log::Info) );
    _writers.push_back( new WriterWrapper(this, Qt::darkYellow, Log::Warning) );
    _writers.push_back( new WriterWrapper(this, Qt::darkRed, Log::Error) );


    log().setWriter(Log::Info, _writers[0]);
    log().setWriter(Log::Warning, _writers[1]);
    log().setWriter(Log::Error, _writers[2]);
}

ShowLog::~ShowLog()
{

}

void ShowLog::open(WorkCell* workcell)
{
	if( workcell==NULL )
		return;

	log().info() << "WorkCell opened: " << StringUtil::quote(workcell->getName()) << std::endl;
}

void ShowLog::close()
{
	log().info() << "WorkCell closed!" << std::endl;
}

void ShowLog::receiveMessage(
    const std::string& plugin,
    const std::string& id,
    const robwork::Message& msg)
{
	RW_WARN("Deprecated function, use log().info() << \"your string\" instead");
    /*std::stringstream buf;
    buf
        << id
        << " ["
        << plugin
        << "] "
        << "("
        << msg.getFile()
        << ":"
        << msg.getLine()
        << "):\n"
        << msg.getText()
        << "\n";
    _editor->append(buf.str().c_str());
    */
}

void ShowLog::write(const std::string& str, const QColor& color){
    _editor->setTextCursor(*_endCursor);
    _editor->setTextColor( color );

	//_endCursor->insertText(str.c_str());
	_editor->insertPlainText( str.c_str() );
	*_endCursor = _editor->textCursor();
	//_editor->insertPlainText(  );
}

void ShowLog::frameSelectedListener(Frame* frame) {
	if(frame==NULL)
		return;
	log().info() << "Frame selected: " << frame->getName() << std::endl;
}

void ShowLog::initialize() {
    getRobWorkStudio()->frameSelectedEvent().add(
    		boost::bind(&ShowLog::frameSelectedListener, this, _1), this);
}


//----------------------------------------------------------------------

#ifndef RW_STATIC_LINK_PLUGINS
Q_EXPORT_PLUGIN2(Log, ShowLog)
#endif
