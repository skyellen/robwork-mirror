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

#include "LuaExecutionThread.hpp"

#include <QtGui>

extern "C" {
	#include <lua.h>
	#include <lualib.h>
	#include <lauxlib.h>
}

#include <rwlibs/swig/ScriptTypes.hpp>
using namespace rw::common;
using namespace rws;



namespace {

	void luaLineHook(lua_State *L, lua_Debug *ar){
		lua_getinfo(L, "nS", ar);
        if (ar->name != NULL && ar->namewhat != NULL) {
		    //std::cout << "Name: " << ar->name << " " << ar->namewhat << std::endl;
		    //std::cout << "Line: " << ar->currentline << " " << ar->linedefined << std::endl;
        }
	}

}

void LuaExecutionThread::set(const std::string& cmd,
             rwlibs::swig::LuaState::Ptr lstate,
             rw::common::LogWriter::Ptr output)
{
    _cmd = cmd;
    _lua = lstate;
    _output = output;
}

 void LuaExecutionThread::run()
 {
     //QObject::moveToThread(this);
     try {
         //const std::string cmd = _editor->textCursor().block().text().toStdString();
         _resVal = luaL_loadbuffer(_lua->get(), _cmd.data(), _cmd.size(), "");
         if (!_resVal)
             _resVal = lua_pcall(_lua->get(), 0, 0, 0);

         if(_resVal){
             //_editor->setLineState(number, CodeEditor::ExecutedError);
             //processError(error, _lua->get(), _output);

             if (_resVal) {
                 _resstring = lua_tostring(_lua->get(), -1);
                 (*_output) << _resstring << "\n";
                 lua_pop(_lua->get(), 1);
             }
         }
     } catch (const Exception& exp) {
         //QMessageBox::critical(NULL, "Lua Editor", tr("Failed to execute script with message '%1'").arg(exp.what()));
         Log::errorLog() << "Lua: Failed to execute script with message \"" << exp.what() << "\" \n";
     } catch (std::exception& e) {
         //QMessageBox::critical(NULL, "Lua Editor", tr("Failed to execute script with message '%1'").arg(e.what()));
         Log::errorLog() << "Lua: Failed to execute script with message \"" << e.what() << "\" \n";
     } 
 }

 void LuaExecutionThread::stop(){
	 luaL_error(_lua->get(), "User interupt!");
 }
