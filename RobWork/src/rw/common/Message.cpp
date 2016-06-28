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

#include "Message.hpp"

#include <boost/filesystem.hpp>

using namespace rw::common;

Message::Message(const std::string& file, int line, const std::string& message):
#if(BOOST_FILESYSTEM_VERSION==2)
    _file( boost::filesystem::path(file.c_str()).filename() ),
#else
    _file( boost::filesystem::path(file.c_str()).filename().string() ),
#endif
    _line(line),
    _message(message)
{
		
}



std::ostream&
rw::common::operator<<(std::ostream& out, const Message& msg)
{
    out
        << msg.getFile()
        << ":"
        << msg.getLine()
        << " "
        << msg.getText()
        << std::endl;
    return out;
}
