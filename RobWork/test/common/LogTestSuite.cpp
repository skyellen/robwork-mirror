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

#include "../TestSuiteConfig.hpp"

#include <rw/common/Log.hpp>
#include <rw/common/LogStreamWriter.hpp>
#include <rw/common/LogBufferedMsg.hpp>
//#include <rw/common/LogBufferedChar.hpp>
#include <rw/common/macros.hpp>
#include <sstream>

#include <iostream>

using namespace boost;

using namespace rw::common;

BOOST_AUTO_TEST_CASE(LogTest) {
    /**
     * Test the basic log behavior
     */

    {
        std::stringstream outstream;
        Log::log().setWriter(Log::Info, ownedPtr(new LogStreamWriter(&outstream)) );
        Log::log().write(Log::Info, "Message");
        Log::log().flush(Log::Info);
        BOOST_CHECK_MESSAGE(outstream.str() == "Message", "Should be " << outstream.str());
        Log::log().write(Log::Info, "1\n");
        char msg[100];
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "Message1");

        RW_LOG_INFO("Message");
        RW_LOG_INFO("Message2");
        outstream.getline(msg, 100);
        outstream.getline(msg, 100);
        BOOST_CHECK_MESSAGE(std::string(msg) == "Message2", std::string(msg) << " == Message2");
        Log::log().setWriter(Log::Info, ownedPtr( new LogStreamWriter(&std::cout)) );
    }

    /**
     * Test LogBufferedMsg
     */
    {
        // Using plain stringstream did not work with Visual Studio.
        std::ostringstream outstream;
        const Log::LogIndex ID = Log::User1;
        Log::log().setWriter(ID, ownedPtr(new LogBufferedMsg(&outstream)) );
        Log::log().setEnable(Log::User1Mask);
        RW_LOG(ID, "MessageA");
        RW_LOG(ID, "MessageB");
        Log::log().flushAll();

        std::istringstream instream(outstream.str());

        // instream.peek();
        // BOOST_CHECK(instream.eof());
        //instream.clear(); //need to clear the eof bit

        char msg[100];
        instream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "MessageA");
        instream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "MessageB");


        Log::log().setWriter(ID, ownedPtr(new LogBufferedMsg(&std::cout)) );
    }


    /* STUFF DOES NOT WORK FOR SOME REASON -- SEEMS TO BE ISSUE WITH stringstream--- JAR


    // Test LogBufferedChar with REMOVE_FIRST policy

    {

        std::stringstream ooutstream;
        std::stringstream *outstream = &ooutstream;
        const Log::LogIndex ID = Log::User2;
        int size = 6;
        Log::log().setEnable(Log::User2Mask);
        Log::log().setWriter(ID, ownedPtr(new LogBufferedChar(size, outstream, LogBufferedChar::REMOVE_FIRST)) );

        Log::log().get(ID) << "01234567";
        RW_LOG(ID, "01234567"); // appends std::endl and therefore does not work..
        Log::log().flush(ID);
        char msg[100];

        outstream->getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "012345");

        outstream->clear();

        //Log::log().get(ID) << "89A";
        Log::log().getWriter(ID)->write("89A");
        //ooutstream<< "89A";
        //RW_LOG(ID, "89");
        //RW_LOG(ID, "A");
        //Log::log().flush(ID);
        outstream->getline(msg, 100);
        RW_WARN("line: " << std::string(msg));
        BOOST_CHECK(std::string(msg) == "89A");
        outstream->getline(msg, 100);
        RW_WARN("line: " << std::string(msg));
        BOOST_CHECK(std::string(msg) == "A");


        Log::log().write(ID, "0123456789");
        Log::log().flush(ID);
        outstream->getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "456789");

        //Log::remove(ID);
        Log::log().setWriter(ID, ownedPtr( new LogBufferedMsg(&std::cout)) );
        //Log::setWriter(ID, new LogBufferedChar(size, &std::cout, LogBufferedChar::REMOVE_FIRST));

    }


     //Test LogBufferedChar with REMOVE_LAST policy

     {
        std::stringstream outstream;
        const Log::LogIndex ID = Log::User1;
        int size = 6;
        Log::log().setWriter(ID, ownedPtr(new LogBufferedChar(size, &outstream, LogBufferedChar::REMOVE_LAST)) );
        RW_LOG(ID, "0123");
        RW_LOG(ID, "4567");
        RW_LOG(ID, "89");
        Log::log().flush(ID);
        char msg[100];

        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "0123");
        outstream.clear();
        RW_LOG(ID, "0123456789");
        Log::log().flush(ID);
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "012345");
        Log::log().setWriter(ID, ownedPtr(new LogBufferedChar(size, &std::cout, LogBufferedChar::REMOVE_LAST)) );
    }


    // Test LogBufferedChar with AUTO_FLUSH policy

    {
        std::stringstream outstream;
        const Log::LogIndex ID = Log::User1;
        int size = 6;
        Log::log().setWriter(ID, ownedPtr(new LogBufferedChar(size, &outstream, LogBufferedChar::AUTO_FLUSH)) );
        RW_LOG(ID, "0123");
        RW_LOG(ID, "4567");

        char msg[100];
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "01234567");
        outstream.clear();


        RW_LOG(ID, "1234");
        Log::log().flush(ID);

        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "1234");
        Log::log().setWriter(ID, ownedPtr(new LogBufferedChar(size, &std::cout, LogBufferedChar::AUTO_FLUSH)) );
    }
    */
}
