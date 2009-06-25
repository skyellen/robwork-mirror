/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include <rw/common/Log.hpp>
#include <rw/common/LogStreamWriter.hpp>
#include <rw/common/LogBufferedMsg.hpp>
#include <rw/common/LogBufferedChar.hpp>
#include <rw/common/macros.hpp>
#include <sstream>

#include <boost/test/unit_test.hpp>
#include <boost/shared_ptr.hpp>

#include <iostream>

using namespace boost;

using namespace rw::common;

BOOST_AUTO_TEST_CASE(LogTest) {
    /**
     * Test the basic log behavior
     */

    {
        std::stringstream outstream;
        Log::log().setWriter(Log::Info, new LogStreamWriter(&outstream));
        Log::log().write(Log::Info, "Message");
        Log::log().flush(Log::Info);
        BOOST_CHECK(outstream.str() == "Message");
        Log::log().write(Log::Info, "1\n");
        char msg[100];
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "Message1");

        RW_LOG_TEXT(Log::Info, "Message");
        RW_LOG_TEXT(Log::Info, "2"<<std::endl);
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "Message2");
        Log::log().setWriter(Log::Info, new LogStreamWriter(&std::cout));
    }

    /**
     * Test LogBufferedMsg
     */
    {
        // Using plain stringstream did not work with Visual Studio.
        std::ostringstream outstream;
        const Log::LogLevel ID = Log::User1;
        Log::log().setWriter(ID, new LogBufferedMsg(&outstream));
        RW_LOG_TEXT(ID, "Message");
        RW_LOG_TEXT(ID, "A"<<std::endl);
        RW_LOG_TEXT(ID, "MessageB"<<std::endl);
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


        Log::log().setWriter(ID, new LogBufferedMsg(&std::cout));
    }

    /**
     * Test LogBufferedChar with REMOVE_FIRST policy
     */
    {
        std::stringstream* outstream = new std::stringstream();
        const Log::LogLevel ID = Log::User2;
        int size = 6;
        Log::log().setWriter(ID, new LogBufferedChar(size, outstream, LogBufferedChar::REMOVE_FIRST));
        RW_LOG_TEXT(ID, "0123");
        RW_LOG_TEXT(ID, "4567");
        Log::log().flush(ID);
        char msg[100];

        outstream->getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "234567");
        outstream->clear();
        RW_LOG_TEXT(ID, "89"<<std::endl);
        RW_LOG_TEXT(ID, "A"<<std::endl);
        Log::log().flush(ID);
        outstream->getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "89");
        outstream->getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "A");


        Log::log().write(ID, "0123456789");
        Log::log().flush(ID);
        outstream->getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "456789");

        //Log::remove(ID);
        Log::log().setWriter(ID, new LogBufferedMsg(&std::cout));
        //Log::setWriter(ID, new LogBufferedChar(size, &std::cout, LogBufferedChar::REMOVE_FIRST));
    }

    /**
     * Test LogBufferedChar with REMOVE_LAST policy
     */
    {
        std::stringstream outstream;
        const Log::LogLevel ID = Log::User1;
        int size = 6;
        Log::log().setWriter(ID, new LogBufferedChar(size, &outstream, LogBufferedChar::REMOVE_LAST));
        RW_LOG_TEXT(ID, "0123");
        RW_LOG_TEXT(ID, "4567");
        RW_LOG_TEXT(ID, "89");
        Log::log().flush(ID);
        char msg[100];

        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "012345");
        outstream.clear();
        RW_LOG_TEXT(ID, "0123456789");
        Log::log().flush(ID);
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "012345");
        Log::log().setWriter(ID, new LogBufferedChar(size, &std::cout, LogBufferedChar::REMOVE_LAST));
    }

    /**
     * Test LogBufferedChar with AUTO_FLUSH policy
     */
    {
        std::stringstream outstream;
        const Log::LogLevel ID = Log::User1;
        int size = 6;
        Log::log().setWriter(ID, new LogBufferedChar(size, &outstream, LogBufferedChar::AUTO_FLUSH));
        RW_LOG_TEXT(ID, "0123");
        RW_LOG_TEXT(ID, "4567");

        char msg[100];
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "01234567");
        outstream.clear();


        RW_LOG_TEXT(ID, "1234");
        Log::log().flush(ID);

        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "1234");
        Log::log().setWriter(ID, new LogBufferedChar(size, &std::cout, LogBufferedChar::AUTO_FLUSH));
    }
}
