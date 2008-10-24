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

void LogTest() {
    BOOST_MESSAGE("- LogTestSuite");
    /**
     * Test the basic log behavior
     */

    {
        std::stringstream outstream;
        Log::setWriter(Log::infoId(), new LogStreamWriter(&outstream));
        Log::write(Log::infoId(), "Message");
        Log::flush(Log::infoId());
        BOOST_CHECK(outstream.str() == "Message");
        Log::write(Log::infoId(), "1\n");
        char msg[100];
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "Message1");

        RW_LOG_TEXT(Log::infoId(), "Message");
        RW_LOG_TEXT(Log::infoId(), "2"<<std::endl);
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "Message2");
        Log::setWriter(Log::infoId(), new LogStreamWriter(&std::cout));
    }

    /**
     * Test LogBufferedMsg
     */
    {
        // Using plain stringstream did not work with Visual Studio.
        std::ostringstream outstream;
        const std::string ID = "Custom";
        Log::setWriter(ID, new LogBufferedMsg(&outstream));
        RW_LOG_TEXT(ID, "Message");
        RW_LOG_TEXT(ID, "A"<<std::endl);
        RW_LOG_TEXT(ID, "MessageB"<<std::endl);
        Log::flushAll();

        std::istringstream instream(outstream.str());

        // instream.peek();
        // BOOST_CHECK(instream.eof());
        //instream.clear(); //need to clear the eof bit

        char msg[100];
        instream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "MessageA");
        instream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "MessageB");


        Log::setWriter(ID, new LogBufferedMsg(&std::cout));
    }

    /**
     * Test LogBufferedChar with REMOVE_FIRST policy
     */
    {
        std::stringstream* outstream = new std::stringstream();
        const std::string ID = "Custom";
        int size = 6;
        Log::setWriter(ID, new LogBufferedChar(size, outstream, LogBufferedChar::REMOVE_FIRST));
        RW_LOG_TEXT(ID, "0123");
        RW_LOG_TEXT(ID, "4567");
        Log::flush(ID);
        char msg[100];

        outstream->getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "234567");
        outstream->clear();
        RW_LOG_TEXT(ID, "89"<<std::endl);
        RW_LOG_TEXT(ID, "A"<<std::endl);
        Log::flush(ID);
        outstream->getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "89");
        outstream->getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "A");


        Log::write(ID, "0123456789");
        Log::flush(ID);
        outstream->getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "456789");

        //Log::remove(ID);
        Log::setWriter(ID, new LogBufferedMsg(&std::cout));
        //Log::setWriter(ID, new LogBufferedChar(size, &std::cout, LogBufferedChar::REMOVE_FIRST));
    }

    /**
     * Test LogBufferedChar with REMOVE_LAST policy
     */
    {
        std::stringstream outstream;
        const std::string ID = "Custom";
        int size = 6;
        Log::setWriter(ID, new LogBufferedChar(size, &outstream, LogBufferedChar::REMOVE_LAST));
        RW_LOG_TEXT(ID, "0123");
        RW_LOG_TEXT(ID, "4567");
        RW_LOG_TEXT(ID, "89");
        Log::flush(ID);
        char msg[100];

        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "012345");
        outstream.clear();
        RW_LOG_TEXT(ID, "0123456789");
        Log::flush(ID);
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "012345");
        Log::setWriter(ID, new LogBufferedChar(size, &std::cout, LogBufferedChar::REMOVE_LAST));
    }

    /**
     * Test LogBufferedChar with AUTO_FLUSH policy
     */
    {
        std::stringstream outstream;
        const std::string ID = "Custom";
        int size = 6;
        Log::setWriter(ID, new LogBufferedChar(size, &outstream, LogBufferedChar::AUTO_FLUSH));
        RW_LOG_TEXT(ID, "0123");
        RW_LOG_TEXT(ID, "4567");

        char msg[100];
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "01234567");
        outstream.clear();


        RW_LOG_TEXT(ID, "1234");
        Log::flush(ID);

        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "1234");
        Log::setWriter(ID, new LogBufferedChar(size, &std::cout, LogBufferedChar::AUTO_FLUSH));
    }
}
