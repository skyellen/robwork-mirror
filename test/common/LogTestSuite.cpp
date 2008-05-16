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
    /**
     * Test the basic log behavior
     */
    {
        std::stringstream outstream;
        Log::setWriter(Log::Info, new LogStreamWriter(&outstream));
        Log::write(Log::Info, "Message");
        Log::flush(Log::Info);
        BOOST_CHECK(outstream.str() == "Message");
        Log::write(Log::Info, "1\n");
        char msg[100];
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "Message1");
        
        RW_LOG_TEXT(Log::Info, "Message");
        RW_LOG_TEXT(Log::Info, "2"<<std::endl);
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "Message2");
    }    
    /**
     * Test LogBufferedMsg
     */
    {
        std::stringstream outstream;
        const std::string ID = "Custom";
        Log::setWriter(ID, new LogBufferedMsg(outstream));
        RW_LOG_TEXT(ID, "Message");
        RW_LOG_TEXT(ID, "A"<<std::endl);
        RW_LOG_TEXT(ID, "MessageB"<<std::endl);
        
        outstream.peek();
        BOOST_CHECK(outstream.eof());
        outstream.clear(); //need to clear the eof bit
        
        Log::flushAll();
    
        char msg[100];
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "MessageA");
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "MessageB");
        
        RW_LOG_TEXT(ID, "MessageC"<<std::endl;);
        Log::flush(ID);
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "MessageC");
    }
    
    /**
     * Test LogBufferedChar with REMOVE_FIRST policy
     */
    {
        std::stringstream outstream;
        const std::string ID = "Custom";
        int size = 6;
        Log::setWriter(ID, new LogBufferedChar(size, outstream, LogBufferedChar::REMOVE_FIRST));
        RW_LOG_TEXT(ID, "0123");    
        RW_LOG_TEXT(ID, "4567");
        Log::flush(ID);
        char msg[100];
        
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "234567");
        outstream.clear();
        RW_LOG_TEXT(ID, "89"<<std::endl);
        RW_LOG_TEXT(ID, "A"<<std::endl);
        Log::flush(ID);
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "89");
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "A");
        
        Log::write(ID, "0123456789");
        Log::flush(ID);
        outstream.getline(msg, 100);
        BOOST_CHECK(std::string(msg) == "012345");
    }    

    /**
     * Test LogBufferedChar with REMOVE_LAST policy
     */
    {
        std::stringstream outstream;
        const std::string ID = "Custom";
        int size = 6;
        Log::setWriter(ID, new LogBufferedChar(size, outstream, LogBufferedChar::REMOVE_LAST));
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
    }    

    /**
     * Test LogBufferedChar with AUTO_FLUSH policy
     */
    {
        std::stringstream outstream;
        const std::string ID = "Custom";
        int size = 6;
        Log::setWriter(ID, new LogBufferedChar(size, outstream, LogBufferedChar::AUTO_FLUSH));
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
    } 
}
