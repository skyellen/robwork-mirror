#include "LogWriter.hpp"

#include <sstream>

using namespace rw::common;

void LogWriter::write(const Message& msg)
{
    std::ostringstream buf;
    buf << msg;
    write(buf.str());
}

void LogWriter::writeln(const std::string& str)
{
    write(str + '\n');
}

LogWriter::~LogWriter()
{}
