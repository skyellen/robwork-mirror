#include "LogStreamWriter.hpp"

using namespace rw::common;

LogStreamWriter::LogStreamWriter(std::ostream& stream):
    _stream(stream)
{
}

LogStreamWriter::~LogStreamWriter()
{
}


void LogStreamWriter::write(const std::string& str) {
    _stream<<str;
}

void LogStreamWriter::flush() {
    _stream.flush();
}
