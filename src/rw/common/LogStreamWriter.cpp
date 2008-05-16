#include "LogStreamWriter.hpp"

#include "macros.hpp"

using namespace rw::common;

LogStreamWriter::LogStreamWriter(std::ostream* stream) :
    _stream(stream)
{
    RW_ASSERT(stream);
}

LogStreamWriter::~LogStreamWriter()
{
    flush();
}

void LogStreamWriter::write(const std::string& str)
{
    *_stream << str;
}

void LogStreamWriter::flush()
{
    _stream->flush();
}
