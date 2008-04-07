#ifndef LOGBUFFEREDMSG_HPP_
#define LOGBUFFEREDMSG_HPP_

#include <ostream>
#include <list>
#include <string>
#include "LogWriter.hpp"


namespace rw {
namespace common {


/**
 * @brief Buffers messages before writing them to the output stream. 
 * 
 * The size of the buffer is not fixed and will grow until flush is called.
 * To have a fixed size buffer use LogBufferedChar instead.
 */
class LogBufferedMsg: public LogWriter
{
public:    
    /**
     * @brief Constructs LogBufferedMsg with a target ostream
     * 
     * The LogBufferedMsg keeps a reference to the stream object. Destroying
     * the stream object while the LogBufferedMsg has a reference to it
     * results in undefined behavior.
     * 
     * @param stream [in] Stream to write to
     */
	LogBufferedMsg(std::ostream& stream);
	
	/**
	 * @brief Destructor
	 * 
	 * Calls flush before destruction
	 */
	virtual ~LogBufferedMsg();
	
		
	/**
	 * @brief Writes str to the buffer
	 * @param str [in] str to write
	 */
	virtual void write(const std::string& str);

	/**
	 * @brief Write content of buffer to output stream and flush it
	 */
	virtual void flush();
   
private:
    std::list<std::string> _buffer;
    std::ostream& _stream;

};

} //end namespace sandbox
} //end namespace rw

#endif /*LOGBUFFEREDMSG_HPP_*/
