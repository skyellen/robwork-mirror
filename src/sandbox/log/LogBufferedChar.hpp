#ifndef LOGBUFFEREDCHAR_HPP_
#define LOGBUFFEREDCHAR_HPP_

#include <ostream>
#include "LogWriter.hpp"

namespace rw {
namespace sandbox {

/** @addtogroup common */
/*@{*/

/**
 * @brief Buffers characters before writing them to the output stream. 
 * 
 * The size of the buffer is fixed. On overflow the behavior depends on the 
 * specified OverFlowPolicy. If a single message is larger than
 * the entire content of the buffer it is truncated.
 * 
 */
class LogBufferedChar: public LogWriter
{
public:
    /**
     * @brief Behaviors for the OverflowPolicy 
     */     
    enum OverflowPolicy { 
        //! Remove the first added content (circular queue)
        REMOVE_FIRST,  
        //! Skip the content which does not fit input the buffer
        REMOVE_LAST, 
        //! Automatically calls flush to write the buffer and the new message to the output stream. Using AUTO_FLUSH it is possible to write messages larger than the buffer size. 
        AUTO_FLUSH };   
        
    /**
     * @brief Constructs a LogBufferedChar
     *      
     * The LogBufferedMsg keeps a reference to the stream object. Destroying
     * the stream object while the LogBufferedChar has a reference to it
     * results in undefined behavior.
     * 
     * @param size [in] Size of buffer (in characters)
     * @param stream [in] Stream to write to
     * @param policy [in] Overflow policy. Default is REMOVE_FIRST
     */    
	LogBufferedChar(size_t size, std::ostream& stream, OverflowPolicy policy = REMOVE_FIRST);
	
	/**
	 * @brief Destructor
	 * 
	 * Calls flush before destroying the object
	 */
	virtual ~LogBufferedChar();
	
	/**
	 * @brief Writes str to the buffer
	 * @param str [in] String to write
	 */
	void write(const std::string& str);
	
	/**
	 * @brief Flushes the buffer to the output stream
	 */
	void flush();
	
private:
    std::ostream& _stream;
    char* _buffer;
    size_t _size;
    int _index;
    OverflowPolicy _policy;
    bool _overflow;
};


/* @} */

} //end for namespace sandbox
} //end for namespace rw

#endif /*LOGBUFFEREDCHAR_HPP_*/
