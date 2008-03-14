#ifndef RW_COMMON_LOGWRITER_HPP
#define RW_COMMON_LOGWRITER_HPP

#include <string>

namespace rw {
namespace sandbox {

/** @addtogroup common */
/*@{*/
    
/**
 * @brief Write interface for Logs
 * 
 * LogWriter provides an output strategy for a log.
 */
class LogWriter
{
public:

    /**
     * @brief Descructor
     */
    virtual ~LogWriter();

    /**
     * @brief Flush method
     */
	virtual void flush() = 0;
	
	/**
	 * @brief Writes \b str to the log
	 * @param str [in] message to write
	 */
	virtual void write(const std::string& str) = 0;

	/**
	 * @brief Writes \b str as a line
	 * 
	 * By default writeln writes \b str followed by a '\\n'. However, logs
	 * are free to implement a line change differently.
	 */
    virtual void writeln(const std::string& str) {
        write(str + '\n');        
    }
    
protected:
    LogWriter();
};

/* @} */

} //end namespace common
} //end namespace rw

#endif /*RW_COMMON_LOGWRITER_HPP*/
