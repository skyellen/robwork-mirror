#ifndef LOGOUTSTREAM_HPP_
#define LOGOUTSTREAM_HPP_

#include <ostream>
#include "LogWriter.hpp"

namespace rw {
namespace sandbox {

/** @addtogroup common */
/*@{*/
    
/**
 * @brief Writes log output to a std::ostream
 */
class LogStreamWriter : public LogWriter
{
public:
    /**
     * @brief Constructs LogStreamWriter with a target output stream
     * 
     * The LogStreamWriter keeps a reference to the stream object. Destroying
     * the stream object while the LogStreamWriter has a reference to it
     * results in undefined behavior.
     * 
     * @param stream [in] Stream to write to
     */
	LogStreamWriter(std::ostream& stream);
	
	/**
	 * @brief Destructor
	 * 
	 * Calls flush on the output stream before destruction
	 */
	virtual ~LogStreamWriter();
	
	/**
	 * @copydoc LogWriter::write
	 */
	virtual void write(const std::string& str);
	
	/**
	 * @brief Calls flush on the ostream
	 */
	virtual void flush();
	
private:
    std::ostream& _stream;
};

/* @} */

} //end namespace common
} //end namespace rw
#endif /*LOGOUTSTREAM_HPP_*/
