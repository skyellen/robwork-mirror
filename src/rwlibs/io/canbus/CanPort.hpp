#ifndef CANPORT_HPP_
#define CANPORT_HPP_

#include <vector>
#include <string>

namespace rwlibs { namespace io {

    /** @addtogroup io */
    /*@{*/

    /**
     * @brief CanPort provides an interface for communication through CAN bus. 
     *
     */

    class CanPort
    {

    public:
        
        /**
         * @brief The structure for transmitting and recieving can messg  es
         */
        struct CanMessage {
            unsigned int timeStamp;  // Timestamp for receive queue objects
            unsigned int id;         // Identifier 11/29-Bit
            unsigned char length;    // number of data bytes ( 0~8)
            unsigned char rtr;       // RTR-Bit: 0=dataframe, 1=Remoteframe
            unsigned char data[8];   // Array for up to 8 data bytes
        };


    public:
        
        /**
         * @brief Destructor
         */
        virtual ~CanPort() {};
        
        /**
         * @brief returns wether this can port is open.
         */
        virtual bool isOpen() = 0;

        /**
         * @brief opens this can port
         */
        virtual bool open(/* baudrate, 11/29bit option,  */) = 0;

        /**
         * @brief closes this can port
         */
        virtual void close() = 0;

        /**
         * @brief reads a message from the recieve buffer
         */
        virtual bool read( CanPort::CanMessage  &msg) = 0;

        /**
         * @brief writes message to the transmit buffer
         */
        virtual bool write(
            unsigned int id,
            const std::vector<unsigned char>& data) = 0;
    };
    
    /*@}*/
    
} // namespace io
} // namespace rwlibs


#endif /*CANPORT_HPP_*/
