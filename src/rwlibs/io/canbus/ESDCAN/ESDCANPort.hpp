#ifndef IEICANPORT_HPP_
#define ESDCANPORT_HPP_

#include <rwlibs/io/canbus/CanPort.hpp>

#include "ntcan.h" 

#include <vector>
#include <iostream>

namespace rwlibs { namespace io {

    /** @addtogroup io */
    /*@{*/

    /**
     * @brief CanPort driver wrapper for the ESDCAN driver.
     */
    class ESDCANPort: public CanPort
    {
    public:
        /**
         * @brief Status struct
         */
        struct CanDeviceStatus {
            /** @brief The identifier/address of the device */
            int netid; 
        };
        
        /**
         * @brief Baud settings for can bus
         */
        typedef enum {CanBaud1000=0, CanBaud666_6, CanBaud500, CanBaud333_3, 
            CanBaud250, CanBaud166, CanBaud125, CanBaud100, CanBaud66_6,
            CanBaud50, CanBaud33_3, CanBaud20, CanBaud12_5, CanBaud10} CanBaud;
        
    private:
        /**
         * 
         */
        ESDCANPort(unsigned int netId);
        virtual ~ESDCANPort();
		
    public:
               
        /**
         * @brief Returns all devices connected to the can bus
         */
        static std::vector<CanDeviceStatus> getConnectedDevices();
        
        /**
         * Gets an instance of IEICANPort by specifiing card and port nr.
         */
        static ESDCANPort* getPortInstance(
            unsigned int netId); // TODO: add baud ad can id type
		
        /**
         * @copydoc CanPort::isOpen
         */
        bool isOpen();

        /**
         * @copydoc CanPort::open
         */
        bool open(/* baudrate, 11/29bit option,  */);
	
        /**
         * @copydoc CanPort::close
         */
        void close();
	
        /**
         * @copydoc CanPort::read
         */
        bool read( CanPort::CanMessage  &msg);
	
        /**
         * @copydoc CanPort::write
         */
        bool write(unsigned int id, const std::vector<unsigned char>& data);
		
    private:
        unsigned int _netId;
        bool _portOpen;
        
        HANDLE _handle;
    };
    
    /*@}*/
    
}}

#endif /*IEICANPORT_HPP_*/
