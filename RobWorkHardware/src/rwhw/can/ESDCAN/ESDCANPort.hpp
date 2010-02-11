#ifndef RWHW_ESDCANPORT_HPP
#define RWHW_ESDCANPORT_HPP

#include <rwhw/can/CanPort.hpp>

#include <vector>
#include <iostream>

#include <ntcan.h>

namespace rwhw {

    /** @addtogroup rwhw  */
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
        ESDCANPort(unsigned int netId, long txQueueSize, long rxQueueSize, CanBaud canBaud, int transmitDelay);
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
            unsigned int netId, long txQueueSize=1, long rxQueueSize=1,
			CanBaud canBaud=CanBaud250, int transmitDelay=0); // TODO: add baud ad can id type

        /**
         * @copydoc CanPort::isOpen
         */
        bool isOpen();

        /**
         * @copydoc CanPort::open
         */
        bool open();
        bool open(int idlow, int idhigh);

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

		void setBaudRate(CanBaud canBaud);

		NTCAN_HANDLE getHandle() { return _handle; }

    private:
        unsigned int _netId;
        bool _portOpen;
		long _txQueueSize;
		long _rxQueueSize;
		CanBaud _canBaud;
		int _transmitDelay;

		NTCAN_HANDLE _handle;
    };

    /*@}*/

}

#endif /*ESDCANPORT_HPP_*/
