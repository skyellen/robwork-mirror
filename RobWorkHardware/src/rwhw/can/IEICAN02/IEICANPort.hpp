#ifndef RWHW_IEICANPORT_HPP
#define RWHW_IEICANPORT_HPP


#include "../CanPort.hpp"
#include <rw/common/macros.hpp>
#include <iostream>

namespace rwhw {

    /** @addtogroup rwhw */
    /*@{*/

    /**
     * @brief CanPort driver wrapper for the IEICAN02 driver.
     */
    class IEICANPort: public CanPort
    {
    private:
        /**
         *
         */
        IEICANPort(unsigned int cardIdx,unsigned int portNr);
        virtual ~IEICANPort();

    public:
        /**
         * Gets an instance of IEICANPort by specifiing card and port nr.
         */
        static IEICANPort* getIEICANPortInstance(
            unsigned int cardIdx,
            unsigned int portNr); // TODO: add baud ad can id type

        /**
         * @copydoc CanPort::isOpen
         */
        bool isOpen();

        /**
         * @copydoc CanPort::open
         */
        bool open(/* baudrate, 11/29bit option,  */);
		bool open(int idlow, int idhigh)
		{
			RW_THROW("Method NOT IMPLEMENTED !!!");
			return false;
		}

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
        unsigned int _cardIdx;
        unsigned int _portNr;
        bool _portOpen;
    };

    /*@}*/

}

#endif /*IEICANPORT_HPP_*/
