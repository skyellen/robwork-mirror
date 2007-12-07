#ifndef IEICANPORT_HPP_
#define IEICANPORT_HPP_


#include "../CanPort.hpp"
#include <iostream>

namespace rwlibs { namespace io {

    /** @addtogroup io */
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
    
}}

#endif /*IEICANPORT_HPP_*/
