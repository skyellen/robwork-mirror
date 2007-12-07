#ifndef RWLIBS_DEVICES_KATANA_HPP
#define RWLIBS_DEVICES_KATANA_HPP

#include <rw/math/Q.hpp>
#include <rw/common/macros.hpp>

 

#include <string>


class CCdlCOM;
class CCplSerialCRC;
class CKatana;


namespace rwlibs {
namespace devices {

    /** @addtogroup devices */
    /*@{*/
    
    /**
     * @brief Katana device driver interface. 
     */
    
    class Katana {
    public:
        /**
         * @brief default constructor
         */
        Katana(const std::string& configfile);
        
        /**
         * @brief default destructor
         */        
        ~Katana();
        
        bool connectDevice(int port);
        
        void disconnectDevice();

        bool isConnected();
        
        bool calibrate();
        
        bool move(const rw::math::Q& q, bool wait = false);
        
        rw::math::Q getQ();

        bool grip(double pos);
        
        double getGripperPos();
        
    private:
        std::string _configfile;
        CCdlCOM* _communication;
        CCplSerialCRC* _protocol;
        CKatana* _katana;
        rw::math::Q _qcurrent;
        
        
    };
    
    /*@}*/

} //end namespace devices
} //end namespace rwlibs

#endif //#ifndef RWLIBS_DEVICES_KATANA_HPP
