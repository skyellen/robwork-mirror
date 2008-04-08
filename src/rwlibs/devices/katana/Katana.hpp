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
        
        /**
         * @brief Connect to the Katana on \b port
         * @param port [in] Port on which to connect
         * @return True if connected 
         */
        bool connectDevice(int port);
        
        /**
         * @brief Disconnect from Katana
         */
        void disconnectDevice();

        /**
         * @brief Returns true if connected to the Katana 
         * @return True if connected to the Katana
         */
        bool isConnected();
        
        /**
         * @brief Calls calibrate on the Katana.
         * 
         * @return True if the Katana is calibrating correctly. 
         */
        bool calibrate();

        /**
         * @brief Calls move on the Katana.
         * 
         * @param q [in] Joint configuration \f$\mathbf{q}\in \mathbb{R}^{5}\f$
         * @param blocking [in] unused 
         */
        bool move(const rw::math::Q& q, bool wait = false);
        
        /**
         * @brief Returns the current configuration 
         * 
         * Request the current configuration from the Katana
         * @return The current configuration \f$\in\mathbb{R}^5\f$.
         */
        rw::math::Q getQ();

        /**
         * @brief Sets angle of the gripper
         * @param pos [in] Position of gripper in radians
         * @return If the position was successfully send to the Katana
         */
        bool grip(double pos);
        
        /**
         * @brief Returns the angle of the gripper
         * @return Angle of gripper in radians.
         */
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
