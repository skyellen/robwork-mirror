#ifndef RW_CONTROL_VIRTUALDEVICE_HPP
#define RW_CONTROL_VIRTUALDEVICE_HPP

#include <core/models/Q.hpp>
#include <core/models/WorkCell.hpp>
#include <core/models/DeviceModel.hpp>
#include <core/kinematics/State.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>

#include <fstream>

namespace rwlibs { namespace components {

    /** @addtogroup components */
    /*@{*/

    /**
     * @brief a component that simulates a real device with velocity profiles
     * generated from accelleration and velocity limits from the devicemodel
     */

    class VirtualDevice: public RTT::TaskContext
    {
    public:
        VirtualDevice(const std::string& name,
                      rw::core::models::WorkCell* workcell,
                      rw::core::models::DeviceModel* model,
                      double dt,
                      double updateRate);
    
        virtual ~VirtualDevice();
    
        rw::core::models::Q getQ();


    protected:

        RTT::ReadDataPort< std::vector<rw::core::models::Q> > _qPathIn;
    
        RTT::WriteDataPort<rw::core::models::Q> _qOut;
    
        RTT::WriteDataPort<rw::core::models::Q> _qdotOut;
    
        RTT::DataPort<bool> _errorPort;
    
        private:
        bool startup();
        void update();
        void shutdown();
    
        rw::core::models::DeviceModel* _device;
        rw::core::kinematics::State _state;
        double _dt;
        double _updateRate;
        rw::core::models::Q _qdotlast;
        rw::core::models::Q _acclimits;
        std::ofstream _log;
        std::vector<rw::core::models::Q> _lastPath;
        rw::core::models::Q _currStepSize;

    };

    /* @} */

}} // end namespaces
    

#endif //RW_CONTROL_VIRTUALDEVICE_HPP
