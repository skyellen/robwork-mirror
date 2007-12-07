#ifndef SAFETYCOMPONENT
#define SAFETYCOMPONENT

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>

#include <core/models/Q.hpp>
#include <core/models/DeviceModel.hpp>
#include <core/models/WorkCell.hpp>
#include <core/kinematics/State.hpp>


#include <core/collision/CollisionDetector.hpp>
#include <boost/shared_ptr.hpp>

namespace rwlibs { namespace components {

    /** @addtogroup components */
    /*@{*/

    /**
     * @brief a component for signalling when a device/robot is about to 
     * collide with some predifined virtual area or object.
     */
    class SafetyComponent: public RTT::TaskContext {
    public:
        
        /**
         * @brief constructor
         * @param name [in] name id of this component
         * @param model [in] the devicemodel of the robot
         * @param workcell [in] the virtual workcell wherein the devicemodel is described
         */
        SafetyComponent(const std::string& name,
    		    rw::core::models::DeviceModel* model,
    		    rw::core::models::WorkCell* workcell,
    		    double dt);
        
        /**
         * @brief destructor
         */
        virtual ~SafetyComponent();
    
        /**
         * @brief  
         */
        bool pathInCollision(const rw::core::models::Q& qgoal);
    
    protected:
        //! joint configuration of the device
        RTT::ReadDataPort<rw::core::models::Q> _jointConfig;
        
        //! velocity of the joint configuration
        RTT::ReadDataPort<rw::core::models::Q> _jointVel;
        
        //! Warning port, true if warning condition is satisfied
        RTT::WriteDataPort<bool> _warningPort;
        
        //! Error port, true if error condition is satisfied
        RTT::WriteDataPort<bool> _errorPort;
        
        RTT::ReadDataPort<bool> _errorRecovery;
    
    private:
        bool startup();
        void update();
        void shutdown();
    
        bool inCollision(const rw::core::models::Q& qinit, const rw::core::models::Q& qgoal);
        bool inCollision(const rw::core::models::Q& q);
    
        rw::core::models::Q _last;
        rw::core::models::DeviceModel* _model;
        rw::core::models::WorkCell* _workcell;
        rw::core::kinematics::State _state;
        double _dt;
        std::pair<double, double> _zwarning;
        std::pair<double, double> _zerror;
    
        std::pair<double, double> _radiusWarning;
        std::pair<double, double> _radiusError;
    
        std::pair<double, double> _thetaWarning;
        std::pair<double, double> _thetaError;
        std::pair<rw::core::models::Q, rw::core::models::Q> _jointLimits;
    
        boost::shared_ptr<rw::core::collision::CollisionDetector> _detector;
        double _deltaq;
    
    };

    /* @} */

}} // end namespaces


#endif //SAFETYCOMPONENT
