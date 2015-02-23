/* */
#ifndef RWHW_ROBOTIQ2_HPP
#define RWHW_ROBOTIQ2_HPP

#include "Robotiq.hpp"

namespace rwhw {
class Robotiq2: public Robotiq {
public:

    // for robot input
    /*
     enum{ GRIPPER_STATUS=0,
           RESERVED,
           FAULT_STATUS,
           POS_REQUEST_ECHO,
           POSITION,
           CURRENT
     };

     // for robot output
     enum{ ACTION_REQUEST=0,
           RESERVED,
           RESERVED,
           POSITION_REQUEST,
           SPEED,
           FORCE
     };
     */

    /// INPUT STUFF
    union FaultStatus {
        struct {
            unsigned int gFLT :4;
            unsigned int gRS4 :4;
        } data;
        boost::uint8_t value;
    };

    union GripperStatus {
        struct {
            unsigned int gACT :1;
            unsigned int gRS1 :1;
            unsigned int gRS2 :1;
            unsigned int gGTO :1;
            unsigned int gSTA :2;
            unsigned int gOBJ :2;
        } data;
        boost::uint8_t value;
    };

    // internal status variables
    struct GripperStatusAll {
        union {
            struct {
                boost::uint8_t _gripperStatus;
                boost::uint8_t _reserved;
                boost::uint8_t _faultStatus;

                boost::uint8_t _pos_req_echo;
                boost::uint8_t _pos;
                boost::uint8_t _current;
            } data;
            boost::uint8_t value[6];
        };
    };

    /// OUTPUT STUFF
    union ActionRequest {
        struct {
            unsigned int rACT :1;
            unsigned int rRS1 :1;
            unsigned int rRS2 :1;
            unsigned int rGTO :1;
            unsigned int rATR :1;
            unsigned int rRS3 :1;
            unsigned int rRS4 :1;
            unsigned int rRS5 :1;
        } data;
        boost::uint8_t value;
    };

    union ActionRequestCMD {
        struct {
            boost::uint8_t _actionRequest;
            boost::uint8_t _reserved1;
            boost::uint8_t _reserved2;

            boost::uint8_t _pos_req;
            boost::uint8_t _speed;
            boost::uint8_t _force;
        } data;
        boost::uint8_t value[6];
    };

    Robotiq2();

    virtual std::pair<rw::math::Q, rw::math::Q> getLimitPos();
    virtual std::pair<rw::math::Q, rw::math::Q> getLimitVel();
    virtual std::pair<rw::math::Q, rw::math::Q> getLimitForce();

    virtual bool isActivated(){ return _statusDataValid && _gripperStatus.data.gACT == 1; }
    virtual bool isGripperMoving(){ return _statusDataValid && _gripperStatus.data.gOBJ == 0; }
    virtual bool isGripperBlocked(){ return _statusDataValid && (_gripperStatus.data.gOBJ == 2 || _gripperStatus.data.gOBJ == 1); }
    virtual bool isGripperAtTarget(){ return _statusDataValid && _gripperStatus.data.gOBJ == 3; }

    double getFingerDistanceInMetersFromTicks(int ticks) const;
    int getTicksFromFingerDistanceInMeters(double distance) const;
    double getVelocityInMetersPerSecFromTicks(int ticks) const;
    int getTicksFromVelocityInMetersPerSec(double velocity) const;
    double getApproximateForceInNewtonFromTicks(int ticks) const;
    int getApproximateTicksFromForceInNewton(double force) const;

protected:
    FaultStatus _faultStatus;
    GripperStatus _gripperStatus;
    bool _statusDataValid;

    virtual bool isGripperInReset();
    virtual bool isGripperInActivationProcess();
    virtual ModbusPackage getMoveCMDRequestPackage(const rw::math::Q & target) const;
    virtual ModbusPackage getAllStatusCMDRequestPackage() const;
    virtual void validateStatusPackageAndUpdateState(const ModbusPackage & package);
    virtual ModbusPackage getStopCMDRequestPackage() const;
    virtual void validateStopCMDResponseMessage(const ModbusPackage & answer) const;
    virtual ModbusPackage getActivateRequestPackage() const;
};

} //end namespace

#endif //#ifndef RWHW_ROBOTIQ2_HPP
