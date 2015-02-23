/* */
#ifndef RWHW_ROBOTIQ3_HPP
#define RWHW_ROBOTIQ3_HPP


#include "Robotiq.hpp"

namespace rwhw {

/**
 * @brief Implements the interface for a Robotiq-3 finger
 * The Robotiq is an underactuated robot hand where all joints depends
 * on a few joints.
 *
 * The joint limits.
 * [0:255] - thumb
 * [0:255] - finger 1
 * [0:255] - finger 2
 * [0:255] - the scissor grasping
 *
 * The speed limits and force limits are all [0:255]
 *
 */

class Robotiq3 : public Robotiq {
public:
    typedef rw::common::Ptr<Robotiq3> Ptr;

    /*
     Global Gripper Status - A global Gripper Status is available. This gives information such as which Operation
     Mode is currently active or if the Gripper is closed or open.

     Object Status - There is also an Object Status that let you know if there is an object in the Gripper and, in the
     affirmative, how many fingers are in contact with it.

     Fault Status - The Fault Status gives additional details about the cause of a fault.

     Position Request Echo - The Gripper returns the position requested by the robot to make sure that the new
     command has been received correctly.

     Motor Encoder Status - The information of the encoders of the four motors is also available.

     Current Status - The current of the motors can also be known. Since the torque of the motor is a linear
     function of the current, this gives information about the torque applied.
     *
     */

    /// INPUT STUFF
    union FaultStatus {
        struct {
            unsigned int gFLT :4;
            unsigned int gRS1 :4;
        } data;
        boost::uint8_t value;
    };

    union ObjectStatus {
        struct {
            unsigned int gDTA :2;
            unsigned int gDTB :2;
            unsigned int gDTC :2;
            unsigned int gDTS :2;

        } data;
        boost::uint8_t value;
    };

    union GripperStatus {
        struct {
            unsigned int gACT :1;
            unsigned int gMOD :2;
            unsigned int gGTO :1;
            unsigned int gIMC :2;
            unsigned int gSTA :2;
        } data;
        boost::uint8_t value;
    };

    // internal status variables
    struct GripperStatusAll {
        union {
            struct {
                boost::uint8_t _gripperStatus;
                boost::uint8_t _objectStatus;
                boost::uint8_t _faultStatus;

                boost::uint8_t _posA_req;
                boost::uint8_t _posA;
                boost::uint8_t _currentA;

                boost::uint8_t _posB_req;
                boost::uint8_t _posB;
                boost::uint8_t _currentB;

                boost::uint8_t _posC_req;
                boost::uint8_t _posC;
                boost::uint8_t _currentC;

                boost::uint8_t _posScissor_req;
                boost::uint8_t _posScissor;
                boost::uint8_t _currentScissor;

                boost::uint8_t _reserved;
            } data;
            boost::uint8_t value[16];
        };
    };

    /// OUTPUT STUFF
    union ActionRequest {
        struct {
            unsigned int rACT :1;
            unsigned int rMOD :2;
            unsigned int rGTO :1;
            unsigned int rATR :1;
            unsigned int rRS0 :3;
        } data;
        boost::uint8_t value;
    };
    union GripperOptions {
        struct {
            unsigned int rGLV :1;
            unsigned int rAAC :1;
            unsigned int rICF :1;
            unsigned int rICS :1;
            unsigned int rRS0 :4;
        } data;
        boost::uint8_t value;
    };

    union ActionRequestCMD {
            struct {
                boost::uint8_t _actionRequest;
                boost::uint8_t _gripperOptions;
                boost::uint8_t _gripperOptions2;

                boost::uint8_t _posA_req;
                boost::uint8_t _speedA;
                boost::uint8_t _forceA;

                boost::uint8_t _posB_req;
                boost::uint8_t _speedB;
                boost::uint8_t _forceB;

                boost::uint8_t _posC_req;
                boost::uint8_t _speedC;
                boost::uint8_t _forceC;

                boost::uint8_t _posScissor_req;
                boost::uint8_t _speedScissor;
                boost::uint8_t _forceScissor;

                boost::uint8_t _reserved;
            } data;
            boost::uint8_t value[16];
    };

    Robotiq3();

    virtual std::pair<rw::math::Q,rw::math::Q> getLimitPos();
    virtual std::pair<rw::math::Q,rw::math::Q> getLimitVel();
    virtual std::pair<rw::math::Q,rw::math::Q> getLimitForce();

    virtual bool isActivated(){ return _statusDataValid && _gripperStatus.data.gACT == 1; }
    virtual bool isGripperMoving(){ return _statusDataValid && _gripperStatus.data.gSTA == 0; }
    virtual bool isGripperBlocked(){ return _statusDataValid && (_gripperStatus.data.gSTA == 2 || _gripperStatus.data.gSTA == 1); }
    virtual bool isGripperAtTarget(){ return _statusDataValid && _gripperStatus.data.gSTA == 3; }

    double getVelocityInMetersPerSecFromTicks( int ticks) const;
    int getTicksFromVelocityInMetersPerSec( double velocity) const;
    double getApproximateForceInNewtonFromTicks( int ticks) const;
    int getApproximateTicksFromForceInNewton( double force) const;

protected:
    FaultStatus _faultStatus;
    ObjectStatus _objectStatus;
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

    GripperStatusAll _status;

};


} //end namespace

#endif //#ifndef RWHW_ROBOTIQ3_HPP
