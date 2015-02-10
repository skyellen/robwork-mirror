/* */
#ifndef RWHW_ROBOTIQ3_HPP
#define RWHW_ROBOTIQ_HPP


#include <rw/math/Q.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/task/Task.hpp>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread.hpp>

#include <queue>
#include <fstream>

namespace rwhw {

/**
 * @brief Implements the interface for a Robotiq-3 finger
 * The Robotiq is an underactuated robot hand where all joints depends
 * on a few joints. The limits are therefore approximate
 *
 * The joint limits and their mapping to Robotiq control input.
 * [0:66] -> [0:0xFF] - thumb
 * [0:66] -> [0:0xFF] - finger 1
 * [0:66] -> [0:0xFF] - finger 2
 * [-16:10 degree] -> [0:0xFF] - the scissor grasping, angle between
 *
 * The speed limits and force limits are all [0:1]
 *
 */
class Robotiq  {

public:
    typedef rw::common::Ptr<Robotiq> Ptr;



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
       function of the current, this gives infor
     *
     */

    // for robot input
    /*
    enum{ GRIPPER_STATUS=0,
           OBJECT_DETECTION,
           FAULT_STATUS,
           POS_REQUEST_ECHO,
           FINGER_A_POS,
           FINGER_A_CURRENT,
           FINGER_B_POS_REQ_ECHO,
           FINGER_B_POS,
           FINGER_B_CURRENT,
           FINGER_C_POS_REQ_ECHO,
           FINGER_C_POS,
           FINGER_C_CURRENT,
           SCISSOR_POS_REQ_ECHO,
           SCISSOR_POS,
           SCISSOR_CUR,
           RESERVED
    };

    // for robot output
    enum{ ACTION_REQUEST=0,
          GRIPPER_OPTIONS,
          GRIPPER_OPTIONS2,
          POS_REQ, // finger A in individual mode
          SPEED,
          FORCE,
          FINGER_B_POS_REQ,
          FINGER_B_SPEED,
          FINGER_B_FORCE,
          FINGER_C_POS_REQ,
          FINGER_C_SPEED,
          FINGER_C_FORCE,
          SCISSOR_POS_REQ,
          SCISSOR_SPEED,
          SCISSOR_FORCE,
          RESERVED
    };
    */


    /// INPUT STUFF

        union FaultStatus{
            struct  {
                unsigned int gFLT : 3;
                unsigned int gRS1 : 5;
            } data;
            boost::uint8_t value;
        } ;
        FaultStatus _faultStatus;

        union  ObjectStatus {
            struct {
                unsigned int gDTA : 2;
                unsigned int gDTB : 2;
                unsigned int gDTC : 2;
                unsigned int gDTS : 2;

            } data;
            boost::uint8_t value;
        };
        ObjectStatus _objectStatus;

        union GripperStatus {
            struct {
                unsigned int gACT : 1;
                unsigned int gMOD : 2;
                unsigned int gGTO : 1;
                unsigned int gIMC : 2;
                unsigned int gSTA : 2;
            } data;
            boost::uint8_t value;
        };
        GripperStatus _gripperStatus;

    // internal status variables
    struct GripperStatusAll {
        union {
            struct {
                boost::uint8_t _gripperStatus;
                boost::uint8_t _objectStatus;
                boost::uint8_t _faultStatus;

                boost::uint8_t _posA_req;
                boost::uint8_t _posA;
                boost::uint8_t _forceA;

                boost::uint8_t _posB_req;
                boost::uint8_t _posB;
                boost::uint8_t _forceB;

                boost::uint8_t _posC_req;
                boost::uint8_t _posC;
                boost::uint8_t _forceC;

                boost::uint8_t _posScissor_req;
                boost::uint8_t _posScissor;
                boost::uint8_t _forceScissor;

                boost::uint8_t _reserved;
            } data;
            boost::uint8_t value[16];
        };
    };

    union ActionRequest {
        struct {
            unsigned int rACT : 1;
            unsigned int rMOD : 2;
            unsigned int rGTO : 1;
            unsigned int rATR : 1;
            unsigned int rRS0 : 3;
        } data;
        boost::uint8_t value;
    } ;
    union GripperOptions {
        struct {
            unsigned int rGLV : 1;
            unsigned int rAAC : 1;
            unsigned int rICF : 1;
            unsigned int rICS : 1;
            unsigned int rRS0 : 4;
        } data;
        boost::uint8_t value;
    } ;

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

    /// OUTPUT STUFF

    struct ModbusPackage {
        union {
            struct {
                boost::uint16_t transactionID;
                boost::uint16_t protocolID;
                boost::uint16_t length;
                boost::uint8_t unitID;
                boost::uint8_t functionCode;
            } data;
            boost::uint8_t value[8];
        } header;

        boost::uint8_t data[30]; // this is a limitation of the Robotiq and not the gripper
    };


    /**
     * @brief Creates object
     */
    Robotiq();

    //! destructor
    virtual ~Robotiq();

    /**
     * @brief connect using
     * @param host
     * @param port
     * @return
     */
    bool connect(const std::string& host, unsigned int port=502);
    bool isConnected() const{ return _connected; }
    void disconnect();

    // blocking commands
    void getAllStatus();
    bool isActivated(){ return _gripperStatus.data.gACT == 1; }
    bool isGripperMoving(){ return _gripperStatus.data.gSTA == 0; }
    bool isGripperBlocked(){ return _gripperStatus.data.gSTA == 2 || _gripperStatus.data.gSTA == 1; }
    bool isGripperStopped(){ return _gripperStatus.data.gSTA == 3; }

    /// user interface stuff

    /**
     * @brief sends a move to \b target command to the hand
     * @param block [in] if true the method will block until the hand
     * has reached the target or if it is not moving for a timeout period
     */
    void moveCmd(bool block);

    /**
     * @brief sends a move to \b target command to the hand
     * @param target [in] the target to move to
     * @param block [in] if true the method will block until the hand
     * has reached the target or if it is not moving for a timeout period
     */
    void moveCmd(rw::math::Q target, bool block=false);


    /**
     * @brief Like moveCmd but this only moves one joint.
     * @param target
     * @param block
     */
    void moveJointCmd(int jointIdx, double target, bool block=false);

    /**
     * @brief waits until the joint positions has reached
     * the target of the last moveCmd or timeout occurs. A timeout of -1 means
     * wait indefinitely.
     * @param timeout [in] in seconds
     * @return true if target was reached, false otherwise
     */
    bool waitCmd(double timeout);

    /**
     * @brief stop movement of all gripper joints
     */
    void stopCmd();

    /**
     * @brief sets the wanted target in rad.
     * @param jointPos
     */
    void setTargetQ(const rw::math::Q& jointPos);

    /**
     * @brief sets the wanted target velocity. The velocity must be within
     * the velocity limits.
     * @param jointVel
     */
    void setTargetQVel(const rw::math::Q& jointVel);

    /**
     * @brief sets the wanted target Acceleration. The acceleration must be within
     * the acceleration limits.
     * @param jointAcc
     */
    void setTargetQAcc(const rw::math::Q& jointAcc);

    /**
     * @brief sets the wanted target Acceleration. The acceleration must be within
     * the acceleration limits.
     * @param jointAcc
     */
    void setTargetQCurrent(const rw::math::Q& jointCurr);

    /**
     * @brief queries the hand for its current target configuration
     * @return
     */
    rw::math::Q getTargetQ();

    /**
     * @brief queries the hand for its joint configuration.
     */
    rw::math::Q getQ();

    /**
     * @brief queries the hand for its current velocity.
     */
    rw::math::Q getdQ();

    /**
     * @brief queries the hand for its current power use.
     */
    rw::math::Q getQCurrent();


    std::pair<rw::math::Q,rw::math::Q> getLimitPos();
    std::pair<rw::math::Q,rw::math::Q> getLimitVel();
    std::pair<rw::math::Q,rw::math::Q> getLimitCurr();

private:
rw::common::Ptr<boost::thread> _thread;
mutable boost::mutex _mutex;
    bool _stop;
    void activate();
    void run();
    void start();
    void stop();
    ModbusPackage send(ModbusPackage package);

    bool _haveReceivedSize;
    uint32_t messageLength, messageOffset;

    boost::asio::ip::tcp::socket* _socket;
    boost::asio::io_service _ioService;

    std::string _hostName;

    bool _connected;

    static const unsigned int max_buf_len = 5000000;
    char buf[max_buf_len];

    GripperStatusAll _status;

    boost::uint8_t _packageIDCounter;

    std::map<boost::uint8_t, std::pair<ModbusPackage,bool> > _packagesIntransit;

    std::queue<ModbusPackage> _packagesOutgoing;
    std::queue<ModbusPackage> _packagesRecieved;

    long long _statusTimeStamp;
    rw::math::Q _currentQ, _currentSpeed, _currentForce;
    rw::math::Q _target, _speed, _force;

};


} //end namespace

#endif //#ifndef RWHW_URPRIMARYINTERFACE_HPP
