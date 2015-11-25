/* */
#ifndef RWHW_ROBOTIQ_HPP
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

    boost::uint8_t data[30];
};

/**
 * @brief Implements the generic parts of the robotiq hand interface
 * The Robotiq hands are an underactuated robot hands where all joints depends
 * on a few joints. The limits are therefore approximate
 *
 */
class Robotiq {

public:
    typedef rw::common::Ptr<Robotiq> Ptr;

    virtual ~Robotiq();

    /**
     * @brief connect to hand using
     * @param host
     * @param port
     * @return
     */
    bool connect(const std::string& host, unsigned int port = 502);
    bool isConnected() const {
        return _connected;
    }
    void disconnect();

    /**
     * @brief Command to get the current status from the hand and to store the result in
     * class variables
     */
    void getAllStatusCMD();

    /**
     * @brief tells if the hand thinks it is activated or not. Depends on the internal state
     * variables. Call getAllStatusCMD first.
     */
    virtual bool isActivated() = 0;

    /**
      * @brief tells if the hand thinks it is moving. Depends on the internal state
      * variables. Call getAllStatusCMD first.
      */
    virtual bool isGripperMoving() = 0;

    /**
      * @brief tells if the hand has been stopped by running into an obstacle before reaching its
      * target position since the last move call. Depends on the internal state
      * variables. Call getAllStatusCMD first.
      */
    virtual bool isGripperBlocked() = 0;

    /**
      * @brief tells if the hand has reached its target. Depends on the internal state
      * variables. Call getAllStatusCMD first.
      */
    virtual bool isGripperAtTarget() = 0;

    /**
     * @brief sends a move to \b target command to the hand
     * @param target [in] the target to move to
     */
    void moveCmd(rw::math::Q target);

    /**
     * @brief sends a move to \b target command to the hand
     */
    void moveCmd();

    /**
     * @brief stop movement of all gripper joints
     */
    void stopCmd();

    /**
     * @brief sets the wanted target for each joint. The target must be within
     * the position limits.
     * @param jointPos
     */
    void setTargetQ(const rw::math::Q& jointPos);

    /**
     * @brief sets the wanted target velocity (between 0 and 255) for each joint.
     * The velocity must be within the velocity limits.
     * @param jointVel
     */
    void setTargetQVel(const rw::math::Q& jointVel);

    /**
     * @brief sets the wanted target Force. The force must be within
     * the force limits.
     * @param jointAcc
     */
    void setTargetQForce(const rw::math::Q& jointCurr);

    /**
     * @brief gets the hands current target configuration. (This is based on the internal
     * representation not necessarily the actual target configuration that the hand has.)
     * @return
     */
    rw::math::Q getTargetQ();

    /**
     * @brief queries the hand for its joint configuration. Depends on the internal state
     * variables. Call getAllStatusCMD first.
     */
    rw::math::Q getQ();

    /**
     * @brief queries the hand for its current power use. Depends on the internal state
     * variables. Call getAllStatusCMD first.
     */
    rw::math::Q getQCurrent();

    /**
      * @brief returns the hands upper and lower configuration bounds.
      */
    virtual std::pair<rw::math::Q, rw::math::Q> getLimitPos() = 0;

    /**
      * @brief returns the hands upper and lower velocity bounds.
      */
    virtual std::pair<rw::math::Q, rw::math::Q> getLimitVel() = 0;

    /**
      * @brief returns the hands upper and lower force bounds.
      */
    virtual std::pair<rw::math::Q, rw::math::Q> getLimitForce() = 0;

    /**
      * @brief transforms the hands velocity value to a velocity measured in
      * m/s.
      */
    virtual double getVelocityInMetersPerSecFromTicks(int ticks) const = 0;

    /**
      * @brief transforms a velocity in m/s in a corresponding value that the hand
      * understands.
      * mm/s.
      */
    virtual int getTicksFromVelocityInMetersPerSec(double velocity) const = 0;

    /**
      * @brief transforms the hands force value into an approximate force measured in
      * Newton.
      */
    virtual double getApproximateForceInNewtonFromTicks(int ticks) const = 0;

    /**
      * @brief transforms a force in Newton into a value the hand understands.
      */
    virtual int getApproximateTicksFromForceInNewton(double force) const = 0;

    /**
      * @brief transforms the hand delivered current value into current measured in ampere.
      */
    double getCurrentInAmpereFromTicks(int ticks) const;

    /**
      * @brief Returns the number of joints the device has. Zero indicates a problem.
      */
    unsigned int getNumberOfJoints() const;

private:
    rw::common::Ptr<boost::thread> _thread;
    mutable boost::mutex _mutex;
    bool _stop;
    bool activate(unsigned int timeout = 0);
    void run();
    void start();
    void stop();

    bool _haveReceivedSize;

    boost::asio::ip::tcp::socket* _socket;
    boost::asio::io_service _ioService;

    std::string _hostName;

    bool _connected;

    static const unsigned int max_buf_len = 5000000;
    char buf[max_buf_len];

    boost::uint8_t _packageIDCounter;

    std::map<boost::uint8_t, std::pair<ModbusPackage, bool> > _packagesIntransit;

    std::queue<ModbusPackage> _packagesOutgoing;
    std::queue<ModbusPackage> _packagesRecieved;

protected:
    Robotiq(rw::math::Q currentQ, rw::math::Q currentCurrent, rw::math::Q target, rw::math::Q speed, rw::math::Q force, unsigned int numberOfJoints);

    ModbusPackage send(ModbusPackage package);

    virtual bool isGripperInReset() = 0;
    virtual bool isGripperInActivationProcess() = 0;

    virtual ModbusPackage getMoveCMDRequestPackage(const rw::math::Q & target) const = 0;
    virtual ModbusPackage getAllStatusCMDRequestPackage() const = 0;
    virtual void validateStatusPackageAndUpdateState(const ModbusPackage & package) = 0;
    virtual ModbusPackage getStopCMDRequestPackage() const = 0;
    virtual void validateStopCMDResponseMessage(const ModbusPackage & answer) const = 0;
    virtual ModbusPackage getActivateRequestPackage() const = 0;

    virtual bool handAfterActivationConnected() const;


    // Helper functions for big endian / little endian conversion for modbus
    void setReg(boost::uint8_t& reg, const boost::uint8_t& val) const;
    void setReg(boost::uint16_t& reg, const boost::uint16_t& val) const;
    void getReg(const boost::uint16_t& reg, boost::uint16_t& val) const;
    boost::uint8_t toVal8(const int val) const;

    rw::math::Q _currentQ, _currentCurrent;
    rw::math::Q _target, _speed, _force;

    static const boost::uint16_t FC04 = 0x04;
    static const boost::uint16_t FC16 = 0x10;

    unsigned int _numberOfJoints;

};

} //end namespace

#endif //#ifndef RWHW_ROBOTIQ_HPP
