/* */
/* */
#ifndef RWHW_URCALLBACKINTERFACE_HPP
#define RWHW_URCALLBACKINTERFACE_HPP

#include "URPrimaryInterface.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Wrench6D.hpp>
#include <rw/common/Ptr.hpp>

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <queue>

namespace rwhw {
class URCallBackInterface {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * @brief Which controller version of the UR to use
	 * 
	 * Check the controller box to see which version. Notice that not all commands are supported in all versions.
	 */
	enum ControllerBox { CB2 = 2, CB3 = 3};
	
	/**
	 * @brief Default constructor
	 */
	URCallBackInterface();


	/**
	 * @brief Connect to the UR
	 * @param host [in] IP address of the UR
	 * @param port [in] Port for connecting to the UR. Defaults to 30002.
 	 * @return True if successfully connected
 	 */
	void connect(const std::string& host, unsigned int port  = 30002);
	    
    /**
    * @brief Starts robot communication thread and sends a script to the controller.
    * 
    * Required to start robot communication. Call connect() first, then startCommunication().
    *  
    * @param host [in] IP address of the host (to which the robot will connect), e.g. 192.168.100.1.
    * @param callbackPort [in] port used for communicating with robot. Defaults to 33334.
	* @param version [in] Controller version to use. Defaults to CB2
    * @param filename [in] UR script filename; if not specified, a default bundled script is used.
    */
	void startCommunication(const std::string& callbackIP, const unsigned int callbackPort = 33334, ControllerBox cb = CB2, const std::string& filename="");

    /**
    * @brief Starts robot communication thread and sends a script to the controller.
    * 
    * Required to start robot communication. Call connect() first, then startCommunication().
	* Selects the local IP as the host for the call back. 
    * 
    * @param callbackPort [in] port used for communicating with robot. Defaults to 33334.
	* @param version [in] Controller version to use. Defaults to CB2
    * @param filename [in] UR script filename; if not specified, a default bundled script is used.
    */
	void startCommunication(const unsigned int callbackPort = 33334, ControllerBox cb = CB2, const std::string& filename = "");


    /**
    * @brief Stops the robot communication thread
    */
    void stopCommunication();



	/**	 
	 * @brief Returns the URPrimaryInterface used to send the script.
	 */
	URPrimaryInterface& getPrimaryInterface();

	/**
	 * @brief Stops the robot
	 */
	void stopRobot();

	/**
	 * @brief Move to the specified configuration
	 *
	 * @param transform [in] Target configuration of the robot
	 * @param speed [in] NOT USED
	 */
	void moveQ(const rw::math::Q& q, float speed);

	/**
	 * @brief Move to the specified transformation
	 *
	 * The coordinates are send directly to the robot, hence the inverse kinematics is the robots internal model.
	 * @param transform [in] Target of the robot
	 * @param speed [in] NOT USED
	 */
	void moveT(const rw::math::Transform3D<>& transform, float speed);

	/**
	 * @brief Servos to the specified configuration.
	 *
	 * In contrast to a move instruction, the servo target can be updated while the robot is moving. 
	 * WARNING: Giving a target far away may lead to very fast motions.
	 * @param q [in] Configuration to servo to.
	 */
	void servo(const rw::math::Q& q);


	/**
	 * @brief Returns timestamp of the driver
	 */
	double driverTime() const;

	/**
	 * @brief Returns true if the robot is moving
	 */
	bool isMoving() const;

	/**
	 * @brief Start force mode
	 * 
	 * Consult the force_mode command in the UR script manual for further details.
	 *
	 * @param base2ref [in] Specify the reference frame for the force
	 * @param selection [in] 6D vector containing 0 and 1, where one indicates that the robot will be compliant in that direction.
	 * @param wrench [in] 6D wrench to apply
	 * @param limits [in] For compliant axis the limit is the maximal tcp speed along/around the axis. For non-compliant axes it is the maximal deviation allowed for the TCP.
	 */ 
	void forceModeStart(const rw::math::Transform3D<>& base2ref, const rw::math::Q& selection, const rw::math::Wrench6D<>& wrench, const rw::math::Q& limits);

	/**
	 * @brief Update wrench to apply in force mode
	 */ 
	void forceModeUpdate(const rw::math::Wrench6D<>& wrench);


	/**
	 * @brief Ends force mode
	 */ 
	void forceModeEnd();


	/**
	 * @brief Start teach mode
	 * Only supported on CB3 and newer controllers
	 */
	void teachModeStart(); 

	/**
	 * @brief End teach mode
	 * Only supported on CB3 and newer controllers
	 */
	void teachModeEnd();

	/**
	 * @brief Sets Digital Output \b id to \b value
	 * @param id [in] Id of the output
	 * @param value [in] True means the Output is high
	 */
	void setDigitalOutput(int id, bool value);
	 
private:
    private:
    void run();

	URPrimaryInterface _urPrimary;
	ControllerBox _cb;
	rw::common::Ptr<boost::thread> _thread;


	unsigned int _callbackPort;
        boost::asio::ip::address _callbackIP;

	bool _stopServer;
	bool _robotStopped;

	bool _isMoving;
        bool _isServoing;

	class URScriptCommand {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		enum CmdType { STOP = 0, MOVEQ = 1, MOVET = 2, SERVOQ = 3, FORCE_MODE_START = 4, FORCE_MODE_UPDATE = 5, FORCE_MODE_END = 6, TEACH_MODE_START = 7, TEACH_MODE_END = 8, SET_DIGOUT = 9, DO_NOTHING = 9999 };

            URScriptCommand(CmdType type, const rw::math::Q& q, float speed):
                _type(type),
                _q(q),
                _speed(speed)
            {
            }

            URScriptCommand(CmdType type):
                _type(type)
            {
            }


            URScriptCommand(CmdType type, const rw::math::Transform3D<>& transform):
                _type(type),
                _transform(transform)
            {
            }

            URScriptCommand(CmdType type, const rw::math::Transform3D<>& transform, const rw::math::VelocityScrew6D<>& velocity):
                _type(type),
                _transform(transform)/*,
                _velocity(velocity)*/
            {}

            URScriptCommand(CmdType type, const rw::math::Q& selection, const rw::math::Wrench6D<>& wrench, const rw::math::Q& limits):
                _type(type),
                _selection(selection),
                _wrench(wrench),
                _limits(limits)
            {}

            URScriptCommand(CmdType type, const rw::math::Transform3D<>& base2ref, const rw::math::Q& selection, const rw::math::Wrench6D<>& wrench, const rw::math::Q& limits):
                _type(type),
                _transform(base2ref),
                _selection(selection),
                _wrench(wrench),
                _limits(limits)
            {}

            URScriptCommand(CmdType type, const rw::math::Wrench6D<>& wrench):
                _type(type),
                _wrench(wrench)
            {}

			URScriptCommand(CmdType type, int id, bool bValue):
				_type(type),
				_id(id),
				_bValue(bValue)
			{}

            CmdType _type;
            rw::math::Q _q;
            rw::math::Transform3D<> _transform;
            //rw::math::VelocityScrew6D<> _velocity;
            rw::math::Q _selection;
            rw::math::Wrench6D<> _wrench;
            rw::math::Q _limits;
            float _speed;
			int _id;
			bool _bValue;
	};

	std::queue<URScriptCommand> _commands;

	boost::mutex _mutex;

	void handleCmdRequest(boost::asio::ip::tcp::socket& socket);
        void sendStop(boost::asio::ip::tcp::socket& socket);

        void popAllUpdateCommands();


	/** Stuff needed for the servoing */
	rw::math::Q _qcurrent;
	/*rw::math::Q _qservo;
          rw::math::Q _dqservo;
          double _dt;*/
//	rwlibs::algorithms::XQPController::Ptr _xqp;

    };

} //end namespace rwhw

#endif //#ifndef RWHW_URCALLBACKINTERFACE_HPP
