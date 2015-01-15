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
	URCallBackInterface();

	bool connect(const std::string& host, unsigned int port);
	
	/**
	 * @brief Starts robot interface thread and sends a script to the controller.
	 * 
	 * @deprecated Use startCommunication() instead.
	 * 
	 * Uses default host (192.168.100.1).
	 * 
	 * @param callbackPort [in] port used for communicating with robot, e.g. 33334.
	 * @param filename [in] UR script filename; if not specified, a default bundled script is used.
	 */
        void startInterface(const unsigned int callbackPort, const std::string& filename="");
    
        /**
         * @brief Starts robot communication thread and sends a script to the controller.
         * 
         * Required to start robot communication. Call connect() first, then startCommunication().
         * 
         * @param host [in] IP address of the host (to which the robot will connect), e.g. 192.168.100.1.
         * @param callbackPort [in] port used for communicating with robot, e.g. 33334.
         * @param filename [in] UR script filename; if not specified, a default bundled script is used.
         */
        void startCommunication(const std::string& callbackIP, const unsigned int callbackPort, const std::string& filename="");

        /**
         * @brief Stops the robot interface thread
         *
         * @deprecated Use stopCommunication() instead
         */
        void stopInterface();

        /**
         * @brief Stops the robot communication thread
         */
        void stopCommunication();



	URPrimaryInterface& getPrimaryInterface();

	void stopRobot();

	void moveQ(const rw::math::Q& q, float speed);

	void moveT(const rw::math::Transform3D<>& transform, float speed);

	void servo(const rw::math::Q& dq);

//	void servo(const rw::math::Transform3D<>& target);

	double driverTime() const;

	bool isMoving() const;

	void forceModeStart(const rw::math::Transform3D<>& base2ref, const rw::math::Q& selection, const rw::math::Wrench6D<>& wrench, const rw::math::Q& limits);

	void forceModeUpdate(const rw::math::Wrench6D<>& wrench);

	void forceModeEnd();

    private:
    void run();

	URPrimaryInterface _urPrimary;

	rw::common::Ptr<boost::thread> _thread;


	unsigned int _callbackPort;
        boost::asio::ip::address _callbackIP;

	bool _stopServer;
	bool _robotStopped;

	bool _isMoving;
        bool _isServoing;

	class URScriptCommand {
	public:

            enum CmdType { STOP = 0, MOVEQ = 1, MOVET = 2, SERVOQ = 3, FORCE_MODE_START = 4, FORCE_MODE_UPDATE = 5, FORCE_MODE_END = 6, DO_NOTHING = 9999 };

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
                _transform(transform),
                _velocity(velocity)
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


            CmdType _type;
            rw::math::Q _q;
            rw::math::Transform3D<> _transform;
            rw::math::VelocityScrew6D<> _velocity;
            rw::math::Q _selection;
            rw::math::Wrench6D<> _wrench;
            rw::math::Q _limits;
            float _speed;
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
