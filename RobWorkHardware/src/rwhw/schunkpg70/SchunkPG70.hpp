#ifndef RWHW_SCHUNKPG70_HPP
#define RWHW_SCHUNKPG70_HPP

// RW
#include <rw/math/Q.hpp>
#include <rwhw/PowerCube/Cube.hpp>
#include <rwhw/PowerCube/CubePort.hpp>


namespace rwhw {


// Gripper device controller class
class SchunkPG70  {
public:
	typedef rw::common::Ptr<SchunkPG70> Ptr;

	SchunkPG70();
	virtual ~SchunkPG70();
	bool connectSerial(const std::string& name);
	void disconnect();
	void home();
	void open();
	void close();
	void stop();
	//void retractGrippers();
	bool isConnected();
	bool getQ(rw::math::Q &q);
	bool setQ(const rw::math::Q& q);
	bool setGraspPowerPct(const double pct);
	bool status(unsigned int &status);

	void logTextReadySig(const std::string& text, const bool warning = false);

private:
	rwhw::SerialPort* _port;
	rwhw::CubePort* _cubePort;
	rwhw::Cube* _cube;
	bool _connected;
	double _graspCurrent;
	float _defMinPos, _defMaxPos, _defMaxDeltaVel;
	float _defTorqueRatio, _defCurRatio;
	float _defMinVel, _defMaxVel, _defMinAcc, _defMaxAcc;
	float _defMinCur, _defMaxCur;

	bool initialize(const std::string& name);

	// Defines
	static const float HOMEPOS;
	static const float VEL;
	static const float ACC;
	static const float MAXPOS;
	static const float MAXVEL;
	static const float MAXACC;
	static const float MAXCUR;


};

} //end namespace rwhw

#endif //end include guard
