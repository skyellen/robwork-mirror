#ifndef SchunkPG70_HPP_
#define SchunkPG70_HPP_
// RW
#include <rw/math/Q.hpp>
#include <rwhw/PowerCube/Cube.hpp>
#include <rwhw/PowerCube/CubePort.hpp>




// Gripper device controller class
class PG70Controller {
public:
	typedef rw::common::Ptr<PG70Controller> Ptr;

	PG70Controller();
	virtual ~PG70Controller();
	bool connect(const std::string& name);
	void disconnect();
	void goHome();
	void applyGrasp();
	void stopGrasp();
	void retractGrippers();
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
	float _defMinCur, _defMacCur;

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

#endif
