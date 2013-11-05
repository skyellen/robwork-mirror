#ifndef FALCON_INTERFACE_HPP
#define FALCON_INTERFACE_HPP

#include <QObject>
#include <QtGui>
#include <QTimer>

#include <fstream>
#include <iostream>

#include <boost/lexical_cast.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/RPY.hpp>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"

#define MAX_GRASPING_VALUE			0.035
#define NO_MOVEMENT_RADIUS			0.01
#define NO_FORCE_RADIUS				0.001
#define DEFAULT_FORCE_COEFFICENT	-30
#define ROTATE_COEFFICIENT			60
#define MOVEMENT_COEFFICIENT		120
#define DIFF2_OFFSET				0.11
#define STRONG_FORCE_COEFFICENT 	-700
#define STRONG_FORCE_RADIUS			0.035
#define GRAVITY_FORCE				0.5

using namespace libnifalcon;
using namespace rw::math;
using namespace std;


class FalconInterface : public QThread {

private:
	FalconDevice *falcon;
	boost::array<double, 3>	pos, strongForce, defaultForce;
	Vector3D<> diff;
	Transform3D<> position;
	bool grasping;
	bool enableStrongForce;
	bool isInNMA;
	bool initializeFalcon();
	unsigned int _buttonState;
public:
	
	bool isThreadStopped;
	
	FalconInterface() 
	{
		grasping = false;
		enableStrongForce = false;
		initializeFalcon();
	}
	
	~FalconInterface()
	{
		delete falcon;
	}
	
	void run();
	
	bool getGrasping();
	
	Transform3D<> getPosition();
	
	void setStrongForce(bool);
	
	unsigned int getButtonStates();

};


#endif
