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


/**
 * @brief The wrapper class for the Novint Falcon 3D controller device.
 */
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
	
	/// Constructor
	FalconInterface() 
	{
		grasping = false;
		enableStrongForce = false;
		initializeFalcon();
	}
	
	/// Destructor
	~FalconInterface()
	{
		delete falcon;
	}
	
	/// Starts the I/O loop
	void run();
	
	/// Returns @b true if the middle button was pressed
	bool getGrasping();
	
	/**
	 * @brief Returns the position given by the controller
	 * 
	 * Depending on the button state, (...)
	 */
	Transform3D<> getPosition();
	
	/**
	 * @brief Sets whether the controler uses @a strong force
	 * 
	 * The @a strong force is used to give a stronger haptic feedback
	 * when user moves the controller out of the resting area.
	 */ 
	void setStrongForce(bool);
	
	/// Returns raw button register state
	unsigned int getButtonStates();

};


#endif
