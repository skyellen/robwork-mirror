
#include "Trakstar.hpp"
#include <iostream>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "RWATC3DG.hpp"

#include <rw/rw.hpp>

using namespace std;
using namespace rw::common;
using namespace rwhw;

/*

THIS IS THE DATATYPE WE USE FOR COMMUNICATION

typedef struct tagDOUBLE_POSITION_QUATERNION_TIME_Q_BUTTON_RECORD
{
    double  x;
    double  y;
    double  z;
    double  q[4];
    double  time;
    USHORT  quality;
    USHORT  button;
} DOUBLE_POSITION_QUATERNION_TIME_Q_BUTTON_RECORD;
*/

#define NUMBER_OF_TRAKSTAR_SENSORS          4

#define TRAKSTAR_DATA_RECORD_SIZE           8

#define TRAKSTAR_DATA_RECORD_X_IDX          0
#define TRAKSTAR_DATA_RECORD_Y_IDX          1
#define TRAKSTAR_DATA_RECORD_Z_IDX          2
//#define 3, 4, 5 (rotation)
#define TRAKSTAR_DATA_RECORD_TIME_IDX       6
#define TRAKSTAR_DATA_RECORD_STATUS_IDX     7

namespace {


    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    //
    //  MACROS for simplifying the procedure calls
    //
    // This macro will set a system parameter and call the error handler if there is
    // an error reported. Note These macros do not print to the standard output the
    // set value

    #define SET_SYSTEM_PARAMETER(type, value, l)                                    \
        {                                                                           \
            type##_TYPE buffer = value;                                             \
            _errorCode = SetSystemParameter(type, &buffer, sizeof(buffer));         \
            if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, l);         \
        }

    #define SET_SENSOR_PARAMETER(sensor, type, value, l)                            \
        {                                                                           \
            type##_TYPE buffer = value;                                             \
            type##_TYPE *pBuffer = &buffer;                                         \
            _errorCode = SetSensorParameter(sensor, type, pBuffer, sizeof(buffer)); \
            if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, l);         \
        }

    #define SET_TRANSMITTER_PARAMETER(xmtr, type, value, l)                         \
        {                                                                           \
            type##_TYPE buf = value;                                                \
            type##_TYPE *pBuf = &buf;                                               \
            _errorCode = SetTransmitterParameter(xmtr, type, pBuf, sizeof(buf));        \
            if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, l);         \
        }

    // In order for the above macros to compile without error it is necessary
    // to provide typedefs for all the XXX_TYPEs that are generated by "type##_TYPE"
    typedef short                   SELECT_TRANSMITTER_TYPE;
    typedef double                  POWER_LINE_FREQUENCY_TYPE;
    // AGC_MODE_TYPE already defined as an enumerated type
    typedef double                  MEASUREMENT_RATE_TYPE;
    typedef short                   REPORT_RATE_TYPE;
    typedef double                  MAXIMUM_RANGE_TYPE;
    typedef BOOL                    METRIC_TYPE;
    // DATA_FORMAT_TYPE already defined as an enumerated type
    typedef DOUBLE_ANGLES_RECORD    ANGLE_ALIGN_TYPE;
    typedef DOUBLE_ANGLES_RECORD    REFERENCE_FRAME_TYPE;
    typedef BOOL                    XYZ_REFERENCE_FRAME_TYPE;
    // HEMISPHERE_TYPE already defined as an enumerated type
    typedef BOOL                    FILTER_AC_WIDE_NOTCH_TYPE;
    typedef BOOL                    FILTER_AC_NARROW_NOTCH_TYPE;
    typedef double                  FILTER_DC_ADAPTIVE_TYPE;
    typedef ADAPTIVE_PARAMETERS     FILTER_ALPHA_PARAMETERS_TYPE;
    typedef BOOL                    FILTER_LARGE_CHANGE_TYPE;
    typedef QUALITY_PARAMETERS      QUALITY_TYPE;

}



// forward declaration of error handler
void errorHandler(int error, int lineNum);

Trakstar::Trakstar() {
	
	_flagInitBird = false;
	_initStatus = -1;
	
	
	// Initialize system by calling _InitializeBird in a thread
	_initThread = boost::thread(boost::bind(&Trakstar::initializeBird, this));

	// Start polling thread
	_pollThread = boost::thread(boost::bind(&Trakstar::pollData, this));
	_flagStopPoll = true;
}

Trakstar::~Trakstar() {
	_id = -1;
	SetSystemParameter(SELECT_TRANSMITTER, &_id, sizeof(_id));
	if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, __LINE__);
	
	cout << "Trakstar was deconstructed!" << endl;
	CloseBIRDSystem();
}

void Trakstar::initialize() {
	if (!_flagInitBird) 
	{
		// Aquire mutex
		boost::mutex::scoped_lock lock(_mutexInitBird);
		
		// Set flag
		_flagInitBird = true;
		
		// Signal condition to _InitializeBird()
		_conditionInitBird.notify_one();
	}
}

/** 
 * Initialize System by calling library function InitializeBIRDSystem.
 * Should run on plugin startup and thus start in a thread so it doesn't increase startup time
 */
void Trakstar::initializeBird()
{
	int initStatusLocal = -1;
	while (1) {
				
		// Aquire mutex
		boost::mutex::scoped_lock lock(_mutexInitBird);

		// Stop this thread until the condition is signalled from another thread (InitializeSystem())
		_conditionInitBird.wait(lock);
		
		if (_flagInitBird)
		{
			// Set initStatusLocal to "initializing"
			initStatusLocal = 0;
			
			// Update _initStatus immediately
			_initStatus = initStatusLocal;
			
			// Initialize Bird system
			Log::log().info() << "Initializing Trakstar System... This takes some seconds." << endl;
			_errorCode = InitializeBIRDSystem();
			if(_errorCode!=BIRD_ERROR_SUCCESS) {
				errorHandler(_errorCode, __LINE__);
				initStatusLocal = -1; // Failed. Not initialized
			} else
			{
				Log::log().info() << "Initialization successfull." << endl;

				// Set parameters
				// Measurement rate, 80 is standard
				//SET_SYSTEM_PARAMETER( MEASUREMENT_RATE, 100, __LINE__ );

				// Metric (use millimeters)
				SET_SYSTEM_PARAMETER( METRIC, true, __LINE__ );

				// Report Rate (how fast does the box prepare new data). reportRate / ( 3*measure_rate ). eg. 120 / (3*80) = 0.500seconds per update.
				// Similarly 1 / (3*80) = 4ms per update or 240hz
//				SET_SYSTEM_PARAMETER( REPORT_RATE, 1, __LINE__ );
			
				// Get configuration (read from system into m_config struct)
				_errorCode = GetBIRDSystemConfiguration(&_ATC3DG);
				if(_errorCode!=BIRD_ERROR_SUCCESS) {
					errorHandler(_errorCode, __LINE__);
					initStatusLocal = -1; // Failed. Not initialized
				} else
				{
					std::cout << "Trakstar system configuration information read." << endl;
					std::cout << "Number Boards          = " << _ATC3DG.numberBoards << endl;
					std::cout << "Number Sensors         = " << _ATC3DG.numberSensors << endl;
					std::cout << "Number Transmitters    = " << _ATC3DG.numberTransmitters << endl << endl;

					std::cout << "System AGC mode	       = " << _ATC3DG.agcMode << endl;
					std::cout << "Maximum Range          = " << _ATC3DG.maximumRange << endl;
					std::cout << "Measurement Rate       = " << _ATC3DG.measurementRate << endl;
					std::cout << "Metric Mode            = " << _ATC3DG.metric << endl;
					std::cout << "Line Frequency         = " << _ATC3DG.powerLineFrequency << endl;
					std::cout << "Transmitter ID Running = " << _ATC3DG.transmitterIDRunning << endl;

					// we now know the number of sensors so we setup _record
					_rawValues.resize(_ATC3DG.numberSensors);
					_record.resize(_ATC3DG.numberSensors);
					_recordTmp.resize(_ATC3DG.numberSensors);

					//////////////////////////////////////////////////////////////////////////////
					//////////////////////////////////////////////////////////////////////////////
					//////////////////////////////////////////////////////////////////////////////
					//
					// GET TRANSMITTER CONFIGURATION
					//
					// The call to GetTransmitterConfiguration() performs a similar task to the 
					// GetSensorConfiguration() call. It also returns a status in the filled
					// structure which indicates whether a transmitter is attached to this
					// port or not. In a single transmitter system it is only necessary to 
					// find where that transmitter is in order to turn it on and use it.
					//

					_pXmtr.resize(_ATC3DG.numberTransmitters);
					for(int i=0;i<_ATC3DG.numberTransmitters;i++)
					{
						_errorCode = GetTransmitterConfiguration(i, &_pXmtr[i]);
						if(_errorCode!=BIRD_ERROR_SUCCESS) {
							errorHandler(_errorCode, __LINE__);
							initStatusLocal = -1; // Failed. Not initialized
						} else {
							
							// We have successfully initialized the system.
							initStatusLocal = 1;
						}
					}
					
					DOUBLE_ANGLES_RECORD anglesRecord = {0, 0, 180};

					SET_TRANSMITTER_PARAMETER(0, REFERENCE_FRAME, anglesRecord, __LINE__);
					SET_TRANSMITTER_PARAMETER(0, XYZ_REFERENCE_FRAME, true, __LINE__);
					
					// Set Sensors to FRONT Hemisphere
					for(int i = 0; i < _ATC3DG.numberSensors ; i++)
					{
						SET_SENSOR_PARAMETER(i, HEMISPHERE, FRONT, __LINE__);
/*						HEMISPHERE_TYPE buffer;
						_errorCode = GetSensorParameter(i, HEMISPHERE, &buffer, sizeof(buffer));
						if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, __LINE__);
						cout << buffer << endl;*/
					}
				}
			}
			
			_initStatus = initStatusLocal;
			_flagInitBird = false;
			
		} // if (_flagInitBird)
		
		
	} // while(1)
}

int Trakstar::getInitStatus() {
	return _initStatus;
}

int Trakstar::numberSensorsAttached() {
	if (_initStatus)
		return _sensorsAttached;
	else 
		return -1;
}

void Trakstar::startPolling() {

	// Search for transmitters. Turn the first one on. (there is only one)
	for(_id=0; _id < _ATC3DG.numberTransmitters; _id++)
	{
		if(_pXmtr[_id].attached)
		{
			// Transmitter selection is a system function.
			// Using the SELECT_TRANSMITTER parameter we send the id of the
			// transmitter that we want to run with the SetSystemParameter() call
			_errorCode = SetSystemParameter(SELECT_TRANSMITTER, &_id, sizeof(_id));
			if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, __LINE__);
			break;
		}
	}
		
	// Set the data format type for each attached sensor.
	for(int i = 0; i < _ATC3DG.numberSensors ; i++)
	{
		DATA_FORMAT_TYPE type = DOUBLE_POSITION_QUATERNION_TIME_Q_BUTTON;
		_errorCode = SetSensorParameter(i, DATA_FORMAT, &type, sizeof(type));
		if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, __LINE__);
	}
	
	// Count how many sensors are attached by fetching a single record from all and reading status
	_errorCode = GetSynchronousRecord(ALL_SENSORS, &_rawValues[0], sizeof(_rawValues[0]));
	if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, __LINE__);
	

	// Read sensorStatus
	_sensorsAttached = 0;
	for(int i = 0; i < _ATC3DG.numberSensors ; i++)
	{
		unsigned int status = GetSensorStatus(i);
		if (status == VALID_STATUS)
		{
			_sensorsAttached++;
		}
	}
	
	_flagStopPoll = false;
	
}

void Trakstar::pollData() {

	while (1) {
		
		if (!_flagStopPoll ) {

			// Sleep to enable other threads to read data.
			boost::this_thread::sleep(boost::posix_time::milliseconds(2));
		
			// scan the sensors (all)
			_errorCode = GetSynchronousRecord(ALL_SENSORS, &_rawValues[0], sizeof(_rawValues[0]));
			if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, __LINE__);
			
			// Get status of sensors (only updates after a Get***Record call)
			for (_sensorID = 0; _sensorID < _ATC3DG.numberSensors; _sensorID++) {
			
				// get the status of the last data record
				// only report the data if everything is okay
				unsigned int status = GetSensorStatus( _sensorID);

				// Set default state
				_recordTmp[_sensorID].status = status;
				_recordTmp[_sensorID].valid = false;

				if( status == VALID_STATUS)
				{
					_analogButtonOn = (bool)(_rawValues[_sensorID].button);
					// copy everything into record

					_recordTmp[_sensorID].pos[0] = _rawValues[_sensorID].x;
					_recordTmp[_sensorID].pos[1] = _rawValues[_sensorID].y;
					_recordTmp[_sensorID].pos[2] = _rawValues[_sensorID].z;
					_recordTmp[_sensorID].rot(0) = _rawValues[_sensorID].q[0];
					_recordTmp[_sensorID].rot(1) = _rawValues[_sensorID].q[1];
					_recordTmp[_sensorID].rot(2) = _rawValues[_sensorID].q[2];
					_recordTmp[_sensorID].rot(3) = _rawValues[_sensorID].q[3];
					_recordTmp[_sensorID].time = _rawValues[_sensorID].time;
					_recordTmp[_sensorID].quality = _rawValues[_sensorID].quality* 1.0/65536.0;
					_recordTmp[_sensorID].valid = true;
					
				} else if ( status == (SATURATED|GLOBAL_ERROR) ) {
					cout << "Sensor[" << _sensorID << "] is saturated." << endl;
				} else if ( status == (OUT_OF_MOTIONBOX|GLOBAL_ERROR) ) {
					cout << "Sensor[" << _sensorID << "] is out of range." << endl;
				} else if ( status == (NOT_ATTACHED|GLOBAL_ERROR) ) {
					// Don't tell us that a sensor is not attached. We probably(hopefully!) know
					// This would be a place to debug for sensor-data not read if that error present.
					//cout << "Not attached sensor" << endl;
					
				} else if ( status == (NO_TRANSMITTER_RUNNING|GLOBAL_ERROR) ) {
					//cout << "Transmitter not ready." << endl;					
				} else {
					cout << "Sensor[" << _sensorID << "]: status not valid: " << status-1 << endl;	
				}
			}
			
			// Mutex is scoped and is thus released automatically.
			{
                // Get mutex so data can be updated
                boost::mutex::scoped_lock lock(_mutexSensorValues);
                _record = _recordTmp;
			}
		} else {

			boost::this_thread::sleep(boost::posix_time::milliseconds(100));			
		}
	}
	
}


void Trakstar::stopPolling() {
		
	// Signal to _poll to stop polling
	_flagStopPoll = true;
	
	// Get active transmitter
	short buffer, *pBuffer = &buffer;
	_errorCode = GetSystemParameter(SELECT_TRANSMITTER, pBuffer, sizeof(buffer));
	if(_errorCode!=BIRD_ERROR_SUCCESS) {
		errorHandler(_errorCode, __LINE__);
		return;
	}
	
	// Stop transmitter if one was turned on
	if (buffer != -1)
	{
		_id = -1;
		_errorCode = SetSystemParameter(SELECT_TRANSMITTER, &_id, sizeof(_id));
		if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, __LINE__);
	}
}

std::vector<rwhw::Trakstar::PoseData> Trakstar::getSensorValues() {

	// Wait until we can get mutex
	boost::mutex::scoped_lock lock(_mutexSensorValues);
	// Mutex is scoped and is thus released automatically.
	return _record;
}


std::string Trakstar::getSensorStatusString(int errorCode)
{
	if (errorCode == VALID_STATUS)
	{
		return "OK";
	
	} else if (errorCode & NOT_ATTACHED) 
	{
		return "NOT CONNECTED";
	
	} else if (errorCode & OUT_OF_MOTIONBOX)
	{
		return "RANGE";
	
	} else if (errorCode & SATURATED)
	{
		return "SATURATED";
	}
	
	// else	
	return "ERROR";
}


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// This is a simplified error handler.
// This error handler takes the error code and passes it to the GetErrorText()
// procedure along with a buffer to place an error message string.
// This error message string can then be output to a user display device
// like the console
// Specific error codes should be parsed depending on the application.
//
void errorHandler(int error, int lineNum)
{
	char			buffer[1024];
	int				currentError = error;
	int				nextError;
	

	do{
		nextError = GetErrorText(currentError, buffer, sizeof(buffer), SIMPLE_MESSAGE);
		Log::log().info() << buffer << endl;
		currentError = nextError;
	}while(currentError!=BIRD_ERROR_SUCCESS);

	//exit(0);
}
