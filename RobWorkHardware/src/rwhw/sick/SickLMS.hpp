#ifndef RWHW_SICKLMS_HPP
#define RWHW_SICKLMS_HPP

#include <rwhw/serialport/SerialPort.hpp>


namespace rwhw {

/** @addtogroup rwhw */
/*@{*/


/**
 * @brief Driver to SICK LMS200/LMS291
 *
 * The SickLMS enables connecting to a SICK LMS200 or LMS291 using a serial connection
 */
class SickLMS {
public:
    /**
     * @brief Constructs SickLMS
     */
    SickLMS();

    /**
     * @brief Destructor calling stop and reset on the scanner
     */
    ~SickLMS();

    /**
     * @brief Baud rates to communication
     */
    enum BaudRate { Baud9600 = 0, Baud19200, Baud38400, Baud500000 };

    /**
     * @brief Angular range of the scanner
     */
    enum AngRange { AngRange100 = 0, AngRange180 };

    /**
     * @brief Angular resolution.
     *
     * Notice that 0.25 deg is only allow when running with a angular range of 100degrees
     */
    enum AngResolution { AngRes1Deg = 0, AngRes05Deg, AngRes025Deg };

    /**
     * @brief The units of distance
     */
    enum Units{ UnitMM = 0, UnitCM};

    /**
     * @brief Connects to Sick LMS
     *
     * Connects with the parameters specified
     * @param range [in] Angular range
     * @param angRes [in] Angular resolution
     * @param units [in] Unit of distance
     * @param port [in] The communication port. On linux typically /dev/ttyS? and on Windows COM?
     * @param baud [in] Baud rate for the communication
     */
    bool connectToLMS(AngRange range, AngResolution angRes, Units units, const std::string& port, BaudRate baud);

    /**
     * @brief Acquire a new scan
     * @return Whether a new scan was successfully acquired
     */
    bool acquire();

    /**
     * @brief Sends stop command to the LMS
     */
    void stopLMS();

    /**
     * @brief Resets LMS to default rate of 9600 baud
     */
    void resetLMS();

    /**
     * @brief Returns the number of measurements
     */
    int getDataCount();

    /**
     * @brief Returns angle for the \b i'th measurement
     * @param i [in] Measurement index. i \in [0;getDataCount[
     * @return Angle in radians
     */
    float getAngle(int i);

    /**
     * @brief Returns distance to the \b i'th measurement
     * @param i [in] Measurement index. i \in [0;getDataCount[
     * @return Distance in meters (independent on unit specified when connecting)
     */
    float getDistance(int i);
private:
    rwhw::SerialPort _port;

    //Maximal amount of data for the SICK is 802
    unsigned char _buf[802];

    //Number of measurements
    int _cnt;

    Units _units;

    AngResolution _angResolution;


    typedef unsigned char uchar;

    void write(int len, const uchar *msg);
    int read(int len, uchar *buf);
    uchar read();

    bool checkAck(const uchar *ackmsg, int ackmsglen);
    bool initLMS(const std::string& port, BaudRate baud);
    bool setRangeRes(AngRange range, AngResolution res);
    bool setUnits(int unit);
    bool startLMS();

};

 /** @} */

} //end namespace rwhw

#endif /*RWHW_SICKLMS_HPP*/
