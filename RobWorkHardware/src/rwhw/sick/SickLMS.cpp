/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "SickLMS.hpp"
#include <rw/common/macros.hpp>
#include <rw/common/TimerUtil.hpp>

using namespace rwhw;
using namespace rw::common;

namespace {
    typedef unsigned char uchar;

    const int MAX_RETRIES = 25;
    const int MAX_BUFFER_LENGTH = 802;

    //Packages to setup the resolution and range
    //Set to 100deg angular range and 1deg resolution
    const uchar LMS_SETUP1[] = { 0x02, 0x00, 0x05, 0x00, 0x3b, 0x64, 0x00,
                                 0x64, 0x00 };
    const uchar LMS_SETUP1_ACK[] = { 0x06, 0x02, 0x80, 0x07, 0x00, 0xbb, 0x01,
                                     0x64, 0x00, 0x64, 0x00, 0x10 };

    //Set to 100deg angular range and 0.5deg resolution
    const uchar LMS_SETUP2[] = { 0x02, 0x00, 0x05, 0x00, 0x3b, 0x64, 0x00,
                                 0x32, 0x00 };
    const uchar LMS_SETUP2_ACK[] = { 0x06, 0x02, 0x80, 0x07, 0x00, 0xbb, 0x01,
                                     0x64, 0x00, 0x32, 0x00, 0x10 };

    //Set to 100deg angular range and 0.25deg resolution
    const uchar LMS_SETUP3[] = { 0x02, 0x00, 0x05, 0x00, 0x3b, 0x64, 0x00,
                                 0x19, 0x00 };
    const uchar LMS_SETUP3_ACK[] = { 0x06, 0x02, 0x80, 0x07, 0x00, 0xbb, 0x01,
                                     0x64, 0x00, 0x19, 0x00, 0x10 };

    //Set to 180deg angular range and 1deg resolution
    const uchar LMS_SETUP4[] = { 0x02, 0x00, 0x05, 0x00, 0x3b, 0xb4, 0x00,
                                 0x64, 0x00 };
    const uchar LMS_SETUP4_ACK[] = { 0x06, 0x02, 0x80, 0x07, 0x00, 0xbb, 0x01,
                                     0xb4, 0x00, 0x64, 0x00, 0x10 };

    //Set to 180deg angular range and 0.5deg resolution
    const uchar LMS_SETUP5[] = { 0x02, 0x00, 0x05, 0x00, 0x3b, 0xb4, 0x00,
                                 0x32, 0x00 };
    const uchar LMS_SETUP5_ACK[] = { 0x06, 0x02, 0x80, 0x07, 0x00, 0xbb, 0x01,
                                     0xb4, 0x00, 0x32, 0x00, 0x10 };

    //Packages to set up the modes
    const uchar LMS_PASSWORD[] = { 0x02, 0x00, 0x0a, 0x00, 0x20, 0x00, 0x53,
                                   0x49, 0x43, 0x4b, 0x5f, 0x4c, 0x4d, 0x53 };

    const uchar LMS_SETUP_UNIT_MM[] = { 0x02, 0x00, 0x23, 0x00, 0x77, 0x00,
                                        0x00, 0x46, 0x00, 0x01, 0x00, 0x01,
                                        0x00, 0x00, 0x02, 0x02, 0x02, 0x00,
                                        0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00,
                                        0x0a, 0x0a, 0x50, 0x80, 0x00, 0x0a,
                                        0x0a, 0x50, 0x64, 0x00, 0x00, 0x00,
                                        0x00, 0x02, 0x00 };

    const uchar LMS_SETUP_UNIT_CM[] = { 0x02, 0x00, 0x23, 0x00, 0x77, 0x00,
                                        0x00, 0x46, 0x00, 0x01, 0x00, 0x00,
                                        0x00, 0x00, 0x02, 0x02, 0x02, 0x00,
                                        0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00,
                                        0x0a, 0x0a, 0x50, 0x80, 0x00, 0x0a,
                                        0x0a, 0x50, 0x64, 0x00, 0x00, 0x00,
                                        0x00, 0x02, 0x00 };

    const uchar
            LMS_SETUP_UNIT_MM_ACK[] = { 0x06, 0x02, 0x80, 0x25, 0x00, 0xf7,
                                        0x01, 0x00, 0x00, 0x46, 0x00, 0x01,
                                        0x00, 0x01, 0x00, 0x00, 0x02, 0x02,
                                        0x02, 0x00, 0x00, 0x0a, 0x0a, 0x50,
                                        0x64, 0x00, 0x0a, 0x0a, 0x50, 0x80,
                                        0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00,
                                        0x00, 0x00, 0x00, 0x02, 0x00, 0x10 };

    const uchar
            LMS_SETUP_UNIT_CM_ACK[] = { 0x06, 0x02, 0x80, 0x25, 0x00, 0xf7,
                                        0x01, 0x00, 0x00, 0x46, 0x00, 0x01,
                                        0x00, 0x00, 0x00, 0x00, 0x02, 0x02,
                                        0x02, 0x00, 0x00, 0x0a, 0x0a, 0x50,
                                        0x64, 0x00, 0x0a, 0x0a, 0x50, 0x80,
                                        0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00,
                                        0x00, 0x00, 0x00, 0x02, 0x00, 0x10 };

    //Status package
    const uchar LMS_STATUS[] = { 0x02, 0x00, 0x01, 0x00, 0x31 };
    //Start package
    const uchar LMS_START[] = { 0x02, 0x00, 0x02, 0x00, 0x20, 0x24 };
    //Stop package
    const uchar LMS_STOP[] = { 0x02, 0x00, 0x02, 0x00, 0x20, 0x25 };
    //Set Baud to 9600 package
    const uchar LMS_B9600[] = { 0x02, 0x00, 0x02, 0x00, 0x20, 0x42 };
    //Set Baud to 19200 package
    const uchar LMS_B19200[] = { 0x02, 0x00, 0x02, 0x00, 0x20, 0x41 };
    //Set Baud to 38400 package
    const uchar LMS_B38400[] = { 0x02, 0x00, 0x02, 0x00, 0x20, 0x40 };
    //Set Baud to 500000 package
    const uchar LMS_B500000[] = { 0x02, 0x00, 0x02, 0x00, 0x20, 0x48 };
    //Ack for commands
    const uchar LMS_CMD_ACK[] = { 0x06, 0x02, 0x80, 0x03, 0x00, 0xa0, 0x00,
                                  0x10 };

    // Calculates the CRC for packets sent to/from the LMS
    unsigned short calculateCRC(const uchar* data, int len)
    {
        unsigned char tmp1 = 0;
        unsigned char tmp2 = 0;
        unsigned short checksum = 0;
        for (int i = 0; i < len; i++) {
            tmp2 = tmp1;
            tmp1 = data[i];

            if (checksum & 0x8000) {
                checksum = (checksum & 0x7fff) << 1;
                checksum = checksum ^ 0x8005;
            } else {
                checksum = checksum << 1;
            }
            checksum = checksum ^ (tmp1 | (tmp2 << 8));
        }
        return checksum;
    }

    // Compares two messages
    bool compareMesseges(const uchar *s1, int len1, const uchar *s2, int len2)
    {
        if (len1 != len2) {
            RW_WARN("SickLMS: Message length does not match");
            return false;
        }

        for (int i = 0; i < len1; i++) {
            if (s1[i] != s2[i]) {
                RW_WARN("SickLMS: Message does not match");
                return false;
            }
        }
        unsigned short crcval = calculateCRC(s1, len1);
        if ((crcval & 0xff) != s2[len2]) {
            RW_WARN("SickLMS: CRC Sum did not match");
            return false;
        }
        if (((crcval >> 8) & 0xff) != s2[len2 + 1]) {
            RW_WARN("SickLMS: CRC Sum did not match");
            return false;
        }

        return true;
    }

}

// Writes a message to the LMS
void SickLMS::write(int len, const uchar *msg)
{
    int i;
    unsigned short crcval = calculateCRC(msg, len);
    uchar sendchar;

    for (i = 0; i < len; i++) {
        _port.write((const char*) (msg + i), 1);
    }

    sendchar = crcval & 0xff;
    _port.write((char*) &sendchar, 1);
    sendchar = (crcval >> 8) & 0xff;
    _port.write((char*) &sendchar, 1);
}

int SickLMS::read(int len, unsigned char *buf)
{
    if (_port.read((char*) buf, len, 500, 1))
        return len;
    else
        return 0;
}

uchar SickLMS::read()
{
    uchar buf;
    if (_port.read((char*) &buf, 1) == 0) {
        buf = 0;
    }
    return buf;

}

//Checks ack package
bool SickLMS::checkAck(const uchar *ackmsg, int len)
{
    int buflen;
    uchar buf[MAX_BUFFER_LENGTH];

    //Sleep a bit until data starts to arrive
    TimerUtil::sleepMs(100);
    for (int i = 0; i < MAX_RETRIES; i++) {
        if (read() == ackmsg[0]) {
            buflen = read(len + 1, buf);
            return compareMesseges(ackmsg + 1, len - 1, buf, buflen - 2);
        }
    }
    return false;
}

//Initializes LMS
bool SickLMS::initLMS(const std::string& port, BaudRate baud)
{
    if (!_port.open(port, SerialPort::Baud9600)) {
        RW_WARN("Unable to connect to SickLMS");
    }

    const uchar* msg;
    SerialPort::Baudrate baudrate = SerialPort::Baud9600;
    if (baud == Baud500000) { // step to the 500000bps mode
        msg = LMS_B500000;
        baudrate = SerialPort::Baud460800;
    } else if (baud == Baud38400) { // step to the 38400bps mode
        msg = LMS_B38400;
        baudrate = SerialPort::Baud38400;
    } else if (baud == Baud19200) { // step to the 19200bps mode
        msg = LMS_B19200;
        baudrate = SerialPort::Baud19200;
    } else {
        // Do nothing - goal is 9600 and already at 9600
        return true;
    }

    write(sizeof(LMS_B500000) / sizeof(uchar), msg);
    if (!checkAck(LMS_CMD_ACK, sizeof(LMS_CMD_ACK) / sizeof(uchar)))
        RW_WARN("SickLMS: Initialization failed - Unable to change baud rate");

    _port.close();
    TimerUtil::sleepMs(500);
    if (!_port.open(port, baudrate)) {
        RW_WARN("Unable to reopen port with new baud rate");
        return false;
    }

    return true;
}

// Sets the sweep width and resolution
bool SickLMS::setRangeRes(AngRange range, AngResolution res)
{
    const uchar *msg, *ackmsg;

    if (range == AngRange100 && res == AngRes1Deg) {
        msg = LMS_SETUP1;
        ackmsg = LMS_SETUP1_ACK;
    } else if (range == AngRange100 && res == AngRes05Deg) {
        msg = LMS_SETUP2;
        ackmsg = LMS_SETUP2_ACK;
    } else if (range == AngRange100 && res == AngRes025Deg) {
        msg = LMS_SETUP3;
        ackmsg = LMS_SETUP3_ACK;
    } else if (range == AngRange180 && res == AngRes1Deg) {
        msg = LMS_SETUP4;
        ackmsg = LMS_SETUP4_ACK;
    } else if (range == AngRange180 && res == AngRes05Deg) {
        msg = LMS_SETUP5;
        ackmsg = LMS_SETUP5_ACK;
    } else {
        RW_WARN("Invalid resolution selected. Using 100 degree view and 1 degree");
        msg = LMS_SETUP1;
        ackmsg = LMS_SETUP1_ACK;
    }

    write(sizeof(LMS_SETUP1) / sizeof(uchar), msg);
    if (!checkAck(ackmsg, sizeof(LMS_SETUP1_ACK) / sizeof(char))) {
        RW_WARN("Unable to setup resolution");
        return false;
    }
    return true;
}

// Selects unit mode
// This might not work on a LMS291
bool SickLMS::setUnits(int unit)
{
    //Start by sending the password needed to change the units
    write(sizeof(LMS_PASSWORD) / sizeof(char), LMS_PASSWORD);
    if (!checkAck(LMS_CMD_ACK, sizeof(LMS_CMD_ACK) / sizeof(char))) {
        RW_WARN("Unable to send password");
        return false;
    }

    const uchar* msg;
    const uchar* ackmsg;

    if (unit == UnitMM) {
        msg = LMS_SETUP_UNIT_MM;
        ackmsg = LMS_SETUP_UNIT_MM_ACK;
    } else if (unit == UnitCM) {
        msg = LMS_SETUP_UNIT_CM;
        ackmsg = LMS_SETUP_UNIT_CM_ACK;
    } else {
        RW_WARN("Invalid units specified");
        return false;
    }

    // the following two line works only because msg & ackmsg are const uchar str
    write(sizeof(LMS_SETUP_UNIT_MM) / sizeof(uchar), msg);
    if (!checkAck(ackmsg, sizeof(LMS_SETUP_UNIT_MM_ACK) / sizeof(uchar))) {
        RW_WARN("Failed to setup unit mode");
        return false;
    }
    return true;
}

//Starts the scanner
bool SickLMS::startLMS()
{
    write(sizeof(LMS_START) / sizeof(char), LMS_START);
    if (!checkAck(LMS_CMD_ACK, sizeof(LMS_CMD_ACK) / sizeof(char))) {
        RW_WARN("Failed to start SickLMS");
        return false;
    }
    return true;
}

//Stops the LMS
void SickLMS::stopLMS()
{
    write(sizeof(LMS_STOP) / sizeof(uchar), LMS_STOP);
    if (!checkAck(LMS_CMD_ACK, sizeof(LMS_CMD_ACK) / sizeof(uchar)))
        RW_WARN("Failed to stop SickLMS");
}

//Resets LMS to default
void SickLMS::resetLMS()
{
    write(sizeof(LMS_B9600) / sizeof(uchar), LMS_B9600);
    if (!checkAck(LMS_CMD_ACK, sizeof(LMS_CMD_ACK) / sizeof(uchar)))
        RW_WARN("Failed to reset SickLMS to default baud rate");

}

SickLMS::SickLMS()
{

}

SickLMS::~SickLMS()
{
    stopLMS();
    resetLMS();
}

//bool SickLMS::connectToLMS(int range_mode, int angResMode, int unitMode, char * port, int baud_sel)

bool SickLMS::connectToLMS(AngRange angRange, AngResolution angResolution,
                           Units units, const std::string& port, BaudRate baud)
{

    bool success = true;
    const int MAX_TRIES = 20;

    _units = units;
    _angResolution = angResolution;

    for (int attempt = 0; attempt < MAX_TRIES; attempt++) {

        // Set the baud rate of the PC and the LMS
        success = initLMS(port, baud);

        //Set the range and resolution
        success &= setRangeRes(angRange, angResolution);

        //Send password to allow setting up the units
        success &= setUnits(units);

        //Start the scanner
        success &= startLMS();

        if (success) {
            // LMS is working
            return true;
        } else {
            // Reset before retrying
            stopLMS();
            resetLMS();
            RW_WARN("Failed to connect to SickLMS - retries");
        }
    }
    RW_THROW("Failed to connect to SickLMS");
}

bool SickLMS::acquire()
{
    unsigned short crc = 0;
    int datalen;
    _buf[0] = 0;

    while (_buf[0] != 0x02) {
        _buf[0] = read();
    }

    _buf[1] = read(); // ADR byte, ADR=0X80 here
    _buf[2] = read(); // package length low byte
    _buf[3] = read(); // package length high byte
    _buf[4] = read(); // CMD byte, CMD=0xb0 in this mode
    _buf[5] = read(); // samples low byte
    _buf[6] = read(); // samples high byte

    // only lower 12 bits of high byte are significant
    datalen = _buf[5] | ((_buf[6] & 0x1f) << 8);
    datalen = datalen * 2;

    // Check that we have a valid ADR byte, valid data length and valid CMD byte
    if ((datalen > MAX_BUFFER_LENGTH) || (0x80 != _buf[1]) || (0xb0 != _buf[4])) {
        return 0;
    }

    read(datalen, _buf + 7);

    _buf[datalen + 7] = read(); // Status byte
    _buf[datalen + 8] = read(); // CRC low byte
    _buf[datalen + 9] = read(); // CRC high byte
    crc = _buf[datalen + 8] | (_buf[datalen + 9] << 8);

    int lenBytes = datalen + 8;
    unsigned short checksum = calculateCRC(_buf, lenBytes);

    if (checksum != crc) {
        RW_WARN("CRC-Error in communication with SickLMS");
        return false;
    }
    _cnt = datalen / 2.0;
    return true;
}

int SickLMS::getDataCount()
{
    return _cnt;
}

float SickLMS::getAngle(int i)
{
    switch (_angResolution)
    {
    case AngRes1Deg:
        return i - 90.0f;
    case AngRes05Deg:
        return i / 2.0f - 90.0f;
    case AngRes025Deg:
        return i / 4.0f - 90.0f;
    }
    return 0;
}

float SickLMS::getDistance(int i)
{
    int index = 2 * i + 7;
    if (_units == UnitMM)
        return (float) ((_buf[index + 1] & 0x1f) << 8 | _buf[index]) / 1000.0;
    else
        return (float) ((_buf[index + 1] & 0x1f) << 8 | _buf[index]) / 100.0;

}
