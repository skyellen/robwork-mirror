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

#ifndef RWHW_DSACON32_HPP
#define RWHW_DSACON32_HPP

#include <rwhw/serialport/SerialPort.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include "ConvertUtil.hpp"
#include "TactileMatrix.hpp"
#include "TactileMaskMatrix.hpp"
#include <rw/common/TimerUtil.hpp>
#include <rw/common/macros.hpp>

namespace rwhw {

    /** @addtogroup tactile */
    /* @{ */


    /**
     * @brief Interface for the DSACON.
     */
    class DSACON32
    {
    public:
        struct SensorConfig;
        struct ControllerConfig;
        struct MatrixConfig;
    private:
        DSACON32( rwhw::SerialPort &port,
                  const ControllerConfig &cConfig,
                  const SensorConfig &sConfig,
                  MatrixConfig *mConfigs,
                  const std::vector<TactileMaskMatrix> &staticMasks);

        virtual ~DSACON32();


    public:

        struct ControllerConfig
        {
        public:
            enum SensconTypes { DSACON16 = 0, DSACON32 = 1, DSA100_256 = 2 };

            enum InterfaceTypes { RS232 = 0, CAN = 1, USB = 2 };

            int serialNumber;
            unsigned char hwRevision;
            int swBuild;
            unsigned char statusFlags;
            unsigned char featureFlags;
            SensconTypes sensconType;
            InterfaceTypes interfaceType;
            int canBaudrate;
            int canID;

            void parse(unsigned char payload[]);
            std::string toString() const;

            bool isDataAcqRunning(){
                return ((statusFlags & 64) == 64);
            }
        };

        struct SensorConfig
        {
        public:
            unsigned char hwRevision;
            int serialNumber;
            unsigned char featureFlags;
            unsigned short numMatrices;
            unsigned short firmwareRevision;

        public:

            SensorConfig() {}

            void parse(unsigned char payload[]);
            std::string toString() const;
        };

        struct MatrixConfig
        {
        public:
            float texelWidth, texelHeight;
            int cellsX, cellsY;
            unsigned short uid[3];
            unsigned char hwRevision;
            float centerX, centerY, centerZ;
            float thetaX, thetaY, thetaZ;
            int fullscale;
            unsigned char featureFlags;
            std::string descrStr;

            void parse(unsigned char payload[]);
            std::string toString() const;
        };

        static DSACON32* GetInstance(rwhw::SerialPort &port);

        //************ And now the member functions
        /**
         * @brief
         * @return false when acknowledge was not recieved, true otherwise
         */
        bool startDataAcquisition();
        bool stopDataAcquisition();
        bool queryControllerState();
        bool queryLoopTest();
        bool readThreadFunc();
        std::string toString() const;

        void setUseCompression(bool useCompression){
            _useCompression = useCompression;
        }

        void setFrameRate(unsigned short fps){
            _fps = fps;
        }

        bool isDataAcqRunning() {
            using namespace rw::common;
            bool dataAcqTimeout = (_lastDataAcqTime+3000/_fps)<TimerUtil::currentTimeMs();
            return _cConfig.isDataAcqRunning() && !dataAcqTimeout;
        }

        int getTimestamp(){
            return _timeStamp;
        }

        const ControllerConfig& getControllerConfig(){
            return _cConfig;
        }

        const SensorConfig& getSensorConfig(){
            return _sConfig;
        }

        const std::vector<TactileMatrix>& getPads()
        {
            return _pads;
        }

        const TactileMatrix& getPad(int indx)
        {
            return _pads[indx];
        }

        const std::vector<TactileMaskMatrix>& getStaticMasks()
        {
            return _staticMasks;
        }

        const TactileMaskMatrix& getStaticMask(int indx)
        {
            return _staticMasks[indx];
        }

    private:
        void parseDataFrame(unsigned char *payload, unsigned short payloadSize);
        void parsePropertiesFrame(unsigned char *payload, unsigned short payloadSize);

        static bool StopDataAcquisition(rwhw::SerialPort *sPort);
        static bool QueryControllerConfig(rwhw::SerialPort *sPort, ControllerConfig *cConfig);
        static bool QuerySensorConfig(rwhw::SerialPort *sPort, SensorConfig *sConfig);
        static bool QueryMatrixConfig(rwhw::SerialPort *sPort, unsigned char matrixNum,
                                      MatrixConfig *mConfig);
        //static bool SetPropertiesSampleRate(rwlibs::io::SerialPort *sPort, unsigned short sampleRate);
        //static bool SetPropertiesVectorMatrix(rwlibs::io::SerialPort *sPort,
        //                                    unsigned char matrixNum, unsigned char bitVector);
        static bool ReadMatrixMask(rwhw::SerialPort *sPort, unsigned char maskType,
                                   unsigned char matrixNum, TactileMaskMatrix *mat);
        static bool ReadDescriptorString(rwhw::SerialPort *sPort, unsigned char descrType,
                                         unsigned char matrixNum, std::string *str);
        static bool ReadAck(unsigned char cmdId, unsigned int timeout,
                            rwhw::SerialPort *sPort, void* answer=NULL);
        static bool ReadHeader(unsigned char &id, unsigned short &payloadSize,
                               unsigned char *header, unsigned int timeout,
                               rwhw::SerialPort *sPort);
        static void ParsePacket(unsigned char *payload, unsigned short payloadSize,
                                unsigned char id, void* answer);
        static void ValidateChecksum(unsigned char *header, unsigned char *payload,
                                     unsigned short payloadSize, unsigned int timeout,
                                     rwhw::SerialPort *sPort);
        static unsigned short CalcCrc16Checksum(unsigned char *data, unsigned short size,
                                                unsigned short crc=0xFFFF);

        static void SendBlocking(unsigned char* data, int len, unsigned char id,
                                int timeout, rwhw::SerialPort *sPort)
        {
            for(int retry=0; retry<5;retry++)
            {
                try {
                    //unsigned char *buffer = &(data[len]);
                    sPort->write((char*)data, len);
                    ReadAck(id, timeout, sPort);
                    break;
                } catch (const rw::common::Exception&) {
                    // do nothing
                }
            }
        };
    private:
        rwhw::SerialPort *_sPort;
        int _timeStamp;
        bool _useCompression;
        unsigned short _fps;
        //bool _isDataAckRunning;

        ControllerConfig _cConfig;
        SensorConfig _sConfig;
        MatrixConfig *_mConfigs;
        std::vector<TactileMatrix> _pads;
        std::vector<TactileMaskMatrix> _staticMasks;

        //unsigned char _buffer[2048];
        long _lastDataAcqTime;

        static size_t _preambleIdx;
        static const unsigned short CRC_TABLE_CCITT16[256];
    };

    /* @} */

}

#endif //RWHW_DSACON32_HPP
