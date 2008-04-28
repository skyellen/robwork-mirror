#ifndef DSACON32_HPP_
#define DSACON32_HPP_

#include <rwlibs/io/serialport/SerialPort.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <rw/common/ConvertUtil.hpp>
#include "TactileMatrix.hpp"
#include <rw/common/TimerUtil.hpp>
#include <rw/common/macros.hpp>


namespace rwlibs{
namespace sensors {

    class DSACON32
    {
    public:
    	struct SensorConfig;
    	struct ControllerConfig;
    private:
        DSACON32( rwlibs::io::SerialPort &port, 
        		  const ControllerConfig &config, 
        		  const SensorConfig &sConfig);
        
        virtual ~DSACON32();

    
    public:
    	
    	struct ControllerConfig {
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
    		
    		void parse(unsigned char data[])
    		{
    			serialNumber = ConvertUtil::toInt32(data, 8);
    			hwRevision = data[12];
    			swBuild = ConvertUtil::toInt16(data, 13);
    			statusFlags = data[15];
    			featureFlags = data[16];
    			sensconType = (SensconTypes)data[17];
    			interfaceType = (InterfaceTypes)data[18];
    		}
    		
    		bool isDataAcqRunning(){
    			return ((statusFlags & 64) == 64);
    		}
    		
    		std::string toString() const 
    		{
    			std::ostringstream ostr;
    			ostr << "Serial number: " << serialNumber << std::endl;
    			ostr << "HW revision: " << (hwRevision / 16) << "." << (hwRevision % 16) << std::endl;
    			ostr << "SW build: " << swBuild << std::endl;
    			ostr << "Status -\n\tsensor ctrl operable: " << ((statusFlags & 128) == 128) << std::endl;
    			ostr << "\tdata acquisition running: " << ((statusFlags & 64) == 64) << std::endl;
    			ostr << "Features - \n\t USB: " << ((featureFlags & 64) == 64) << std::endl;
    			ostr << "\t CAN-bus: " << ((featureFlags & 32) == 32) << std::endl;
    			ostr << "\t RS232: " << ((featureFlags & 16) == 16) << std::endl;
    			ostr << "Sensor controller type: " <<  sensconType << std::endl;
    			ostr << "Current interface: " << interfaceType<< std::endl;

    			return ostr.str();
    		}
    		
    		
    		
    	};
    	
    	
        struct SensorConfig {
        public:
            float texelWidth, texelHeight;
            int cellsX, cellsY;
            unsigned char hwRevision;
            int serialNumber;
            unsigned char featureFlags;
                        
        public:
            
            SensorConfig(){ }
            
            bool parse(unsigned char data[]){
                using namespace rw::common;
                unsigned char buffer[4];
                for(int i=0; i<4; i++)
                    buffer[i] = data[9 + i];
                texelWidth = ConvertUtil::toFloat32(buffer, 0);
                for(int i=0; i<4; i++)
                    buffer[i] = data[13 + i];
                texelHeight = ConvertUtil::toFloat32(buffer, 0);
                cellsX = ConvertUtil::toInt16(data, 17);
                cellsY = ConvertUtil::toInt16(data, 19);
                hwRevision = data[21];
                serialNumber = ConvertUtil::toInt32(data, 22);
                return true;
            }
            
            std::string toString() const {
                std::ostringstream ostr;
                ostr << "Texel width:  " << texelWidth << "mm\n";
                ostr <<  "Texel height: " << texelHeight << "mm\n";
                ostr <<  "#cells x: " << cellsX << std::endl;
                ostr <<  "#cells y: " << cellsY << std::endl;
                ostr <<  "HW revision: " << (unsigned short)hwRevision << std::endl;
                ostr <<  "Serial number: " << serialNumber << std::endl;
                return ostr.str();
            }
            
        };        
    	
        static DSACON32* GetInstance(rwlibs::io::SerialPort &port);
                
        static void QueryControllerConfig(ControllerConfig &config, 
        								  rwlibs::io::SerialPort &port);
        
        static void QuerySensorConfig(SensorConfig &sport, 
        							  rwlibs::io::SerialPort &port);
        
        static void QueryControllerState(rwlibs::io::SerialPort &port);
        static void QueryLoopTest(rwlibs::io::SerialPort &port);

        //virtual void Initialize();
        
        //virtual void Close();
        
        //************ And now the member functions
        /**
         * @brief 
         * @return false when acknowledge was not recieved, true otherwise
         */
        bool startDataAcquisition();
        bool stopDataAcquisition();
        bool queryControllerConfig();
        bool querySensorConfig();
        bool queryControllerState();
        bool queryLoopTest();
        
        bool readThreadFunc();
        
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
        
        const TactileMatrix& getPadA(){
        	return _padA;
        }

        const TactileMatrix& getPadB(){
        	return _padB;
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
        
    private:
        void parsePacket(unsigned char data[], unsigned char id, unsigned short payloadSize);
        void parseDataFrame(unsigned char data[], unsigned int payloadSize);
        void validateChecksum(unsigned char data[], int checksumIndx);
        bool readHeader(unsigned char &id, unsigned short &payload, unsigned int timeout);
        bool readAck(unsigned char cmdId, unsigned int timeout);        
        
        static void ReadAck(unsigned char *buff, unsigned char cmdId, 
        					unsigned int timeout,
        					rwlibs::io::SerialPort &port);
        
        static bool ReadHeader(unsigned char *buffer, unsigned char &id, 
        						  unsigned short &payload, unsigned int timeout,
        						  rwlibs::io::SerialPort &port);
        
        static void SendBlocking(unsigned char* data, int len, unsigned char id,
        						int timeout, rwlibs::io::SerialPort &port){
            for(int retry=0; retry<5;retry++){  
        	    try {
        	    	unsigned char *buffer = &(data[len]); 
        	    	port.write((char*)data, len);
        	    	ReadAck(buffer, id, timeout, port);
        	    	break;
        	    } catch (const rw::common::Exception& exc) {
        	    	// do nothing
        	    }
            }
        };
    private:
        rwlibs::io::SerialPort *_sPort;
        int _timeStamp; 
        TactileMatrix _padA, _padB;
        bool _useCompression;
        unsigned short _fps;
        bool _isDataAckRunning;
        
        ControllerConfig _cConfig;
        SensorConfig _sConfig;
        unsigned char _buffer[2048];
        size_t _preambleIdx;
        long _lastDataAcqTime;
    };

}
}

#endif /*DSACON32_HPP_*/
