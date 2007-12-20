#ifndef DSACON32_HPP_
#define DSACON32_HPP_

#include <rwlibs/io/serialport/SerialPort.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <rw/common/ConvertUtil.hpp>
#include "TactileMatrix.hpp"


namespace rwlibs{
namespace sensors {

    class DSACON32
    {
    private:
        DSACON32( rwlibs::io::SerialPort &port, std::pair<int,int> dim);
        
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
    			serialNumber = ConvertUtil::ToInt32(data, 8);
    			hwRevision = data[12];
    			swBuild = ConvertUtil::ToInt16(data, 13);
    			statusFlags = data[15];
    			featureFlags = data[16];
    			sensconType = (SensconTypes)data[17];
    			interfaceType = (InterfaceTypes)data[18];
    		}

    		
    		std::string toString()
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
                texelWidth = ConvertUtil::ToFloat32(buffer, 0);
                for(int i=0; i<4; i++)
                    buffer[i] = data[13 + i];
                texelHeight = ConvertUtil::ToFloat32(buffer, 0);
                cellsX = ConvertUtil::ToInt16(data, 17);
                cellsY = ConvertUtil::ToInt16(data, 19);
                hwRevision = data[21];
                serialNumber = ConvertUtil::ToInt32(data, 22);
                return true;
            }
            
            std::string toString(){
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
        
        //virtual void Initialize();
        
        //virtual void Close();
        
        void StartDataAcquisition();
        
        void StopDataAcquisition();
        
        void QueryControllerConfig();
        
        void QuerySensorConfig();
        
        void QueryControllerState();
        
        void ReadThreadFunc();
        
        void setUseCompression(bool useCompression){
            _useCompression = useCompression;
        }
        
        void setFrameRate(unsigned short fps){
            _fps = fps;
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
        void ParsePacket(unsigned char data[],unsigned int payloadSize);
        void ParseDataFrame(unsigned char data[], unsigned int payloadSize);
        void validateChecksum(unsigned char data[], int checksumIndx);
        
    private:
        rwlibs::io::SerialPort *_sPort;
        int _timeStamp; 
        TactileMatrix _padA, _padB;
        std::pair<int,int> _dim;
        bool _useCompression;
        unsigned short _fps;
        bool _isDataAckRunning;
        SensorConfig _sConfig;
        ControllerConfig _cConfig;
    };

}
}

#endif /*DSACON32_HPP_*/
