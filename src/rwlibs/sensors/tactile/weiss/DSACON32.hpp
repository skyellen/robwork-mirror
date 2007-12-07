#ifndef DSACON32_HPP_
#define DSACON32_HPP_

#include <rwlibs/io/serialport/SerialPort.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <rw/common/ConvertUtil.hpp>

namespace rwlibs{
namespace sensors {

    class DSACON32
    {
    private:
        DSACON32( rwlibs::io::SerialPort &port, std::pair<int,int> dim);
        
        virtual ~DSACON32();

    
    public:
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
                ostr <<  "#cells x: " << cellsX << "\n";
                ostr <<  "#cells y: " << cellsY << "\n";
                ostr <<  "HW revision: " << hwRevision << "\n";
                ostr <<  "Serial number: " << serialNumber << "\n";
                return ostr.str();
            }
            
        };        
    	
        static DSACON32* GetInstance(rwlibs::io::SerialPort &port);
        
        virtual void Initialize();
        
        virtual void Close();
        
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
        

        
    private:
        void ParsePacket(unsigned char data[],unsigned int payloadSize);
        void ParseDataFrame(unsigned char data[], unsigned int payloadSize);
        void validateChecksum(unsigned char data[], int checksumIndx);
        
    private:
        rwlibs::io::SerialPort *_sPort;
        boost::numeric::ublas::vector<short> _sensorArray;
        std::pair<int,int> _dim;
        bool _useCompression;
        unsigned short _fps;
        bool _isDataAckRunning;
        SensorConfig _sConfig
    };

}
}

#endif /*DSACON32_HPP_*/
