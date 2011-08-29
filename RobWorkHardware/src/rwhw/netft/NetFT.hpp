#ifndef NETFT_HPP
#define NETFT_HPP

// STL
#include <iostream>

// Boost
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

using namespace boost::asio;
using namespace boost::asio::ip;

/* Communication driver class for the NetFT device
 * 
 * Upon construction, the class sets up variables
 * 
 * The communication is initiated by start()
 * 
 * Newest updated measurements can be retrieved by getData()
 * 
 * The connection is closed upon destruction
 * 
 * Example usage:
 * 
 * rwhw::NetFT netft;
 * netft.start();
 * rwhw::NetFT::NetFTData data = netft.getAllData();
 * netft.print(std::cout, data);
 */
namespace rwhw {
    class NetFT {
        public:
            // NetFT packet
            struct NetFTData {
                // Constructor
                NetFTData(unsigned int statuss, unsigned int lostt, unsigned int countt, const std::vector<double>& dataa, double timestampp) :
                          status(statuss), lost(lostt), count(countt), data(dataa), timestamp(timestampp) {}
                // Status, lost packets, packet count
                unsigned int status, lost, count;

                // F/T data: {Fx, Fy, Fz, Tx, Ty, Tz}
                std::vector<double> data;

                double timestamp;
            };
            
            // Constructor
            NetFT(const std::string& address = "192.168.1.1",
                  unsigned short port = 49152,
                  unsigned int countsPerForce = 1000000,
                  unsigned int countsPerTorque = 1000000);
            
            /* Destructor
             * 
             * Stop the receiver thread and close the socket
             */
            virtual ~NetFT();
            
            /* Open the socket and start the receiver thread
             * 
             * Newest measurements are available through getAllData() and getData()
             */
            void start();
            
            // Get a NetFT packet
            NetFT::NetFTData getAllData();
            
            // Get only F/T data: {Fx, Fy, Fz, Tx, Ty, Tz}
            std::vector<double> getData();
            
            // Print data to a stream
            void print(std::ostream& os, const NetFT::NetFTData& netftData);

            double driverTime();
        
        private:
            // Thread function
            void runReceive();
            
            // Helper functions
            bool waitForNewData();
            void sendStartCommand();
            
            // F/T data: {Fx, Fy, Fz, Tx, Ty, Tz}
            std::vector<double> _data;

            //Time stamp of the current data
            double _timestamp;


            // Socket members
            std::string _address;
            unsigned short _port;
            io_service _ioservice;
            udp::socket _socket;


            // Scaling parameters
            double _countsF, _countsT, _scaleF, _scaleT;

            
            // Thread members
            boost::thread _receiveThread;
            bool _threadRunning, _stopThread;
            boost::mutex _mutex;
      
            // Sequence and status parameters
            uint32_t _lastRdtSequence, _systemStatus;
            unsigned int _outOfOrderCount, _lostPackets, _packetCount;
            
            // Command sent to the device
            struct Command {
                uint16_t _header, _command;
                uint32_t _count;

                Command(uint16_t ct = CMD_START_HIGH_SPEED_STREAMING,
                        uint32_t sz = INFINITE_SAMPLES) : _header(0x1234),
                                                          _command(ct),
                                                          _count(sz) {}

                /* Possible values for command
                 * 
                 * More command values are available, but are unused
                 */
                enum COMMAND_TYPE {
                    CMD_STOP_STREAMING = 0, 
                    CMD_START_HIGH_SPEED_STREAMING = 2
                };

                // Special values for misc. sizes
                enum SIZES {
                    INFINITE_SAMPLES = 0,
                    RDT_COMMAND_SIZE = 8
                };

                //  Buffer should be RDT_COMMAND_SIZE
                void pack(uint8_t *buffer) const {
                    // Data is big-endian
                    buffer[0] = (_header >> 8) & 0xFF;
                    buffer[1] = (_header >> 0) & 0xFF;
                    buffer[2] = (_command >> 8) & 0xFF;
                    buffer[3] = (_command >> 0) & 0xFF;
                    buffer[4] = (_count >> 8) & 0xFF;
                    buffer[5] = (_count >> 0) & 0xFF;
                    buffer[6] = (_count >> 8) & 0xFF;
                    buffer[7] = (_count >> 0) & 0xFF;
                }
            };
            
            // Received message from device
            struct Message {
                uint32_t _rdtSequence, _ftSequence, _status;
                int32_t _fx, _fy, _fz, _tx, _ty, _tz;

                enum {RDT_RECORD_SIZE = 36};
                
                void unpack(const uint8_t *buffer) {
                    _rdtSequence = unpack32(buffer + 0);
                    _ftSequence = unpack32(buffer + 4);
                    _status = unpack32(buffer + 8);
                    _fx = unpack32(buffer + 12);
                    _fy = unpack32(buffer + 16);
                    _fz = unpack32(buffer + 20);
                    _tx = unpack32(buffer + 24);
                    _ty = unpack32(buffer + 28);
                    _tz = unpack32(buffer + 32);
                }
                
                static uint32_t unpack32(const uint8_t *buffer) {
                    return (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | (buffer[3] << 0);
                }
            };
    };
}

#endif
