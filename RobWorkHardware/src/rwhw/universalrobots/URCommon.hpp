/* */
#ifndef RWHW_URCOMMON_HPP
#define RWHW_URCOMMON_HPP

#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/common/types.hpp>
#include <rw/common/TimerUtil.hpp>
#include <boost/asio.hpp>

class URCommon {
public:
	static inline bool getChar(boost::asio::ip::tcp::socket* socket, char* output) {
		return socket->read_some(boost::asio::buffer(output, 1));
	}


	static inline int getData(boost::asio::ip::tcp::socket* socket, int cnt, std::vector<char>& data) {
		if(data.size()<cnt)
			data.resize(cnt);
		socket->read_some(boost::asio::buffer(&data[0], cnt));

		return cnt;
	}


	static inline std::string getString(boost::asio::ip::tcp::socket* socket, int cnt, uint32_t& messageOffset) {
		char* ch = new char[cnt+1];
		ch[cnt] = 0;
		socket->read_some(boost::asio::buffer(ch, cnt));
		messageOffset += cnt;
		return std::string(ch);
	}

	static inline std::string readUntil(boost::asio::ip::tcp::socket* socket, char terminator, uint32_t& messageOffset) {
		char buffer[1024];
		size_t cnt = 0;
		getChar(socket, buffer);
		while (buffer[cnt] != terminator) {
			cnt++;
			getChar(socket, &(buffer[cnt]));
		}
		messageOffset += cnt+1;
		buffer[cnt] = 0;
		return std::string(buffer);
	}

	static inline unsigned char getUChar(boost::asio::ip::tcp::socket* socket, uint32_t &messageOffset) {
		unsigned char output = 0;
		getChar(socket, (char*)&output+0);
		messageOffset += 1;
		return output;
	}

	static inline unsigned char getUChar(std::vector<char>& data, uint32_t &messageOffset) {
		unsigned char output = data[messageOffset];
		messageOffset += 1;
		return output;
	}

	//Extract a 16 bit unsigned int
	static inline uint16_t getUInt16(boost::asio::ip::tcp::socket* socket, uint32_t &messageOffset) {
		uint16_t output = 0;
		getChar(socket, (char*)&output+1);
		getChar(socket, (char*)&output+0);
		messageOffset += 2;
		return output;
	}

	static inline uint16_t getUInt16(std::vector<char>& data, uint32_t &messageOffset) {
		uint16_t output = *((uint16_t*)&data[messageOffset]);
		messageOffset += 2;
		return output;
	}

	static inline uint32_t getUInt32(std::vector<char>& data, uint32_t &messageOffset) {
		uint16_t output = *((uint32_t*)&data[messageOffset]);
		messageOffset += 4;
		return output;
	}

	static inline uint64_t getUInt64(std::vector<char>& data, uint32_t &messageOffset) {
		uint64_t output = *((uint64_t*)&data[messageOffset]);
		messageOffset += 8;
		return output;
	}


	//Extract a 32 bit unsigned int
	static inline uint32_t getUInt32(boost::asio::ip::tcp::socket* socket, uint32_t &messageOffset) {
		uint32_t output = 0;
		getChar(socket, (char*)&output+3);
		getChar(socket, (char*)&output+2);
		getChar(socket, (char*)&output+1);
		getChar(socket, (char*)&output+0);
		messageOffset += 4;
		return output;
	}

	static inline uint64_t getUInt64(boost::asio::ip::tcp::socket* socket, uint32_t &messageOffset) {
		uint64_t output = 0;
		getChar(socket, (char*)&output+7);
		getChar(socket, (char*)&output+6);
		getChar(socket, (char*)&output+5);
		getChar(socket, (char*)&output+4);
		getChar(socket, (char*)&output+3);
		getChar(socket, (char*)&output+2);
		getChar(socket, (char*)&output+1);
		getChar(socket, (char*)&output+0);
		messageOffset += 8;
		return output;
	}

	//Extract a double

	static inline double getDouble(std::vector<char>& data, uint32_t &messageOffset) {
		double output = *((double*)&data[messageOffset]);
		messageOffset += 8;
		return output;
	}

	static inline double getFloat(std::vector<char>& data, uint32_t &messageOffset) {
		float output = *((float*)&data[messageOffset]);
		messageOffset += 4;
		return output;
	}

	static inline double getDouble(boost::asio::ip::tcp::socket* socket, uint32_t &messageOffset) {
		double output = 0;
		getChar(socket, (char*)&output+7);
		getChar(socket, (char*)&output+6);
		getChar(socket, (char*)&output+5);
		getChar(socket, (char*)&output+4);
		getChar(socket, (char*)&output+3);
		getChar(socket, (char*)&output+2);
		getChar(socket, (char*)&output+1);
		getChar(socket, (char*)&output+0);
		messageOffset += 8;
		return output;
	}

	static inline float getFloat(boost::asio::ip::tcp::socket* socket, uint32_t &messageOffset) {
		float output = 0;
		getChar(socket, (char*)&output+3);
		getChar(socket, (char*)&output+2);
		getChar(socket, (char*)&output+1);
		getChar(socket, (char*)&output+0);
		messageOffset += 4;
		return output;
	}



	//Extract a one boolean
	static inline bool getBoolean(std::vector<char>& data, uint32_t &messageOffset) {
		unsigned char tmp = getUChar(data,messageOffset);
		bool output = (tmp & 1)>0;
		return output;
	}


	static inline bool getBoolean(boost::asio::ip::tcp::socket* socket, uint32_t &messageOffset) {
		char tmp = 0;
		getChar(socket, (char*)&tmp);
		bool output = (tmp & 1)>0;
		messageOffset += 1;
		return output;
	}



	//Extract a 3d vector
	static inline rw::math::Vector3D<double> getVector3D(boost::asio::ip::tcp::socket* socket, uint32_t &messageOffset) {
		rw::math::Vector3D<double> output;
		output[0] = getDouble(socket, messageOffset);
		output[1] = getDouble(socket, messageOffset);
		output[2] = getDouble(socket, messageOffset);
		return output;
	}

	static inline rw::math::Vector3D<double> getVector3D(std::vector<char>& data, uint32_t &messageOffset) {
		rw::math::Vector3D<double> output;
		output[0] = getDouble(data, messageOffset);
		output[1] = getDouble(data, messageOffset);
		output[2] = getDouble(data, messageOffset);
		return output;
	}

	static inline rw::math::Q getQ(boost::asio::ip::tcp::socket* socket, size_t cnt, uint32_t& messageOffset) {
		rw::math::Q res(cnt);
		for (size_t i = 0; i<cnt;i++) {
			res(i) = getDouble(socket, messageOffset);
		}
		return res;
	}

    static inline void send(boost::asio::ip::tcp::socket* socket, const std::vector<int>& integers) {        
        const int n = integers.size()*sizeof(int);
        char buffer[n];
        int ibuf[integers.size()];
        for (size_t i = 0; i<integers.size(); i++) {
            ibuf[i] = integers[i];
        }
        memcpy(buffer, ibuf, n);
        
  //      for (size_t i = 0; i<n; i++) {
    //        std::cout<<"buffer = "<<(int)buffer[i]<<" ibuf = "<<(int)((char*)ibuf)[i]<<std::endl;
      //  }

        for (size_t i = 0; i<integers.size(); i++) {
            for (size_t j = 0; j<sizeof(int); j++) {
                buffer[4*i+j] = ((char*)(&integers.at(i)))[3-j];
            }
        }

        /*for (size_t i = 0; i<integers.size(); ++i) {
//            ibuf[i] = integers.at(i);
            std::cout<<"Int = "<<integers.at(i)<<" = "<<(unsigned int)ibuf[4*i]<<" "<<(unsigned int)ibuf[4*i+1]<<" "<<(unsigned int)ibuf[4*i+2]<<" "<<(unsigned int)ibuf[4*i+3]<<std::endl;
            std::cout<<"Int from buffer = "<<*(int*)(&buffer[4*i])<<std::endl;
        }*/
//        ibuf[0] = 254;
        /*for (size_t i = 0; i<n; i+=4) {
            buffer[i] = integers[i/4];
        }
        buffer[0] = 1;
    	socket->send(boost::asio::buffer(buffer, n));*/
    	socket->send(boost::asio::buffer(buffer, n));
    }

    static inline void send(boost::asio::ip::tcp::socket* socket, unsigned char ch) {
    	socket->send(boost::asio::buffer(&ch, 1));
    }

	static inline void send(boost::asio::ip::tcp::socket* socket, const std::string& str) {
		 socket->send(boost::asio::buffer(str.c_str(), str.size()));
	}

	static inline void send(boost::asio::ip::tcp::socket* socket, rw::math::Q& q) {
		std::stringstream sstr;
		sstr<<"(";
		for (size_t i = 0; i<q.size(); i++) {
			sstr<<q(i);
			if (i != q.size()-1)
				sstr<<",";
		}
		sstr<<")";
		send(socket, sstr.str());
	}

	static inline void send(boost::asio::ip::tcp::socket* socket, rw::math::Q& q, float speed) {
		std::stringstream sstr;
		sstr<<"(";
		for (size_t i = 0; i<q.size(); i++) {
			sstr<<q(i)<<",";
		}
		sstr<<speed<<")";
		send(socket, sstr.str());
	}

	static inline double driverTimeStamp() {
		return rw::common::TimerUtil::currentTime();
	}


};

#endif //#ifndef RWHW_URCOMMON_HPP
