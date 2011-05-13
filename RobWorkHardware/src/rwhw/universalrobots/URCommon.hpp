/* */
#ifndef RWHW_URCOMMON_HPP
#define RWHW_URCOMMON_HPP

#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/common/types.hpp>
#include <boost/asio.hpp>

class URCommon {
public:
	static inline bool getChar(boost::asio::ip::tcp::socket* socket, char* output) {
		return socket->read_some(boost::asio::buffer(output, 1));
	}

	static inline std::string getString(boost::asio::ip::tcp::socket* socket, int cnt, uint32_t& messageOffset) {
		char* ch = new char[cnt+1];
		ch[cnt] = 0;
		socket->read_some(boost::asio::buffer(ch, cnt));
		messageOffset += cnt;
		return std::string(ch);
	}

	static inline unsigned char getUChar(boost::asio::ip::tcp::socket* socket, uint32_t &messageOffset) {
		unsigned char output = 0;
		getChar(socket, (char*)&output+0);
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

	static inline rw::math::Q getQ(boost::asio::ip::tcp::socket* socket, size_t cnt, uint32_t& messageOffset) {
		rw::math::Q res(cnt);
		for (size_t i = 0; i<cnt;i++) {
			res(i) = getDouble(socket, messageOffset);
		}
		return res;
	}


};

#endif //#ifndef RWHW_URCOMMON_HPP
