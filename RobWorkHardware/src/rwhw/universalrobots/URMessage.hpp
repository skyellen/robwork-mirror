/* */
#ifndef RWHW_URMESSAGE_HPP
#define RWHW_URMESSAGE_HPP

#include <string>

class URMessage {
public:
	URMessage(int type, int code, int arg, const std::string& text):
		_type(type),
		_code(code),
		_arg(arg),
		_text(text)
	{}


    friend std::ostream& operator<<(std::ostream& os, const URMessage& msg) {
    	switch (msg._type) {
    	case ROBOT_MESSAGE_TEXT:
    		os<<"ROBOT_MESSAGE_TEXT: "<<msg._text;
    		break;
    	case ROBOT_MESSAGE_ERROR_CODE:
    		os<<"ROBOT_MESSAGE_ERROR_CODE: ERROR_CODE = "<<msg._code<<" ERROR_ARG = "<<msg._arg<<" TEXT = "<<msg._text;
    		break;
    	case ROBOT_MESSAGE_SECURITY:
    		os<<"ROBOT_MESSAGE_SECURITY: ERROR_CODE = "<<msg._code<<" ERROR_ARG = "<<msg._arg<<" TEXT = "<<msg._text;
    		break;
    	default:
    		os<<"TYPE = "<<msg._type<<" CODE = "<<msg._code<<" ARG = "<<msg._arg<<" TEXT = "<<msg._text;
    		break;
    	}
        return os;
    }

	enum MSG_TYPE { ROBOT_MESSAGE_VERSION = 3,
		ROBOT_MESSAGE_SECURITY = 5,
		ROBOT_MESSAGE_ERROR_CODE = 6,
		ROBOT_MESSAGE_KEY = 7,
		ROBOT_MESSAGE_PROGRAM_LABEL = 1,
		ROBOT_MESSAGE_POPUP = 2,
		ROBOT_MESSAGE_TEXT = 0,
		ROBOT_MESSAGE_VARIABLE = 8};

private:
	int _type;
	int _code;
	int _arg;
	std::string _text;
};

#endif //#ifndef RWHW_URCOMMON_HPP
