#include "PCubeProtocol.hpp"


std::vector<unsigned char> PCubeProtocol::makeData(int commandId,
                      int motionId,
                      int val)
{
    ToData move_data(val);
    std::vector<unsigned char> data;
    data.push_back(commandId);
    data.push_back(motionId);
    data.insert(data.end(), move_data.data, move_data.data + sizeof(val));
    return data;
}

std::vector<unsigned char> PCubeProtocol::makeData(int commandId,
                      int motionId,
                      float val)
{
    ToData move_data(val);
    std::vector<unsigned char> data;
    data.push_back(commandId);
    data.push_back(motionId);
    data.insert(data.end(), move_data.data, move_data.data + sizeof(val));
    return data;
}

std::vector<unsigned char> PCubeProtocol::makeData(int commandId,
                      int motionId,
                      float x,
                      int y)
{
    std::vector<unsigned char> data;
    data.push_back(commandId);
    data.push_back(motionId);

    {
        ToData move_data(x);
        data.insert(data.end(), move_data.data, move_data.data + sizeof(x));
    }
    {
        ToData move_data(y);
        data.insert(data.end(), move_data.data, move_data.data + sizeof(y));
    }

    return data;
}

std::vector<unsigned char> PCubeProtocol::makeData(int commandId,
                      int motionId,
                      int x,
                      int y)
{
    std::vector<unsigned char> data;
    data.push_back(commandId);
    data.push_back(motionId);

    {
      ToData move_data(x);
      data.insert(data.end(), move_data.data, move_data.data + sizeof(x));
    }
    {
      ToData move_data(y);
      data.insert(data.end(), move_data.data, move_data.data + sizeof(y));
    }

    return data;
}

std::vector<unsigned char> PCubeProtocol::makeData(int commandId, int motionId)
{
    std::vector<unsigned char> data;
    data.push_back(commandId);
    data.push_back(motionId);
    return data;
}

//Bøgild
std::vector<unsigned char> PCubeProtocol::makeData(int commandId,
                      int parameterId,
                      unsigned int val)
{
    ToData move_data(val);
    std::vector<unsigned char> data;
    data.push_back(commandId);
    data.push_back(parameterId);
    data.insert(data.end(), move_data.data, move_data.data + sizeof(val));
    return data;
}


std::vector<unsigned char> PCubeProtocol::makeData(unsigned char x) 
{
    std::vector<unsigned char> data;
    data.push_back(x);
    return data;
}



