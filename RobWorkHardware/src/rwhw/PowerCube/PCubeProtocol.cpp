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
    data.push_back((unsigned char)commandId);
    data.push_back((unsigned char)motionId);
    return data;
}

//B�gild
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



