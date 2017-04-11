/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "PathLoaderCSV.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>

#include <fstream>
#include <string>
#include <vector>

using rw::common::StringUtil;
using rw::kinematics::State;
using rw::loaders::PathLoaderCSV;
using rw::math::Q;
using namespace rw::models;
using namespace rw::trajectory;

namespace
{
//maybe delete these
std::string
quote(const std::string& str)
{
    return StringUtil::quote(str);
}

class Reader
{
public:
    Reader(
            const std::string& name)
        :
          _name(name)
    {
        q_size = 0;
        q_count = 0;
        file.open(name);
        std::string line;

        std::getline(file,line,';');
        q_count = std::stoi(line);
        std::getline(file,line,';');
        q_size = std::stoi(line);

    }


    // This function is where the magic happens.
    void getQState(const WorkCell& workcell, State& state)
    {
        // Read the configuration for the robot frame
        typedef std::vector<Device::Ptr>::const_iterator DevI;

        const std::vector<Device::Ptr>& devices = workcell.getDevices();
        for (DevI it = devices.begin(); it != devices.end(); ++it)
        {
            int n = (*it)->getDOF();
            std::string name = (*it)->getName();
            if (n > q_size)
                RW_THROW(name + " has " + std::to_string(n) +
                         " degrees of freedom, but .csv file only has " +
                         std::to_string(q_size));

            Q q(q_size, current_state.data());
            (*it)->setQ(q,state);
        }


    }
    // This function reads a line from the .csv file,
    // and inserts the values into current_state vector<double>
    void getNextStateFromFile()
    {
        if (file.eof())
            RW_THROW("End of file reached before expected. Check your csv file");
        std::string line;
        std::getline(file,line,';');
        current_state.clear();

        std::string subs;
        for (int i = 0; i < q_size; i++)
        {
            subs = line.substr(0,line.find(","));
            line.erase(0,line.find(",")+1);

            current_state.push_back(std::stod(subs));
        }
        current_state.push_back(std::stod(line));
    }


    State getState(const WorkCell& workcell)
    {
        State state = workcell.getDefaultState();
        getQState(workcell, state);
        return state;
    }

    Timed<State> getTimedState(const WorkCell& workcell)
    {
        const double time = current_state.front();
        current_state.erase(current_state.begin());
        const State state = getState(workcell);
        return Timed<State>(time, state);
    }

    StatePath getStatePath(const WorkCell& workcell)
    {
        StatePath path;
        for (int i = 0; i < q_count; i++)
        {
            getNextStateFromFile();
            path.push_back(getState(workcell));
        }
        return path;
    }

    TimedStatePath getTimedStatePath(const WorkCell& workcell)
    {
        TimedStatePath path;
        for (int i = 0; i < q_count; i++) {
            getNextStateFromFile();
            path.push_back(getTimedState(workcell));
        }
        return path;
    }

    QPath getPath()
    {
        QPath path;
        for (int i = 0; i < q_count; i++)
        {
            getNextStateFromFile();
            Q q(q_size, current_state.data());
            path.push_back(q);
        }
        return path;
    }


private:
    void die(const std::string& msg)
    {
        RW_THROW(
                    header()
                    << "Unexpected end of file: "
                    << msg);
    }

    std::string header() { return "Reading " + quote(_name) + ": "; }

    std::string _name; // Identifier for the resource being read.
    std::ifstream file;
    std::vector<double> current_state; // Current line being read of csv
    int q_size, q_count;
};
}

QPath PathLoaderCSV::loadPath(
        const std::string& file)
{
    Reader reader(file);
    return reader.getPath();
}


StatePath PathLoaderCSV::loadStatePath(
        const WorkCell& workcell,
        const std::string& file)
{
    Reader reader(file);

    return reader.getStatePath(workcell);
}



TimedStatePath PathLoaderCSV::loadTimedStatePath(
        const WorkCell& workcell, const std::string& file)
{

    std::string locale = setlocale(LC_ALL, NULL); // save old locale
    setlocale(LC_ALL, "C"); // change locale
    Reader reader(file);
    return reader.getTimedStatePath(workcell);
    setlocale(LC_ALL, locale.c_str()); // set locale back
}


