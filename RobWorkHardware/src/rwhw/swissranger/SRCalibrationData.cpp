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

#include "SRCalibrationData.hpp"
#include <fstream>

using namespace rwhw::swissranger;

SRCalibrationData::SRCalibrationData() {}

SRCalibrationData::~SRCalibrationData() {}

bool SRCalibrationData::save(const std::string& filename) {
    std::ofstream outfile(filename.c_str());

    if (!outfile.is_open())
        return false;

    outfile<<"<offsets>"<<std::endl;
    for (unsigned short i = 0; i<IMG_SIZE; i++)
        outfile<<_offsets[i]<<" ";
    outfile<<"</offsets>"<<std::endl;

    outfile<<"<gains>"<<std::endl;
    for (unsigned short i = 0; i<IMG_SIZE; i++)
        outfile<<_gains[i]<<" ";
    outfile<<"</gains>"<<std::endl;

    outfile<<"<alpha>"<<std::endl;
    for (unsigned short i = 0; i<IMG_SIZE; i++)
        outfile<<_alphas[i]<<" ";
    outfile<<"</alpha>"<<std::endl;


    outfile<<"<beta>"<<std::endl;
    for (unsigned short i = 0; i<IMG_SIZE; i++)
        outfile<<_betas[i]<<" ";
    outfile<<"</beta>"<<std::endl;

    outfile.close();
    return true;
}


bool SRCalibrationData::load(const std::string& filename) {
    std::ifstream infile(filename.c_str());

    if (!infile.is_open())
        return false;

    std::string token;
    infile >> token;
    if (token != "<offsets>") {
        infile.close();
        return false;
    }
    for (unsigned short i = 0; i<IMG_SIZE; i++)
        infile >> _offsets[i];
    infile >> token;
    if (token != "</offsets>") {
        infile.close();
        return false;
    }

    infile >> token;
    if (token != "<gains>") {
        infile.close();
        return false;
    }
    for (unsigned short i = 0; i<IMG_SIZE; i++)
        infile >> _gains[i];
    infile >> token;
    if (token != "</gains>") {
        infile.close();
        return false;
    }


    infile >> token;
    if (token != "<alpha>") {
        infile.close();
        return false;
    }
    for (unsigned short i = 0; i<IMG_SIZE; i++)
        infile >> _alphas[i];
    infile >> token;
    if (token != "</alpha>") {
        infile.close();
        return false;
    }

    infile >> token;
    if (token != "<beta>") {
        infile.close();
        return false;
    }
    for (unsigned short i = 0; i<IMG_SIZE; i++)
        infile >> _betas[i];
    infile >> token;
    if (token != "</beta>") {
        infile.close();
        return false;
    }

    infile.close();

    return true;
 }
