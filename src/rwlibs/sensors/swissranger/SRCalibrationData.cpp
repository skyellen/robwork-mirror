/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "SRCalibrationData.hpp"
#include <fstream>

using namespace rwlibs::sensors::swissranger;

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
