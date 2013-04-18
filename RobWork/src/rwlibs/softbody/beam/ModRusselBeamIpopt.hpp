/*
    Copyright 2013 <copyright holder> <email>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#ifndef MODRUSSELBEAMIPOPT_HPP
#define MODRUSSELBEAMIPOPT_HPP

#include "ModRusselBeamBase.hpp"

#include "IpIpoptApplication.hpp"



namespace rwlibs {
namespace softbody {
class ModRusselBeamIpopt : public ModRusselBeamBase
{
public:
    ModRusselBeamIpopt ( 
        boost::shared_ptr< BeamGeometry > geomPtr,
        boost::shared_ptr< BeamObstaclePlane > obstaclePtr,
        int M );
    
    virtual ~ModRusselBeamIpopt();
    
    void solve(void);
    
public:
    
    
    
private:
    Ipopt::SmartPtr< Ipopt::TNLP > _nlp;
    Ipopt::SmartPtr< Ipopt::IpoptApplication > _app;
};
}}

#endif // MODRUSSELBEAMIPOPT_HPP
