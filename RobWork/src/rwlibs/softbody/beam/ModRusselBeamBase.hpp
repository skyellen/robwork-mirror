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


#ifndef MODRUSSELBEAMBASE_HPP
#define MODRUSSELBEAMBASE_HPP

#include <boost/shared_ptr.hpp>

#include <rw/math/Transform3D.hpp>

#include "BeamGeometry.hpp"
#include "BeamObstaclePlane.hpp"


namespace rwlibs {
namespace softbody {
class ModRusselBeamBase {
public:
    ModRusselBeamBase (
        boost::shared_ptr< rwlibs::softbody::BeamGeometry > geomPtr,
        boost::shared_ptr< rwlibs::softbody::BeamObstaclePlane > obstaclePtr,
        int M
    );

    virtual ~ModRusselBeamBase();
    
public:
    boost::shared_ptr< BeamGeometry > getGeometry ( void ) const;
    boost::shared_ptr< BeamObstaclePlane > getObstacle ( void ) const;

    int getM ( void )  const;
    
    double getAccuracy ( void ) const;
    void setAccuracy ( double acc );

public:
    rw::math::Transform3D< double > get_planeTbeam ( void ) const ;
    double get_thetaTCP ( void ) const ;

    double get_yTCP ( void ) const ;
    double get_uxTCPy ( void ) const ;
    double get_uyTCPy ( void ) const ;
    
    double get_h ( void )  const ;

private:
    boost::shared_ptr< BeamGeometry > _geomPtr;
    boost::shared_ptr< BeamObstaclePlane > _obstaclePtr;
    int _M;

    double _accuracy;
};
}
};

#endif // MODRUSSELBEAMBASE_HPP
