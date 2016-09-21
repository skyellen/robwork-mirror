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


#include "TactileArrayModel.hpp"

using namespace rw::sensor;
using namespace rw::math;

TactileArrayModel::TactileArrayModel(const std::string& name,
			rw::kinematics::Frame * frame,
			const rw::math::Transform3D<>& fThmap,
			const ValueMatrix& heightMap,
			double cw, double ch):
					SensorModel(name, frame),
					_fThmap(fThmap),_heightMap(heightMap),_cellWidth(cw),_cellHeight(ch),
					_sdata(1, rw::common::ownedPtr( new TactileModelCache()).cast<rw::kinematics::StateCache>())
{
	_minPressure = 0;
	_maxPressure = 1000;

	 int w((int)_heightMap.cols()-1);
	 int h((int)_heightMap.rows()-1);

    // calculate the normals and centers of all texels
    // first calculate the 3D vertexes of the grid from the heightmap specification
    double tw = _cellWidth, th = _cellHeight;
    _vertexGrid.resize(boost::extents[w+1][h+1]);
    for(int y=0;y<h+1; y++){
        for(int x=0;x<w+1; x++){
            _vertexGrid[x][y](0) = x*tw;
            _vertexGrid[x][y](1) = y*th;
            _vertexGrid[x][y](2) = _heightMap(y,x);
        }
    }

    _cellCenters.resize(boost::extents[w][h]);
    _cellNormals.resize(boost::extents[w][h]);
    for(int j=0;j<h; j++){
     	for(int i=0;i<w; i++){
            Vector3D<> p = _vertexGrid[i][j]  +_vertexGrid[i+1][j]+
            		_vertexGrid[i][j+1]+_vertexGrid[i+1][j+1];
            _cellCenters[i][j] = p/4;
            // now calculate unit normal
            Vector3D<> n = cross(_vertexGrid[i+1][j]-_vertexGrid[i][j],
            		_vertexGrid[i+1][j+1]-_vertexGrid[i][j] );
            _cellNormals[i][j] = normalize(n);
        }
    }
    add(_sdata);
}


TactileArrayModel::~TactileArrayModel(){}

rw::math::Vector2D<> TactileArrayModel::getTexelSize(int x, int y) const{
	return rw::math::Vector2D<>(_cellWidth,_cellHeight);
}

std::pair<double,double> TactileArrayModel::getPressureLimit() const {
	return std::make_pair(_minPressure, _maxPressure);
}


void TactileArrayModel::setPressureLimit(double min, double max) {
	_minPressure = min;
	_maxPressure = max;
}


const TactileArrayModel::VertexMatrix& TactileArrayModel::getVertexGrid() const {
	return _vertexGrid;
}

const rw::math::Transform3D<>& TactileArrayModel::getTransform() const{ return _fThmap; }

const TactileArrayModel::VertexMatrix& TactileArrayModel::getCenters() const{
	return _cellCenters;
}

const TactileArrayModel::VertexMatrix& TactileArrayModel::getNormals()  const{
	return _cellNormals;
}

int TactileArrayModel::getWidth() const{
	return static_cast<int>(_heightMap.cols()-1);
}

int TactileArrayModel::getHeight() const{
	return static_cast<int>(_heightMap.rows()-1);
}


//************** the statefull interface (dynamic states) ***************

TactileArrayModel::ValueMatrix& TactileArrayModel::getTexelData(rw::kinematics::State& state) const{
	return *_sdata.getStateCache<TactileModelCache>(state)->_data;
}

const TactileArrayModel::ValueMatrix& TactileArrayModel::getTexelData(const rw::kinematics::State& state) const{
	return *_sdata.getStateCache<TactileModelCache>(state)->_data;
}

void TactileArrayModel::setTexelData(const TactileArrayModel::ValueMatrix& data, rw::kinematics::State& state) const{
	(*_sdata.getStateCache<TactileModelCache>(state)->_data) = data;
}
