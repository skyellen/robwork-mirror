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

#include "Model3D.hpp"

using namespace rw::graphics;

Model3D::Model3D(){}
Model3D::~Model3D(){}


int Model3D::addObject(Model3D::Object3D* obj){
    _objects.push_back(obj);
    return _objects.size()-1;
}

int Model3D::addMaterial(const Model3D::Material& mat){
    _materials.push_back(mat);
    return _materials.size()-1;
}

void Model3D::removeObject(const std::string& name){

}
