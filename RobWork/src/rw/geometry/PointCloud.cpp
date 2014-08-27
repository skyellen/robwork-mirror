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

#include "PointCloud.hpp"

#include "PlainTriMesh.hpp"

#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

using namespace rw::geometry;
using namespace rw::common;

rw::common::Ptr<TriMesh> PointCloud::getTriMesh(bool forceCopy){
    // we create a trimesh with points of size 1mm
    //std::cout << "Creating mesh... " << _data.size() << std::endl;
    PlainTriMeshF::Ptr mesh = ownedPtr( new PlainTriMeshF((int)_data.size()) );
    for(size_t i=0;i<_data.size();i++){
        Triangle<float> tri = (*mesh)[i];
        tri[0] = _data[i];
        tri[1] = _data[i]+rw::math::Vector3D<float>(0.005f,0, 0);
        tri[2] = _data[i]+rw::math::Vector3D<float>(0, 0.005f,0);
        (*mesh)[i] = tri;
    }
    return mesh;
}

rw::common::Ptr<const TriMesh> PointCloud::getTriMesh(bool forceCopy) const{
    PlainTriMeshF::Ptr mesh = ownedPtr( new PlainTriMeshF((int)_data.size()) );
    //std::cout << "Creating mesh... " << _data.size() << std::endl;
    for(size_t i=0;i<_data.size();i++){
        Triangle<float> tri = (*mesh)[i];
        tri[0] = _data[i];
        tri[1] = _data[i]+rw::math::Vector3D<float>(0.005f,0, 0);
        tri[2] = _data[i]+rw::math::Vector3D<float>(0,0.005f,0);
        (*mesh)[i] = tri;
    }
    return mesh;
}


void PointCloud::savePCD(const PointCloud& img, const std::string& filename, const rw::math::Transform3D<float>& t3d){
    std::ofstream output(filename.c_str());
    output << "# .PCD v.5 - Point Cloud Data file format\n";
    output << "FIELDS x y z\n";
    output << "SIZE 4 4 4\n";
    output << "TYPE F F F\n";
    output << "WIDTH " << img.getWidth() << "\n";
    output << "HEIGHT " << img.getHeight() << "\n";
    output << "POINTS " << img.getData().size() << "\n";
    output << "DATA ascii\n";
    BOOST_FOREACH(const rw::math::Vector3D<float>& p_tmp, img.getData()){
        rw::math::Vector3D<float> p = t3d*p_tmp;
        output << p(0) << " " << p(1) << " " << p(2) << "\n";
    }
    output.close();
}

namespace {
    PointCloud::Ptr loadPCD_0_7(const std::string& filename ){
        //Start by storing the current locale. This is retrieved by passing NULL to setlocale
        std::string locale = setlocale(LC_ALL, NULL);
        setlocale( LC_ALL, "C" );

        char line[500];
        int width, height, nr_points;
        float version;
        std::ifstream input(filename.c_str());
        input.getline(line, 500); // output << "# .PCD v.5 - Point Cloud Data file format\n";
        input.getline(line, 500); // output << "FIELDS x y z\n";
        input.getline(line, 500); // output << "SIZE 4 4 4\n";
        input.getline(line, 500); //output << "TYPE F F F\n";
        std::string tmp;
        input >> tmp;
        while(tmp!="DATA"){
            if(tmp=="WIDTH"){ input >> width;}
            else if(tmp=="HEIGHT"){input >> height;}
            else if(tmp=="VERSION"){input >> version;}
            else if(tmp=="FIELDS"){}
            else if(tmp=="HEIGHT"){}
            else if(tmp=="SIZE"){}
            else if(tmp=="TYPE"){}
            else if(tmp=="COUNT"){}
            else if(tmp=="VIEWPOINT"){}
            else if(tmp=="POINTS"){input >> nr_points;}
            input.getline(line, 500);
            input >> tmp;
        }
        PointCloud::Ptr img;
        // next data should be either ascii or binary
        std::string type;
        input >> type;
        if(type=="binary"){
            setlocale( LC_ALL, locale.c_str());
            RW_THROW("binary PCD format not supported! Please convert to ascii PCD format in order to use this loader!");
        } else {
            // assume that data type is ascii
            input.getline(line, 500);
            //std::cout << "w:" << width << " h:" << height << " p:" << nr_points << std::endl;
            PointCloud::Ptr img = rw::common::ownedPtr( new PointCloud(width,height) );
            input.getline(line, 500); // output << "DATA ascii\n";

            for(int i=0;i<nr_points;i++){
                rw::math::Vector3D<float> p;
                input.getline(line, 500);
                sscanf(line, "%f %f %f", &p[0], &p[1], &p[2]);
                img->getData()[i] = p;
            }

            setlocale( LC_ALL, locale.c_str());
        }
        return img;
    }
}


PointCloud::Ptr PointCloud::loadPCD(const std::string& filename ){
    //Start by storing the current locale. This is retrieved by passing NULL to setlocale
    std::string locale = setlocale(LC_ALL, NULL);
    setlocale( LC_ALL, "C" );

    char line[500];
    int width, height, nr_points;
    float version;
    std::ifstream input(filename.c_str());
    input.getline(line, 500); // output << "# .PCD v.5 - Point Cloud Data file format\n";
    input.getline(line, 500); // output << "FIELDS x y z\n";
    input.getline(line, 500); // output << "SIZE 4 4 4\n";
    input.getline(line, 500); //output << "TYPE F F F\n";
    std::string tmp;
    input >> tmp;
    while(tmp!="DATA"){
        if(tmp=="WIDTH"){ input >> width;}
        else if(tmp=="HEIGHT"){input >> height;}
        else if(tmp=="VERSION"){input >> version;}
        else if(tmp=="FIELDS"){}
        else if(tmp=="HEIGHT"){}
        else if(tmp=="SIZE"){}
        else if(tmp=="TYPE"){}
        else if(tmp=="COUNT"){}
        else if(tmp=="VIEWPOINT"){}
        else if(tmp=="POINTS"){input >> nr_points;}
        input.getline(line, 500);
        input >> tmp;
    }
    input.getline(line, 500);
    //std::cout << "w:" << width << " h:" << height << " p:" << nr_points << std::endl;
    PointCloud::Ptr img = rw::common::ownedPtr( new PointCloud(width,height) );
    input.getline(line, 500); // output << "DATA ascii\n";

    for(int i=0;i<nr_points;i++){
        rw::math::Vector3D<float> p;
        input.getline(line, 500);
        sscanf(line, "%f %f %f", &p[0], &p[1], &p[2]);
        img->getData()[i] = p;
    }

    setlocale( LC_ALL, locale.c_str());
    return img;
}
