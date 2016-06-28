
#include "LoaderTRI.hpp"

#include <fstream>

#include <rw/common/macros.hpp>
#include <rw/math/Constants.hpp>

using namespace rw::loaders;
using namespace rw::common;
using namespace rw::graphics;
using namespace rw::geometry;
using namespace rw::math;

#define LINE_MAX_LENGTH 100

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

LoaderTRI::LoaderTRI(){};
LoaderTRI::~LoaderTRI(){};


Model3D::Ptr LoaderTRI::load(const std::string& filename)
{

    std::ifstream input_stream(filename.c_str());
    if (!input_stream.is_open()) {
        RW_THROW("Can't open file " << "'" << filename << "'");
    }
   	//Start by storing the current locale. This is retrieved by passing NULL to setlocale	
	std::string locale = setlocale(LC_ALL, NULL); 

    setlocale(LC_ALL, "C");
    char *next;
    char  token[LINE_MAX_LENGTH];
    int   width;
    char input[LINE_MAX_LENGTH];
    int nb_points = 0;
    Model3D::Ptr model = ownedPtr(new Model3D(filename));
    Model3D::Object3D::Ptr obj = ownedPtr(new Model3D::Object3D("TRIModel"));

    int currentMatIdx = model->addMaterial(Model3D::Material("defcol",0.5,0.5,0.5));
    Model3D::MaterialFaces *mface = new Model3D::MaterialFaces();
    mface->_matIndex = currentMatIdx;
    // while characters still exists and no errors occour
    while (  input_stream.fail()==0 && input_stream.eof()==0 ){
        //  Read the next line of the file into INPUT.
        input_stream.getline(input, LINE_MAX_LENGTH);
        //  Advance to the first nonspace character in INPUT.
        for ( next = input; *next != '\0' && *next == 32; next++ ){}
        // 32==SPACE character

        //  Skip blank lines and comments and linebreaks.
        if ( *next == '\0' || *next == '#' || *next == '!' || *next == '$'  ){
            continue;
        }
        //  Extract the first word in this line.
        sscanf ( next, "%s%n", token, &width );

        //  Set NEXT to point to just after this token.
        next = next + width;

        if ( !strcmp( token, "color" ) ){
        	//if(mface->_subFaces.size()>0){
        	//	obj->_matFaces.push_back(mface);
        	//	mface = new Model3D::MaterialFaces();
        	//}
            float r,g,b;
            sscanf ( next, "%e %e %e", &r, &g, &b );
            // create material in object
            std::stringstream sstr;
            sstr << r <<"_"<< g << "_" << b;
            Model3D::Material mat(sstr.str(), r, g, b);
            currentMatIdx = model->addMaterial(mat);
            obj->setMaterial(currentMatIdx);
            //mface->_matIndex = currentMatIdx;
        } else if( !strcmp( token, "point" ) ){
            float x,y,z,nx,ny,nz;
            sscanf ( next, "%e %e %e %e %e %e", &x, &y, &z, &nx, &ny, &nz );
            Vector3D<float> n(nx,ny,nz);
            Vector3D<float> v(x,y,z);

            obj->_vertices.push_back(v);
            obj->_normals.push_back(n);
            nb_points++;
            if(nb_points%3==0){
                obj->addTriangle(IndexedTriangle<>(nb_points-3,nb_points-2,nb_points-1));
                //obj->_faces.push_back( IndexedTriangle<>(nb_points-3,nb_points-2,nb_points-1) );
                //mface->_subFaces.push_back(obj->_faces.back());
            }
        } else {
            setlocale(LC_ALL, locale.c_str());
            RW_THROW("unrecognized keyword " << "'" << token << "'");
        }
    }
    //obj->_matFaces.push_back(mface);
    delete mface;

    // order stuff in matrial faces

    model->addObject(obj);
    model->optimize(35*Deg2Rad);
    setlocale(LC_ALL, locale.c_str());
	return model;
}
