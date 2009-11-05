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

#include "RenderAC3D.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/common/macros.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/math/Constants.hpp>
#include <rw/loaders/ImageFactory.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>

#include <boost/foreach.hpp>


#define AC3D_OBJECT_WORLD 999
#define AC3D_OBJECT_NORMAL 0
#define AC3D_OBJECT_GROUP 1
#define AC3D_OBJECT_LIGHT 2

#define AC3D_SURFACE_SHADED (1<<4)
#define AC3D_SURFACE_TWOSIDED (1<<5)

#define AC3D_SURFACE_TYPE_POLYGON (0)
#define AC3D_SURFACE_TYPE_CLOSEDLINE (1)
#define AC3D_SURFACE_TYPE_LINE (2)

#define MAX_ANGLE 45*Deg2Rad //in rad

#define streq(a,b)  ( (*(a)==*(b)) && !strcmp(a,b) )

using namespace rwlibs::drawable;
using namespace rw::math;
using namespace rw::common;
/*
enum { } Token;

namespace {

    toToken(char *s, )

}

*/

RenderAC3D::RenderAC3D(const std::string& filename):
	_displayListId(0)
{
//    const int BUFF_SIZE = 4096*16;
//    char mybuffer[BUFF_SIZE];
    std::ifstream in(filename.c_str());
    if (!in.is_open())
        RW_THROW("Can't open file " << StringUtil::quote(filename));
//    in.rdbuf()->pubsetbuf(mybuffer,BUFF_SIZE);

    _currentDir = StringUtil::getDirectoryName(filename);

    initialize(in, 1.0);
}

RenderAC3D::RenderAC3D(std::istream& in):
	_displayListId(0)
{
    initialize(in, 1.0);
}

RenderAC3D::~RenderAC3D()
{
    delete _object;
}

void RenderAC3D::initialize(std::istream& in, float alpha)
{
    RW_ASSERT(!in.bad());

    std::string line;
    in >> line;

    if (line != "AC3Db") {
        RW_THROW("Data stream does not contain a valid AC3D file.");
    }

    _object = load_object(in, NULL);

    if (_displayListId == 0)
    	_displayListId = glGenLists(1);

    glNewList(_displayListId, GL_COMPILE);
    render(_object, alpha);
    glEndList();
}

void RenderAC3D::draw(DrawType type, double alpha) const{
    switch (type) {
    case Render::SOLID:
    	glPolygonMode(GL_FRONT, GL_FILL);
    	glCallList(_displayListId);
    	break;
    case Render::OUTLINE: // Draw nice frame
    	glPolygonMode(GL_FRONT, GL_FILL);
    	glCallList(_displayListId);
    case Render::WIRE:
    	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    	glCallList(_displayListId);
    	break;
    }
}

void RenderAC3D::col_set(long matno, float alphaVal) const {
    AC3DMaterial& m = const_cast<AC3DMaterial&>(_materials.at(matno));

    float alpha = 1.0f - m.transparency;

    if(alphaVal!=1){
        alpha -= 1.0f-alphaVal;
        if(alpha<0.1f)
            alpha = 0.1f;
    }

    m.rgb[3] = alpha;
    glColor4fv(m.rgb);

    glMaterialfv(GL_FRONT, GL_DIFFUSE, m.rgb);
    glMaterialfv(GL_FRONT, GL_AMBIENT, m.ambient);

    glMaterialfv(GL_FRONT, GL_EMISSION, m.emissive);
    glMaterialfv(GL_FRONT, GL_SPECULAR, m.specular);
    glMaterialf(GL_FRONT, GL_SHININESS, m.shininess);
}

void RenderAC3D::col_set_simple(long matno, float alpha) const {
    AC3DMaterial& m = const_cast<AC3DMaterial&>(_materials.at(matno));
    glColor4fv(m.rgb);
}

void RenderAC3D::render(AC3DObject* ob, float alpha) const
{
    glPushMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glTranslated(ob->loc(0), ob->loc(1), ob->loc(2));
    // TODO: should the rotation not also be added here?

    //if (ob->texture != -1) {
    //    RW_WARN("In AC3D file: Textures not supported yet!");
    //}

    if (ob->texture != -1){
        static int lasttextureset = -1;
        //ACImage *i = ac_get_texture(ob->texture);

        // getTexture(ob->texture);
        glEnable(GL_TEXTURE_2D);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_NEAREST );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_NEAREST );

        if (ob->texture != lasttextureset){

            glBindTexture(GL_TEXTURE_2D, ob->texture);
            lasttextureset = ob->texture;
        }

        if( GLenum error = glGetError()>0 )
            std::cout << "GL Error: " << error << "  at " << __LINE__ << std::endl;

    }

    //AC3DObject::MatSurfMap::iterator matIter;
    //for(matIter = ob->_matToSurfArray.begin(); matIter != ob->_matToSurfArray.end(); ++matIter){
    //std::pair<int,int> key = matIter->first;
//    int matId = key.first;
//    int flag = key.second;
//    std::vector<int>& surfIds = matIter->second;

//    if(surfIds.size()==0)
//        continue;
    int lastMat = -1;
    bool wasLastMatTri = false;
    for(size_t i=0; i<ob->surfaces.size(); i++){
        const AC3DSurface& surface = ob->surfaces[i];
    	// draw material once depending on the draw state
        int matId = surface.mat;
        if( matId!=lastMat ){
            lastMat = matId;
            if(wasLastMatTri){
                glEnd(); // GL_TRIANGLES
                wasLastMatTri = false;
            }

            int st = surface.flags & 0xf;
            if (st == AC3D_SURFACE_TYPE_CLOSEDLINE) {
                glDisable(GL_LIGHTING);
                glBegin(GL_LINE_LOOP);
                col_set_simple(matId, alpha);
            } else if (st == AC3D_SURFACE_TYPE_LINE) {
                glDisable(GL_LIGHTING);
                glBegin(GL_LINE_STRIP);
                col_set_simple(matId, alpha);
            } else {
                glEnable(GL_LIGHTING);
                col_set(matId, alpha);
                // Since we can encounter polygons instead of triangles handle it
                if (surface.vertref_cnt == 3){
                    glBegin(GL_TRIANGLES);
                    wasLastMatTri = true;
                }
            }
        }

        if( surface.vertref_cnt != 3){
            glBegin(GL_POLYGON);
        }

        if (surface.flags & AC3D_SURFACE_TWOSIDED){
            glDisable(GL_CULL_FACE);
        } else {
            glEnable(GL_CULL_FACE);
        }

        if (surface.flags & AC3D_SURFACE_SHADED) {
            // use vertex normals in all vertices
            for (int sr = 0; sr < surface.vertref_cnt; sr++) {
                int vIdx = surface.vertrefs[sr];

                if (ob->texture > -1){

                    float tu = surface.uvs[sr][0];//.u;
                    float tv = surface.uvs[sr][1];//.v;

                    float tx = ob->texture_offset_x + tu * ob->texture_repeat_x;
                    float ty = ob->texture_offset_y + tv * ob->texture_repeat_y;

                    glTexCoord2f(tx, ty);
                }

                glNormal3fv(surface.normals[sr].val);
                glVertex3fv(ob->vertices[vIdx].val);
            }
        } else {
            // use the surface normal in all vertices
            glNormal3fv(surface.normal.val);
            for (int sr = 0; sr < surface.vertref_cnt; sr++) {
                if (ob->texture > -1){
                    float tu = surface.uvs[sr](0);//.u;
                    float tv = surface.uvs[sr](1);//.v;

                    float tx = ob->texture_offset_x + tu * ob->texture_repeat_x;
                    float ty = ob->texture_offset_y + tv * ob->texture_repeat_y;

                    glTexCoord2f(tx, ty);
                }
                glVertex3fv(ob->vertices[surface.vertrefs[sr]].val);
            }
        }

        if( surface.vertref_cnt != 3){
            glEnd(); // GL_POLYGON
        }
    }

    if( wasLastMatTri ){
        glEnd(); // GL_TRIANGLES
    }

    if( GLenum error = glGetError()>0 )
        std::cout << "GL Error: " << error << "  at " << __LINE__ << std::endl;

    if (ob->texture != -1 ){
        glDisable(GL_TEXTURE_2D);
    }

    if( GLenum error = glGetError()>0 )
        std::cout << "GL Error: " << error << "  at " << __LINE__ << std::endl;

    for (int n = 0; n < ob->num_kids; n++) {
        render(ob->kids[n], alpha);
    }

    glPopAttrib();
    glPopMatrix();
}

RenderAC3D::AC3DMaterial RenderAC3D::read_material(std::istream& in)
{
    char buff[256];
    AC3DMaterial m;

    // Read the material name
    // the name is within quotes and contain whitespace
    //in >> token;
    in.getline(buff, 256, '"');
    in.getline(buff, 256, '"');
    std::string token(buff);

    // Remove quotes
    //token = token.erase(0, 1);

    //m.name = token.erase(token.length() - 1, 1);
    m.name = token;
    // Read the RGB color
    in >> token;
    if (token == "rgb") {
        in >> (m.rgb[0]);
        in >> (m.rgb[1]);
        in >> (m.rgb[2]);
    } else {
        RW_THROW(
            "Expected \"rgb\" token not found - " << token);
    }

    // Read the Ambient color
    in >> token;
    if (token == "amb") {
        in >> (m.ambient[0]);
        in >> (m.ambient[1]);
        in >> (m.ambient[2]);
    } else {
        RW_THROW("Expected \"amb\" token not found");
    }

    // Read the Emissive Color
    in >> token;
    if (token == "emis") {
        in >> (m.emissive[0]);
        in >> m.emissive[1];
        in >> m.emissive[2];
    } else {
        RW_THROW("Expected \"emis\" token not found");
    }

    // Read the Specular Color
    in >> token;
    if (token == "spec") {
        in >> m.specular[0];
        in >> m.specular[1];
        in >> m.specular[2];
    } else
        RW_THROW("Expected \"spec\" token not found");

    // Read the shininess
    in >> token;
    if (token == "shi")
        in >> m.shininess;
    else
        RW_THROW("Expected \"shi\" token not found");

    // Read the transparency
    in >> token;
    if (token == "trans")
        in >> m.transparency;
    else
        RW_THROW("Expected \"trans\" token not found");
    return m;
}

RenderAC3D::OBJECT_TYPE RenderAC3D::string_to_objecttype(const std::string& s)
{
    if (s == "world")
        return OBJECT_WORLD;
    if (s == "group")
        return OBJECT_GROUP;
    if (s == "light")
        return OBJECT_LIGHT;
    return OBJECT_NORMAL;
}

void RenderAC3D::read_data(std::istream& in, std::vector<char>& out)
{
    int len;
    in >> len;

    // The data is on the next line, therefore we might have to add something to
    // read carry return and linefeed.
    out.resize(len);
    for (int i = 0; i < len; i++)
        in >> out[i];
}

void RenderAC3D::read_surface(std::istream& in, AC3DSurface& surf, AC3DObject* ob)
{
    char buff[256];
    char token[60],value[60];
    //std::string token;
    while (!in.eof()) {
        in.getline(buff,256);

        if(in.fail()){
            RW_WARN("FAiled in read surface!");
            continue;
        }

        //in >> token;
        int res = sscanf(buff, "%s %s", token, value);
        if(res!=2)
            continue;

        //if (token == "SURF") {
        if ( streq(token, "SURF") ) {
            //in >> token;
            //sscanf(&buff[offset],"%s",token);
            //surf.flags = strtol(token.c_str(), NULL, 16);
            surf.flags = strtol(value, NULL, 16);
        }
        //else if (token == "mat") {
        else if ( streq(token, "mat") ) {
            sscanf(value,"%d",&surf.mat);
        }
        //else if (token == "refs") {
        else if( streq(token, "refs") ){
            //in >> surf.vertref_cnt;
            sscanf(value,"%d",&surf.vertref_cnt);
            surf.vertrefs.resize(surf.vertref_cnt);
            surf.normals.resize(surf.vertref_cnt);
            surf.uvs.resize(surf.vertref_cnt);

            for (int i = 0; i < surf.vertref_cnt; i++) {
                in.getline(buff,256);
                int ref;
                float uvs0,uvs1;
                sscanf(buff, "%d %f %f\n", &ref, &uvs0, &uvs1 );
                surf.vertrefs[i] = ref;
                surf.uvs[i](0) = uvs0;
                surf.uvs[i](1) = uvs1;
                //in >> surf.vertrefs[i] >> surf.uvs[i](0) >> surf.uvs[i](1);
                //in >> ;
                //in ;
            }
            if (surf.vertref_cnt >= 3) {
                const Vector3D<float>& v1 = ob->vertices[surf.vertrefs[0]].toV3D();
                const Vector3D<float>& v2 = ob->vertices[surf.vertrefs[1]].toV3D();
                const Vector3D<float>& v3 = ob->vertices[surf.vertrefs[2]].toV3D();
                surf.normal =
                    normalize(
                        cross(
                            Vector3D<float>(v2 - v1),
                            Vector3D<float>(v3 - v1)));
            }
            return;
        }
        else{
            std::cout << "Token: " << std::string(token) << " Value: " << std::string(value) << std::endl;
            RW_WARN("In AC3D file: Ignoring string ");
        }
    }
}

RenderAC3D::AC3DObject* RenderAC3D::load_object(
    std::istream& in, AC3DObject* parent)
{
    std::string token;
    AC3DObject* ob = NULL;
    while (!in.eof()) {

        in >> token;

        if( in.fail() ){
            RW_WARN("Fail bit set on stream!");
            TimerUtil::sleepMs(100);
            continue;
        }

        switch (token[0]){
        case('c'):
            if( token == "crease"){
                double tmp;
                in>>tmp;
            }
            break;
        case('d'):
            if (token == "data") {
                read_data(in, ob->data);
            }
        break;
        case('k'):
            if (token == "kids") {
                in >> ob->num_kids;
                if (ob->num_kids > 0) {
                    ob->kids.resize(ob->num_kids);
                    for (int i = 0; i < ob->num_kids; i++) {
                        AC3DObject* k = load_object(in, ob);
                        if (k != NULL) {
                            ob->kids[i] = k;
                        } else {
                            RW_THROW(
                                "Could not find the specified kid!");
                        }
                    }
                }
                calc_vertex_normals(ob);
                return ob;
            }
        break;
        case('l'):
            if (token == "loc") {
                in >> ob->loc(0);
                in >> ob->loc(1);
                in >> ob->loc(2);
            }
        break;
        case('M'):
            if (token == "MATERIAL") {
                _materials.push_back(read_material(in));
            }
        break;
        case('n'):
            if (token == "name") {
                in >> ob->name;
                ob->name = ob->name.substr(1, ob->name.length()-2);
            } else if (token == "numvert") {
                in >> ob->vertex_cnt;
                ob->vertices.resize(ob->vertex_cnt);
                // std::cout << "- Reading vertices: " << ob->surf_cnt << std::endl;
                // long time = TimerUtil::currentTimeMs();
                char buff[1024];
                in.getline(buff,256);
                for (int i = 0; i < ob->vertex_cnt; i++) {
                    in.getline(buff,1024);
                    float v0=10,v1=10,v2=10;
                    sscanf(buff, "%f %f %f", &v0, &v1, &v2 );
                    ob->vertices[i].val[0] = v0;
                    ob->vertices[i].val[1] = v1;
                    ob->vertices[i].val[2] = v2;

                    //in >> ob->vertices[i].val[0];
                    //in >> ob->vertices[i].val[1];
                    //in >> ob->vertices[i].val[2];
                }
                // long timel = TimerUtil::currentTimeMs();
                // std::cout << "- time: " << timel-time << std::endl;

            } else if (token == "numsurf") {
                in >> ob->surf_cnt;
                ob->surfaces.resize(ob->surf_cnt);
                // std::cout << "- Reading surfaces: " << ob->surf_cnt << std::endl;
                // long time = TimerUtil::currentTimeMs();
                for (int i = 0; i < ob->surf_cnt; i++) {
                    read_surface(in, ob->surfaces[i], ob);
                }
                // long timel = TimerUtil::currentTimeMs();
                // std::cout << "- time: " << timel-time << std::endl;
            }
        break;
        case('O'):
            if (token == "OBJECT") {
                ob = new AC3DObject();
                in >> token;
                ob->type = string_to_objecttype(token);
            }
        break;
        case('r'):
            if (token == "rot") {
                in >> ob->rot(0,0);
                in >> ob->rot(0,1);
                in >> ob->rot(0,2);

                in >> ob->rot(1,0);
                in >> ob->rot(1,1);
                in >> ob->rot(1,2);

                in >> ob->rot(2,0);
                in >> ob->rot(2,1);
                in >> ob->rot(2,2);
            }
        break;
        case('t'):
            if (token == "texture") {
                std::string filename;
                in >> filename;
                filename = filename.substr(1,filename.length()-2);
                ob->texture = loadTexture( _currentDir+filename );
                std::cout << "Texture with name: " << ob->name << " has ID: " << ob->texture << std::endl;
                //RW_WARN("In AC3D file: Textures not supported yet!");
            } else if (token == "texrep") {
                std::cout << "Setting texture repeat!" << std::endl;
                in >> ob->texture_repeat_x;
                in >> ob->texture_repeat_y;
            } else if (token == "texoff") {
                std::cout << "Setting texture offset!" << std::endl;
                in >> ob->texture_offset_x;
                in >> ob->texture_offset_y;
            }
        break;
        case('u'):
            if (token == "url") {
                in >> ob->url;
            }
        break;
        default:
            RW_WARN("RenderAC3D: UNKNOWN token!! ");
        }
    }
    calc_vertex_normals(ob);
    return ob;
}

void RenderAC3D::calc_vertex_normals(AC3DObject *ob)
{
    // std::cout << "---------------- calc_vertex_normals: " << std::endl;
    // std::cout << "- vertex cnt: " << ob->vertex_cnt << std::endl;
    // std::cout << "- surface cnt: " << ob->surf_cnt << std::endl;
    /// long time = TimerUtil::currentTimeMs();
	// create vertexSurfaceNeigh map and matToSurfArray
	// run through all surfaces and add them to the vertex index
	ob->_vertSurfMap.resize( ob->vertex_cnt );
	for (int sIdx = 0; sIdx < ob->surf_cnt; sIdx++) {
		AC3DSurface *surf = &(ob->surfaces[sIdx]);

		//std::pair<int,int> key(surf->mat,surf->flags & 0xf);
    	//ob->_matToSurfArray[ key ].push_back(sIdx);

        for (int vrIdx = 0; vrIdx < surf->vertref_cnt; vrIdx++) {
        	ob->_vertSurfMap[ surf->vertrefs[vrIdx] ].push_back(surf);
        }
	}

	// use vertexSurfaceNeigh map to calculate per vertex normals
	Vector3D<float> n(0.0, 0.0, 0.0);
    for (int vIdx = 0; vIdx < ob->vertex_cnt; vIdx++) {
        BOOST_FOREACH(AC3DSurface *surf, ob->_vertSurfMap[vIdx]){

        	// for each surface calculate the vertex normal considering
        	// the angle between two surfaces
			Vector3D<float> n1 = surf->normal.toV3D();
			n = n1;
        	BOOST_FOREACH(AC3DSurface *nsurf, ob->_vertSurfMap[vIdx]){
				if(nsurf==surf)
					continue;
				// angle between surf normals must be less than MAX_ANGLE
				Vector3D<float> n2 = nsurf->normal.toV3D();
				if( dot(n1,n2) < cos(MAX_ANGLE) )
					continue;
				// if it is then add
	        	n += n2;
        	}
        	// now be sure to update the correct vertex normal in the surface
        	for(int i=0; i<surf->vertref_cnt; i++){
        		if( surf->vertrefs[i]==vIdx){
        			surf->normals[i] = normalize(n);
        			break;
        		}

        	}
        }
    }
    // long timel = TimerUtil::currentTimeMs();
    // std::cout << "- time: " << timel-time << "ms" << std::endl;

}

int RenderAC3D::loadTexture(const std::string& filename){
    std::cout << "LOADING TEXTURE: " << filename <<std::endl;
    rw::sensor::ImagePtr image;
    try{
        image = rw::loaders::ImageFactory::load( filename );
    } catch(...){
        return -1;
    }

    if( image==NULL )
        return -1;

    std::cout << "creating TEXTURE: " << std::endl;
    RWGLTexture *texture = new RWGLTexture( *image );

    _textures.push_back(texture);
    _textureMap[texture->getTextureID()] = texture;

    std::cout << "TEXTURE ID: " << texture->getTextureID() <<std::endl;

    return texture->getTextureID();
}

#ifdef VERY_INEFFICIENT_WAY
void RenderAC3D::calc_object_vertex_normals(AC3DObject* ob)
{
    int s, v, vr;
    /** for each vertex in this object **/
    for (v = 0; v < ob->vertex_cnt; v++) {
        Vector3D<float> n(0.0, 0.0, 0.0);
        int found = 0;

        /** go through each surface **/
        for (s = 0; s < ob->surf_cnt; s++) {
            AC3DSurface *surf = &ob->surfaces[s];

            /** check if this vertex is used in this surface **/
            /** if it is, use it to create an average normal **/
            for (vr = 0; vr < surf->vertref_cnt; vr++) {
                if (surf->vertrefs[vr] == v) {
                    n(0)+=surf->normal(0);
                    n(1)+=surf->normal(1);
                    n(2)+=surf->normal(2);
                    found++;
                }
            }
        }
        if (found > 0) {
            n *= 1.0f / found;
        }
        ob->vertices[v].normal = n;
    }
}
#endif
