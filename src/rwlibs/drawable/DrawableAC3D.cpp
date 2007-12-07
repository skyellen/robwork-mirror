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
#include "DrawableAC3D.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/common/macros.hpp>

#include <fstream>

#define AC3D_OBJECT_WORLD 999
#define AC3D_OBJECT_NORMAL 0
#define AC3D_OBJECT_GROUP 1
#define AC3D_OBJECT_LIGHT 2

#define AC3D_SURFACE_SHADED (1<<4)
#define AC3D_SURFACE_TWOSIDED (1<<5)

#define AC3D_SURFACE_TYPE_POLYGON (0)
#define AC3D_SURFACE_TYPE_CLOSEDLINE (1)
#define AC3D_SURFACE_TYPE_LINE (2)

using namespace rwlibs::drawable;
using namespace rw::math;
using namespace rw::common;

namespace {
    
    
}



DrawableAC3D::DrawableAC3D(const std::string& filename)
{
    const int BUFF_SIZE = 4096;
    char mybuffer[BUFF_SIZE];    
    std::ifstream in(filename.c_str());
    if (!in.is_open())
        RW_THROW("Can't open file " << StringUtil::Quote(filename));
    in.rdbuf()->pubsetbuf(mybuffer,BUFF_SIZE);
    initialize(in);
}

DrawableAC3D::DrawableAC3D(std::istream& in)
{
    initialize(in);
}

DrawableAC3D::~DrawableAC3D()
{
    delete _object;
}

void DrawableAC3D::initialize(std::istream& in)
{
    RW_ASSERT(!in.bad());

    std::string line;
    in >> line;

    if (line != "AC3Db") {
        RW_THROW("Data stream does not contain a valid AC3D file.");
    }    
    
    _object = load_object(in, NULL);
    update(CUSTOM);
}

void DrawableAC3D::update(UpdateType type)
{
    if (type == CUSTOM || type == ALPHA) {
        if (_displayListId != 0)
            glDeleteLists(_displayListId,1);
        _displayListId = glGenLists(1);
        glNewList(_displayListId, GL_COMPILE);

        render(_object);

        glEndList();
    }
}

void DrawableAC3D::col_set(long matno) const {
    AC3DMaterial& m = const_cast<AC3DMaterial&>(_materials.at(matno));

    float alpha = 1.0f - m.transparency;

    if(_alpha!=1){
        alpha -= 1.0f-_alpha;
        if(alpha<0.1f)
            alpha = 0.1f;
    }

    GLfloat diffuse[4] = {m.rgb(0), m.rgb(1), m.rgb(2), alpha};
    GLfloat ambient[4] = {m.rgb(0), m.rgb(1), m.rgb(2), alpha};
    GLfloat emissive[4] = {m.emissive(0), m.emissive(1), m.emissive(2), 1.0};
    GLfloat specular[4] = {m.specular(0), m.specular(1), m.specular(2), 1.0};

    glColor4f(m.rgb(0), m.rgb(1), m.rgb(2), alpha);

    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);

    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emissive);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, m.shininess);
}

void DrawableAC3D::col_set_simple(long matno) const {
    AC3DMaterial& m = const_cast<AC3DMaterial&>(_materials.at(matno));
    glColor3f(m.rgb(0), m.rgb(1), m.rgb(2));
}

void DrawableAC3D::render(AC3DObject* ob) const
{
    glPushMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glTranslated(ob->loc(0), ob->loc(1), ob->loc(2));

    if (ob->texture != -1) {
        RW_WARN("In AC3D file: Textures not supported yet!");
    }
    for (int s = 0; s < ob->surf_cnt; s++) {
        AC3DSurface* surf = &ob->surfaces[s];

        glNormal3f(surf->normal(0), surf->normal(1), surf->normal(2));
        if (surf->flags & AC3D_SURFACE_TWOSIDED)
            glDisable(GL_CULL_FACE);
        else
            glEnable(GL_CULL_FACE);

        int st = surf->flags & 0xf;
        if (st == AC3D_SURFACE_TYPE_CLOSEDLINE) {
            glDisable(GL_LIGHTING);
            glBegin(GL_LINE_LOOP);
            col_set_simple(surf->mat);
        }
        else if (st == AC3D_SURFACE_TYPE_LINE) {
            glDisable(GL_LIGHTING);
            glBegin(GL_LINE_STRIP);
            col_set_simple(surf->mat);
        } else {
            glEnable(GL_LIGHTING);
            col_set(surf->mat);
            if (surf->vertref_cnt == 3)
                glBegin(GL_TRIANGLE_STRIP);
            else
                glBegin(GL_POLYGON);
        }

        for (int sr = 0; sr < surf->vertref_cnt; sr++) {
            AC3DVertex* v = &ob->vertices[surf->vertrefs[sr]];

            if (ob->texture > -1) {
                RW_WARN("In AC3D file: Textures not supported yet!");
            }
            if (surf->flags & AC3D_SURFACE_SHADED) {
                glNormal3f(v->normal(0), v->normal(1), v->normal(2));
            }
            glVertex3f(v->loc(0), v->loc(1), v->loc(2));

        }
        glEnd();
    }

    for (int n = 0; n < ob->num_kids; n++) {
        render(ob->kids[n]);
    }

    glPopAttrib();
    glPopMatrix();
}

DrawableAC3D::AC3DMaterial DrawableAC3D::read_material(std::istream& in)
{
    AC3DMaterial m;
    std::string token;

    // Read the material name
    in >> token;

    // Remove quotes
    token = token.erase(0, 1);
    m.name = token.erase(token.length() - 1, 1);

    // Read the RGB color
    in >> token;
    if (token == "rgb") {
        in >> m.rgb(0);
        in >> m.rgb(1);
        in >> m.rgb(2);
    } else {
        RW_THROW(
            "Expected \"rgb\" token not found");
    }

    // Read the Ambient color
    in >> token;
    if (token == "amb") {
        in >> m.ambient(0);
        in >> m.ambient(1);
        in >> m.ambient(2);
    } else {
        RW_THROW("Expected \"amb\" token not found");
    }

    // Read the Emissive Color
    in >> token;
    if (token == "emis") {
        in >> m.emissive(0);
        in >> m.emissive(1);
        in >> m.emissive(2);
    } else {
        RW_THROW("Expected \"emis\" token not found");
    }

    // Read the Specular Color
    in >> token;
    if (token == "spec") {
        in >> m.specular(0);
        in >> m.specular(1);
        in >> m.specular(2);
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

DrawableAC3D::OBJECT_TYPE DrawableAC3D::string_to_objecttype(const std::string& s)
{
    if (s == "world")
        return OBJECT_WORLD;
    if (s == "group")
        return OBJECT_GROUP;
    if (s == "light")
        return OBJECT_LIGHT;
    return OBJECT_NORMAL;
}

void DrawableAC3D::read_data(std::istream& in, std::vector<char>& out)
{
    int len;
    in >> len;

    // The data is on the next line, therefore we might have to add something to
    // read carry return and linefeed.
    out.resize(len);
    for (int i = 0; i < len; i++)
        in >> out[i];
}

void DrawableAC3D::read_surface(std::istream& in, AC3DSurface& surf, AC3DObject* ob)
{
    std::string token;
    while (!in.eof()) {
        in >> token;
        if (token == "SURF") {
            in >> token;
            surf.flags = strtol(token.c_str(), NULL, 16);

        }
        else if (token == "mat") {
            in >> surf.mat;
        }
        else if (token == "refs") {
            in >> surf.vertref_cnt;
            surf.vertrefs.resize(surf.vertref_cnt);
            surf.uvs.resize(surf.vertref_cnt);
            for (int i = 0; i < surf.vertref_cnt; i++) {
                in >> surf.vertrefs[i];
                in >> surf.uvs[i](0);
                in >> surf.uvs[i](1);
            }

            if (surf.vertref_cnt >= 3) {
                const Vector3D<float>& v1 = ob->vertices[surf.vertrefs[0]].loc;
                const Vector3D<float>& v2 = ob->vertices[surf.vertrefs[1]].loc;
                const Vector3D<float>& v3 = ob->vertices[surf.vertrefs[2]].loc;
                surf.normal =
                    normalize(
                        cross(
                            Vector3D<float>(v2 - v1),
                            Vector3D<float>(v3 - v1)));
            }
            return;
        }
        else
            RW_WARN("In AC3D file: Ignoring string " << StringUtil::Quote(token));
    }
}

DrawableAC3D::AC3DObject* DrawableAC3D::load_object(
    std::istream& in, AC3DObject* parent)
{
    std::string token;
    AC3DObject* ob = NULL;
    while (!in.eof()) {
        in>>token;
        
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
            } else if (token == "numvert") {
                in >> ob->vertex_cnt;
                ob->vertices.resize(ob->vertex_cnt);
                for (int i = 0; i < ob->vertex_cnt; i++) {
                    in >> ob->vertices[i].loc(0);
                    in >> ob->vertices[i].loc(1);
                    in >> ob->vertices[i].loc(2);
                }
            } else if (token == "numsurf") {
                in >> ob->surf_cnt;
                ob->surfaces.resize(ob->surf_cnt);
                for (int i = 0; i < ob->surf_cnt; i++) {
                    read_surface(in, ob->surfaces[i], ob);
                }
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
                RW_WARN("In AC3D file: Textures not supported yet!");
            } else if (token == "texrep") {
                in >> ob->texture_repeat_x;
                in >> ob->texture_repeat_y;
            } else if (token == "texoff") {
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
            RW_WARN("DrawableAC3D: UNKNOWN token!! " << StringUtil::Quote(token));
        }
    }

    calc_vertex_normals(ob);
    return ob;
}

void DrawableAC3D::calc_object_vertex_normals(AC3DObject* ob)
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

void DrawableAC3D::calc_vertex_normals(AC3DObject *ob)
{
    calc_object_vertex_normals(ob);
    if (ob->num_kids)
        for (int n = 0; n < ob->num_kids; n++)
            calc_vertex_normals(ob->kids[n]);
}
