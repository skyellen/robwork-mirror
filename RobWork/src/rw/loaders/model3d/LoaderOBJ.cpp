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

#include "LoaderOBJ.hpp"

#include <fstream>
#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>
#include <boost/foreach.hpp>
#include <rw/geometry/IndexedPolygon.hpp>
#include <rw/geometry/Triangulate.hpp>
#include <rw/math/EAA.hpp>

using namespace rw::loaders;
using namespace rw::graphics;
using namespace rw::math;
using namespace rw::common;
using namespace rw::geometry;


namespace rw {
namespace loaders {

/**
 * @brief Class for loading in OBJ files
 */
class OBJReader
{
public:
	char* rwStrtok(char *_Str, const char *_Delim, char **_Context)
	{
		int numDelim = static_cast<int>(strlen(_Delim));
		if(_Str)
			*_Context = _Str;

		if(!**_Context)
			return NULL;

		bool cont=true;
		while(cont)
		{
			for(int i=0; i<numDelim; i++)
			{
				if(**_Context == _Delim[i])
				{
					(*_Context)++;
					cont = true;
				}
				else
					cont = false;
			}
		}

		_Str = *_Context;

		while(**_Context)
		{
			for(int i=0; i<numDelim; i++)
			{
				if(**_Context == _Delim[i])
				{
					**_Context = 0;
					(*_Context)++;
					return _Str;
				}
			}
			(*_Context)++;
		}

		if(!*_Str)
			return NULL;

		return _Str;
	}

	struct Vec3
	{
	public:
		float _v[3];

		Vec3(float v1, float v2, float v3)
		{
			_v[0] = v1;
			_v[1] = v2;
			_v[2] = v3;
		}
	};

	struct IVec3
	{
	public:
		int _v[3];

		IVec3(int i1, int i2, int i3)
		{
			_v[0] = i1;
			_v[1] = i2;
			_v[2] = i3;
		}
	};

	struct Vec4
	{
	public:
		float _v[4];
		Vec4(float v1, float v2, float v3, float v4)
		{
			_v[0] = v1;
			_v[1] = v2;
			_v[2] = v3;
			_v[3] = v4;
		}
	};

	struct Mtl
	{
	public:
		std::string _name;
		Vec4 _Ka;
		Vec4 _Kd;
		Vec4 _Ks;
		float _Ns;
		Vec4 _Tf;
		float _d;

		Mtl()
		: _Ka(0.2f, 0.2f, 0.2f, 1.0f),
		  _Kd(0.8f, 0.8f, 0.8f, 1.0f),
		  _Ks(1.0f, 1.0f, 1.0f, 1.0f),
		  _Ns(0),
		  _Tf(0.0f, 0.0f, 0.0f, 1.0f),
		  _d(1.0f)
		{}

		void Write(std::ostream &out)
		{
			out << "newmtl " << _name << std::endl;
			out << "Ka " << _Ka._v[0] << " " << _Ka._v[1] << " " << _Ka._v[2] << std::endl;
			out << "Kd " << _Kd._v[0] << " " << _Kd._v[1] << " " << _Kd._v[2] << std::endl;
			out << "Ks " << _Ks._v[0] << " " << _Ks._v[1] << " " << _Ks._v[2] << std::endl;
			out << "Tf " << _Tf._v[0] << " " << _Tf._v[1] << " " << _Tf._v[2] << std::endl;
			out << "d " << _d << std::endl;
			out << "Ns " << _Ns << std::endl;
		}
	};

	class RenderItem
	{
	public:
		virtual void write(std::ostream &out)=0;
	};

	class Face : public RenderItem
	{
	public:
		OBJReader *_objReader;

	public:
		rw::math::Vector3D<float> _vCommonNorm;
		std::vector<IVec3> _element;

		Face(OBJReader *objReader) : _objReader(objReader) {}

		virtual void write(std::ostream &out)
		{
			out << "f";
			std::vector<IVec3>::iterator it;
			for(it=_element.begin(); it!=_element.end(); it++)
			{
				out << " " << it->_v[0];
				if(it->_v[1] != -1)
				{
					out << "/" << it->_v[1] << "/";
					if(it->_v[2] != -1)
						out << it->_v[2];
				}
				else if(it->_v[2] != -1)
				{
					out << "//" << it->_v[2];
				}
			}
			out << std::endl;
		}
		void calcCommonNormal();
	};

	class UseMaterial : public RenderItem
	{
	public:
		Mtl *_material;

	public:
		UseMaterial(Mtl *material) : _material(material) {}
		virtual void write(std::ostream &out)
		{
			out << "usemtl " << _material->_name << std::endl;
		}
	};

public:
	OBJReader();
	~OBJReader();

	void load(const std::string& fileName);
	void render(float alpha) const;
	void scale(float factor);
	void writeFile(const std::string& filename) const;
	void calcVertexNormals();


	std::string _dirName;
	std::vector<Vec3> _geoVertices;
	std::vector<Vec3> _textVertices;
	std::vector<Vec3> _vertexNormals;
	std::vector<RenderItem*> _renderItems;
	std::map<std::string, Mtl*> _materials;

	std::map<double, std::vector<Vec3*> > _yMap;

private:
	void parseMtlFile(const std::string& fileName);
	void writeMtlFile(const std::string& filename) const;

	typedef void (OBJReader::*FuncPtr)(char **next_token);
	std::map<std::string, FuncPtr> _objTypeMap;

	void parse_v(char **next_token);
	void parse_vt(char **next_token);
	void parse_vn(char **next_token);
	void parse_face(char **next_token);
	void parse_g(char **next_token) { RW_WARN("obj type 'g' not implemented"); }
	void parse_usemtl(char **next_token);
	void parse_mtllib(char **next_token);
	void parse_vp(char **next_token) { RW_WARN("obj type 'vp' not implemented"); }
	void parse_cstype(char **next_token) { RW_WARN("obj type 'cstype' not implemented"); }
	void parse_deg(char **next_token) { RW_WARN("obj type 'deg' not implemented"); }
	void parse_bmat(char **next_token) { RW_WARN("obj type 'bmat' not implemented"); }
	void parse_step(char **next_token) { RW_WARN("obj type 'step' not implemented"); }
	void parse_p(char **next_token) { RW_WARN("obj type 'p' not implemented"); }
	void parse_l(char **next_token) { RW_WARN("obj type 'l' not implemented"); }
	void parse_curv(char **next_token) { RW_WARN("obj type 'curv' not implemented"); }
	void parse_curv2(char **next_token) { RW_WARN("obj type 'curv2' not implemented"); }
	void parse_surf(char **next_token) { RW_WARN("obj type 'surf' not implemented"); }
	void parse_parm(char **next_token) { RW_WARN("obj type 'parm' not implemented"); }
	void parse_hole(char **next_token) { RW_WARN("obj type 'hole' not implemented"); }
	void parse_scrv(char **next_token) { RW_WARN("obj type 'scrv' not implemented"); }
	void parse_sp(char **next_token) { RW_WARN("obj type 'sp' not implemented"); }
	void parse_end(char **next_token) { RW_WARN("obj type 'end' not implemented"); }
	void parse_con(char **next_token) { RW_WARN("obj type 'con' not implemented"); }
	void parse_mg(char **next_token) { RW_WARN("obj type 'mg' not implemented"); }
	void parse_o(char **next_token) { RW_WARN("obj type 'o' not implemented"); }
	void parse_bevel(char **next_token) { RW_WARN("obj type 'bevel' not implemented"); }
	void parse_c_interp(char **next_token) { RW_WARN("obj type 'c_interp' not implemented"); }
	void parse_d_interp(char **next_token) { RW_WARN("obj type 'd_interp' not implemented"); }
	void parse_lod(char **next_token) { RW_WARN("obj type 'lod' not implemented"); }
	void parse_shadow_obj(char **next_token) { RW_WARN("obj type 'shadow_obj' not implemented"); }
	void parse_trace_obj(char **next_token) { RW_WARN("obj type 'trace_obj' not implemented"); }
	void parse_ctech(char **next_token) { RW_WARN("obj type 'ctech' not implemented"); }
	void parse_stech(char **next_token) { RW_WARN("obj type 'stech' not implemented"); }
	void parse_s(char **next_token) { RW_WARN("obj type 's' not implemented"); }

	typedef void (OBJReader::*MtlFuncPtr)(char **next_token, Mtl **material);
	std::map<std::string, MtlFuncPtr> _mtlTypeMap;

	void parse_mtl_newmtl(char **next_token, Mtl **material);
	void parse_mtl_illum(char **next_token, Mtl **material);
	void parse_mtl_Kd(char **next_token, Mtl **material);
	void parse_mtl_Ka(char **next_token, Mtl **material);
	void parse_mtl_Tf(char **next_token, Mtl **material);
	void parse_mtl_Ni(char **next_token, Mtl **material);
	void parse_mtl_Ks(char **next_token, Mtl **material);
	void parse_mtl_Ns(char **next_token, Mtl **material);
	void parse_mtl_d(char **next_token, Mtl **material);

    void parse_mtl_map_Ka(char **next_token, Mtl **material);
    void parse_mtl_map_Kd(char **next_token, Mtl **material);
    //void parse_mtl_map_Kd(char **next_token, Mtl **material);

	int parseInt(char **next_token);
	float parseFloat(char **next_token);
	float parseOptionalFloat(char **next_token, float defaultVal);
};

}
}


void OBJReader::Face::calcCommonNormal()
{
	if(_element.size() >= 3)
	{
		Vector3D<float> p0(
			_objReader->_geoVertices[_element[0]._v[0]-1]._v[0],
			_objReader->_geoVertices[_element[0]._v[0]-1]._v[1],
			_objReader->_geoVertices[_element[0]._v[0]-1]._v[2]);
		Vector3D<float> p1(
			_objReader->_geoVertices[_element[1]._v[0]-1]._v[0],
			_objReader->_geoVertices[_element[1]._v[0]-1]._v[1],
			_objReader->_geoVertices[_element[1]._v[0]-1]._v[2]);
		Vector3D<float> p2(
			_objReader->_geoVertices[_element[2]._v[0]-1]._v[0],
			_objReader->_geoVertices[_element[2]._v[0]-1]._v[1],
			_objReader->_geoVertices[_element[2]._v[0]-1]._v[2]);
		Vector3D<float> p01 = p0 - p1;
		Vector3D<float> p21 = p2 - p1;
		_vCommonNorm = cross(p21, p01);
		_vCommonNorm = normalize(_vCommonNorm);
	}
}


OBJReader::OBJReader()
{
	_objTypeMap["v"]			= &OBJReader::parse_v;
	_objTypeMap["vt"]			= &OBJReader::parse_vt;
	_objTypeMap["vn"]			= &OBJReader::parse_vn;
	_objTypeMap["f"]			= &OBJReader::parse_face;
	_objTypeMap["g"]			= &OBJReader::parse_g;
	_objTypeMap["usemtl"]		= &OBJReader::parse_usemtl;
	_objTypeMap["mtllib"]		= &OBJReader::parse_mtllib;
	_objTypeMap["vp"]			= &OBJReader::parse_vp;
	_objTypeMap["cstype"]		= &OBJReader::parse_cstype;
	_objTypeMap["deg"]			= &OBJReader::parse_deg;
	_objTypeMap["bmat"]			= &OBJReader::parse_bmat;
	_objTypeMap["step"]			= &OBJReader::parse_step;
	_objTypeMap["p"]			= &OBJReader::parse_p;
	_objTypeMap["l"]			= &OBJReader::parse_l;
	_objTypeMap["curv"]			= &OBJReader::parse_curv;
	_objTypeMap["curv2"]		= &OBJReader::parse_curv2;
	_objTypeMap["surf"]			= &OBJReader::parse_surf;
	_objTypeMap["parm"]			= &OBJReader::parse_parm;
	_objTypeMap["hole"]			= &OBJReader::parse_hole;
	_objTypeMap["scrv"]			= &OBJReader::parse_scrv;
	_objTypeMap["sp"]			= &OBJReader::parse_sp;
	_objTypeMap["end"]			= &OBJReader::parse_end;
	_objTypeMap["con"]			= &OBJReader::parse_con;
	_objTypeMap["mg"]			= &OBJReader::parse_mg;
	_objTypeMap["o"]			= &OBJReader::parse_o;
	_objTypeMap["bevel"]		= &OBJReader::parse_bevel;
	_objTypeMap["c_interp"]		= &OBJReader::parse_c_interp;
	_objTypeMap["d_interp"]		= &OBJReader::parse_d_interp;
	_objTypeMap["lod"]			= &OBJReader::parse_lod;
	_objTypeMap["shadow_obj"]	= &OBJReader::parse_shadow_obj;
	_objTypeMap["trace_obj"]	= &OBJReader::parse_trace_obj;
	_objTypeMap["ctech"]		= &OBJReader::parse_ctech;
	_objTypeMap["stech"]		= &OBJReader::parse_stech;
	_objTypeMap["s"]			= &OBJReader::parse_s;

	_mtlTypeMap["newmtl"]		= &OBJReader::parse_mtl_newmtl;
	_mtlTypeMap["illum"]		= &OBJReader::parse_mtl_illum;
	_mtlTypeMap["Kd"]			= &OBJReader::parse_mtl_Kd;
	_mtlTypeMap["Ka"]			= &OBJReader::parse_mtl_Ka;
	_mtlTypeMap["Tf"]			= &OBJReader::parse_mtl_Tf;
	_mtlTypeMap["Ni"]			= &OBJReader::parse_mtl_Ni;
	_mtlTypeMap["Ks"]			= &OBJReader::parse_mtl_Ks;
	_mtlTypeMap["Ns"]			= &OBJReader::parse_mtl_Ns;
	_mtlTypeMap["d"]			= &OBJReader::parse_mtl_d;
	_mtlTypeMap["Tr"]            = &OBJReader::parse_mtl_d;
    _mtlTypeMap["map_Kd"]           = &OBJReader::parse_mtl_Kd;
    _mtlTypeMap["map_Ka"]           = &OBJReader::parse_mtl_Ka;

}

OBJReader::~OBJReader()
{
	std::map<std::string, Mtl*>::iterator it;
	for(it=_materials.begin(); it!=_materials.end(); it++)
		delete it->second;
}

void OBJReader::parse_mtl_map_Ka(char **next_token, Mtl **material){
    // TODO: implements map_Ka functionality
}

void OBJReader::parse_mtl_map_Kd(char **next_token, Mtl **material){
    // TODO: implements map_Kd functionality
}


void OBJReader::parse_v(char **next_token)
{
    float x = parseFloat(next_token);
    float y = parseFloat(next_token);
    float z = parseFloat(next_token);
    _geoVertices.push_back(Vec3(x, y, z));
}

void OBJReader::parse_vt(char **next_token)
{
    float u = parseFloat(next_token);
    float v = parseOptionalFloat(next_token, 0.0);
    float w = parseOptionalFloat(next_token, 0.0);
    _textVertices.push_back(Vec3(u, v, w));
}

void OBJReader::parse_vn(char **next_token)
{
    float i = parseFloat(next_token);
    float j = parseFloat(next_token);
    float k = parseFloat(next_token);
    _vertexNormals.push_back(Vec3(i, j, k));
}

void OBJReader::parse_face(char **next_token)
{
	Face *face = new Face(this);
	_renderItems.push_back(face);
	char* token;
	while((token = rwStrtok(NULL, " \t", next_token)) != NULL)
	{
		char* p = token;
		bool cont;
		int v, vt=-1, vn=-1;

		// vertex index
		while(*p != 0 && *p!='/') p++;
		cont = *p != 0;
		*p = 0;
		v = atoi(token);
		token = ++p;

		// texture vertex index (optional)
		if(cont)
		{
			while(*p != 0 && *p!='/') p++;
			cont = *p!= 0;
			*p = 0;
			vt = token!=p ? atoi(token) : -1;
			token = ++p;

			// vertex normal index (optional)
			if(cont)
			{
				while(*p != 0) p++;
				*p = 0;
				vn = token!=p ? atoi(token) : -1;
				token = ++p;
			}
		}
		face->_element.push_back(IVec3(v, vt, vn));
	}
}



void OBJReader::parse_usemtl(char **next_token)
{
	char *token = rwStrtok(NULL, "\r\n", next_token);
	std::map<std::string, Mtl*>::iterator it = _materials.find(token);
	if(it == _materials.end())
		RW_THROW("Material '" << token << "' not defined");
	_renderItems.push_back(new UseMaterial(it->second));
}

void OBJReader::parse_mtllib(char **next_token)
{
	char *token;
	while((token = rwStrtok(NULL, " \t", next_token)) != NULL)
		parseMtlFile(_dirName + token);
}

void OBJReader::parse_mtl_newmtl(char **next_token, Mtl **material)
{
	char *token = rwStrtok(NULL, "\r\n", next_token);
	*material = new Mtl();
	_materials[token] = *material;
	(*material)->_name = token;
}

void OBJReader::parse_mtl_illum(char **next_token, Mtl **material)
{
}

void OBJReader::parse_mtl_Kd(char **next_token, Mtl **material)
{
    (*material)->_Kd._v[0] = parseFloat(next_token);
    (*material)->_Kd._v[1] = parseFloat(next_token);
    (*material)->_Kd._v[2] = parseFloat(next_token);
}

void OBJReader::parse_mtl_Ka(char **next_token, Mtl **material)
{
    (*material)->_Ka._v[0] = parseFloat(next_token);
    (*material)->_Ka._v[1] = parseFloat(next_token);
    (*material)->_Ka._v[2] = parseFloat(next_token);
}

void OBJReader::parse_mtl_Tf(char **next_token, Mtl **material)
{
    (*material)->_Tf._v[0] = parseFloat(next_token);
    (*material)->_Tf._v[1] = parseFloat(next_token);
    (*material)->_Tf._v[2] = parseFloat(next_token);
}

void OBJReader::parse_mtl_Ni(char **next_token, Mtl **material)
{
}

void OBJReader::parse_mtl_Ks(char **next_token, Mtl **material)
{
    (*material)->_Ks._v[0] = parseFloat(next_token);
    (*material)->_Ks._v[1] = parseFloat(next_token);
    (*material)->_Ks._v[2] = parseFloat(next_token);
}

void OBJReader::parse_mtl_Ns(char **next_token, Mtl **material)
{
    (*material)->_Ns = parseFloat(next_token)/1000*128;
}

void OBJReader::parse_mtl_d(char **next_token, Mtl **material)
{
    (*material)->_d = parseFloat(next_token);
}

int OBJReader::parseInt(char **next_token)
{
	char *token = rwStrtok(NULL, " \t", next_token);
	if(token == NULL)
		RW_THROW("Parse error");
	return atoi(token);
}

float OBJReader::parseFloat(char **next_token)
{
	char* token = rwStrtok(NULL, " \t", next_token);
	if(token == NULL)
		RW_THROW("Parse error");
	return static_cast<float>(atof(token));
}

float OBJReader::parseOptionalFloat(char **next_token, float defaultVal)
{
	char* token = rwStrtok(NULL, " \t", next_token);
	if(token == NULL)
		return defaultVal;
	else
		return static_cast<float>(atof(token));
}

void OBJReader::load(const std::string& fileName)
{
	int lineNum = 1;
	char *buffer;
	char *line;
	char *next_line = 0;

	std::vector<char> vecBuf;
	IOUtil::readFile(fileName, vecBuf);
	vecBuf.push_back(0);
	buffer = &vecBuf[0];
	_dirName = rw::common::StringUtil::getDirectoryName(fileName);

	// Read first line
	line = rwStrtok(buffer, "\r\n", &next_line);

	while(line != NULL)
	{
		if(line[0] != '#')
		{
			char *token;
			char *next_token;
			token = rwStrtok(line, " \t", &next_token);

			std::map<std::string, FuncPtr>::iterator it = _objTypeMap.find(token);
			if(it == _objTypeMap.end())
				RW_WARN("Unknown type: '" << token << "' in line " << lineNum);
			(this->*it->second)(&next_token);
		}
		line = rwStrtok(NULL, "\r\n", &next_line);
		lineNum++;
	}
}

void OBJReader::parseMtlFile(const std::string& fileName)
{
	int lineNum = 1;
	char *buffer;
	char *line;
	char *next_line = 0;
	Mtl *material;

	std::vector<char> vecBuf;
	IOUtil::readFile(fileName, vecBuf);
	vecBuf.push_back(0);
	buffer = &vecBuf[0];

	// Read first line
	line = rwStrtok(buffer, "\r\n", &next_line);

	while(line != NULL)
	{
		if(line[0] != '#')
		{
			char *token;
			char *next_token;
			token = rwStrtok(line, " \t", &next_token);

			std::map<std::string, MtlFuncPtr>::iterator it = _mtlTypeMap.find(token);
			if(it == _mtlTypeMap.end())
				RW_THROW("Unknown mtl type: '" << token << "' in line " << lineNum);
			(this->*it->second)(&next_token, &material);
		}
		line = rwStrtok(NULL, "\r\n", &next_line);
		lineNum++;
	}
}


void OBJReader::scale(float factor)
{
	std::vector<Vec3>::iterator it;
	for(it=_geoVertices.begin(); it!=_geoVertices.end(); it++)
	{
		it->_v[0] *= factor;
		it->_v[1] *= factor;
		it->_v[2] *= factor;
	}
}

void OBJReader::writeFile(const std::string& filename) const
{
	std::string objFilename = filename + ".obj";
	std::string mtlFilename = filename + ".mtl";

	writeMtlFile(mtlFilename);

	std::ofstream out(objFilename.c_str());
	if(!out)
		RW_THROW("Unable to write '" << objFilename << "'");

	out << "mtllib " << mtlFilename.substr(mtlFilename.rfind("\\")+1) << std::endl;

	std::vector<Vec3>::const_iterator it;
    out.precision(6);
	out.setf(std::ios_base:: fixed);
	for(it=_geoVertices.begin(); it!=_geoVertices.end(); it++)
		out << "v " << it->_v[0] << " " << it->_v[1] << " " << it->_v[2] << std::endl;

	for(it=_textVertices.begin(); it!=_textVertices.end(); it++)
		out << "vt " << it->_v[0] << " " << it->_v[1] << " " << it->_v[2] << std::endl;

	for(it=_vertexNormals.begin(); it!=_vertexNormals.end(); it++)
		out << "vn " << it->_v[0] << " " << it->_v[1] << " " << it->_v[2] << std::endl;

	std::vector<RenderItem*>::const_iterator it2;
	for(it2=_renderItems.begin(); it2!=_renderItems.end(); it2++)
		(*it2)->write(out);

	out.close();
}

void OBJReader::writeMtlFile(const std::string& filename) const
{
	std::ofstream out(filename.c_str());
	if(!out)
		RW_THROW("Unable to write '" << filename << "'");
    out.precision(6);
	out.setf(std::ios_base:: fixed);

	std::map<std::string, Mtl*>::const_iterator it;
	for(it=_materials.begin(); it!=_materials.end(); it++)
		it->second->Write(out);

	out.close();
}

void OBJReader::calcVertexNormals()
{
	std::map<int, std::vector<Face*> > geoVertexFaceMap;
	std::vector<RenderItem*>::iterator rit;
	std::vector<IVec3>::iterator eit;
	for(rit=_renderItems.begin(); rit!=_renderItems.end(); rit++)
	{
		if(typeid(**rit) == typeid(class OBJReader::Face))
		{
			Face *face = static_cast<Face*>(*rit);
			for(eit=face->_element.begin(); eit!=face->_element.end(); eit++)
			{
				geoVertexFaceMap[eit->_v[0]].push_back(face);
				eit->_v[2] = eit->_v[0];
			}
			face->calcCommonNormal();
		}
	}

	std::map<int, std::vector<Face*> >::iterator git;
	for(git=geoVertexFaceMap.begin(); git!=geoVertexFaceMap.end(); git++)
	{
		Vector3D<float> tmp(0.0f, 0.0f, 0.0f);
		std::vector<Face*>::iterator vit;
		for(vit=git->second.begin(); vit!=git->second.end(); vit++)
			tmp += (*vit)->_vCommonNorm;
		tmp /= static_cast<float>(git->second.size());
		_vertexNormals.push_back(Vec3(tmp[0], tmp[1], tmp[2]));
	}
}




namespace {

    void triangulatePolygon(IndexedPolygonN<>& poly,
                            std::vector<rw::math::Vector3D<float> >& verts,
                            std::vector<IndexedTriangle<> >& res)
    {

        // calculate poly normal from first three vertices
        Vector3D<> v0 = cast<double>( verts[ poly[0] ] );
        Vector3D<> v1 = cast<double>( verts[ poly[1] ] );
        Vector3D<> v2 = cast<double>( verts[ poly[2] ] );
        Vector3D<> n = normalize( cross(v1-v0,v2-v0) );
        //std::cout << "-" << v0 << "\n-" << v1 << "\n-" << v2 << "\n-" << n << std::endl;

        EAA<> eaa(n,Vector3D<>(0,0,1));
        Rotation3D<> rotNtoZ = eaa.toRotation3D();
        // make vector of 2d points
        std::vector<Vector2D<> > points(poly.size());
        for(size_t j=0;j<poly.size();j++){
            // rotate each point such that the xy-plane is perpendicular to the normal
            Vector3D<> v = rotNtoZ * cast<double>( verts[ poly[j] ] );
            //std::cout << v << std::endl;
            points[j](0) = v(0);
            points[j](1) = v(1);
        }

        // now do the triangulation
        std::vector<int> indices;
        int iidx=0;
        if( Triangulate::processPoints(points, indices) ){
            while(iidx < (int)indices.size()){
                IndexedTriangle<> tri(poly[ indices[iidx  ] ],
                                      poly[ indices[iidx+1] ],
                                      poly[ indices[iidx+2] ]);
                res.push_back(tri);
                iidx += 3;
            }
        } else {
            RW_WARN("Could not triangulate polygon face. Check face for overlapping points!");
        }
    }

}



Model3D::Ptr LoaderOBJ::load(const std::string& name){
	//Start by storing the current locale. This is retrieved by passing NULL to setlocale	
	std::string locale = setlocale(LC_ALL, NULL); 
	//We set the locale to make sure things are parsed correctly in from file
	setlocale(LC_ALL, "C");
	OBJReader reader;
	reader.load(name);
	//Restore the old locale
	setlocale(LC_ALL, locale.c_str());

	Model3D::Ptr model( ownedPtr( new Model3D(name) ) );
    Model3D::Object3D *obj = new Model3D::Object3D("OBJModel");
	int currentMatIdx = model->addMaterial(Model3D::Material("defcol",0.5,0.5,0.5));
    Model3D::MaterialFaces *mface = new Model3D::MaterialFaces();
    mface->_matIndex = currentMatIdx;
    size_t nb_points=0;
	BOOST_FOREACH(OBJReader::RenderItem* item, reader._renderItems){
		if(OBJReader::UseMaterial* matobj = dynamic_cast<OBJReader::UseMaterial*>(item)){
        	//if(mface->_subFaces.size()>0){
        	//    obj->_matFaces.push_back(mface);
        	//	mface = new Model3D::MaterialFaces();
        	//}

        	float r = matobj->_material->_Kd._v[0];
        	float g = matobj->_material->_Kd._v[1];
        	float b = matobj->_material->_Kd._v[2];
        	Model3D::Material mat(matobj->_material->_name, r, g, b);
        	for (unsigned int i = 0; i < 3; i++) {
        		mat.ambient[i] = matobj->_material->_Ka._v[i];
        		mat.specular[i] = matobj->_material->_Ks._v[i];
        	}
    		mat.shininess = matobj->_material->_Ns;
    		mat.transparency = matobj->_material->_d;
    		mat.simplergb = false;
            currentMatIdx = model->addMaterial(mat);
            obj->setMaterial( currentMatIdx );
            //mface->_matIndex = currentMatIdx;
		} else if( OBJReader::Face* face = dynamic_cast<OBJReader::Face*>(item) ){

            RW_ASSERT(face->_element.size()>=2);
            for(unsigned int i=0;i<face->_element.size();i++){
                Vector3D<float> n;
                if(face->_element[i]._v[2] != -1){
                    OBJReader::Vec3 nobj = reader._vertexNormals[face->_element[i]._v[2]-1];
                    n = rw::math::Vector3D<float>(nobj._v[0],nobj._v[1],nobj._v[2]);
                } else {
                    n = face->_vCommonNorm;
                }

                OBJReader::Vec3 vobj = reader._geoVertices[face->_element[i]._v[0]-1];

                Vector3D<float> v(vobj._v[0],vobj._v[1],vobj._v[2]);

                obj->_vertices.push_back(v);
                obj->_normals.push_back(n);
                nb_points++;
            }

            if(face->_element.size()<3){
                RW_WARN("An OBJ surface with only 2 vertices detected! It will be ignored!");
            } else if(face->_element.size()==3){
                // use TriangleUtil toIndexedTriMesh, though remember the normals
                //obj->_faces.push_back( rw::geometry::IndexedTriangle<>(nb_points-3,nb_points-2,nb_points-1) );
                obj->addTriangle(rw::geometry::IndexedTriangle<>(
					(uint16_t)(nb_points-3),
					(uint16_t)(nb_points-2),
					(uint16_t)(nb_points-1)));
                //mface->_subFaces.push_back(obj->_faces.back());
            } else {
                // its a polygon, since we don't support that in Model3D, we make triangles of it
                IndexedPolygonN<> poly(face->_element.size());
                for(size_t j=0; j<face->_element.size();j++)
                    poly[j] = (uint16_t)(nb_points-face->_element.size()+j);
                std::vector<IndexedTriangle<> > tris;
                triangulatePolygon(poly, obj->_vertices, tris);
                obj->addTriangles(tris);
                //BOOST_FOREACH(IndexedTriangle<> &tri, tris){
                    //obj->_faces.push_back( tri );
                    //mface->_subFaces.push_back( tri );
                //}
            }
		}
	}
	//std::cout << "nr faces: " << obj->_faces.size() << std::endl;
    //obj->_matFaces.push_back(mface);

    // order stuff in matrial faces

    model->addObject(obj);
    model->optimize(35*Deg2Rad);
	return model;
}

