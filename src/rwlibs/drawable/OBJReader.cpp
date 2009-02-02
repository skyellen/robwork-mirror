#include "OBJReader.hpp"

#include <fstream>
#include <sys/types.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <rwlibs/os/rwgl.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>

using namespace rwlibs::drawable;
using namespace rw::math;
using namespace rw::common;

void OBJReader::Face::render(float alpha)
{
	std::vector<IVec3>::iterator it;

	glBegin(GL_POLYGON);
	for(it=_element.begin(); it!=_element.end(); it++)
	{
		if(it->_v[2] != -1)
			glNormal3fv(_objReader->_vertexNormals[it->_v[2]-1]._v);
		glVertex3fv(_objReader->_geoVertices[it->_v[0]-1]._v);
	}
	glEnd();
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

void OBJReader::UseMaterial::render(float alpha)
{
	//glColor4f(_material->_Kd._v[0], _material->_Kd._v[1], _material->_Kd._v[2], 1.0f);
	glColor4f(_material->_Kd._v[0], _material->_Kd._v[1], _material->_Kd._v[2], alpha);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, _material->_Ka._v);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, _material->_Kd._v);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, _material->_Ks._v);
	//glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, _material->_Ns._v);
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

	_mtlTypeMap["newmtl"]		= &OBJReader::parse_mtl_newmtl;
	_mtlTypeMap["illum"]		= &OBJReader::parse_mtl_illum;
	_mtlTypeMap["Kd"]			= &OBJReader::parse_mtl_Kd;
	_mtlTypeMap["Ka"]			= &OBJReader::parse_mtl_Ka;
	_mtlTypeMap["Tf"]			= &OBJReader::parse_mtl_Tf;
	_mtlTypeMap["Ni"]			= &OBJReader::parse_mtl_Ni;
	_mtlTypeMap["Ks"]			= &OBJReader::parse_mtl_Ks;
	_mtlTypeMap["Ns"]			= &OBJReader::parse_mtl_Ns;
	_mtlTypeMap["d"]			= &OBJReader::parse_mtl_d;
}

OBJReader::~OBJReader()
{
	std::map<std::string, Mtl*>::iterator it;
	for(it=_materials.begin(); it!=_materials.end(); it++)
		delete it->second;
}


void OBJReader::parse_v(tokenizer::iterator *token, tokenizer::iterator end)
{
    float x = parseFloat(token, end);
    float y = parseFloat(token, end);
    float z = parseFloat(token, end);
    _geoVertices.push_back(Vec3(x, y, z));
}

void OBJReader::parse_vt(tokenizer::iterator *token, tokenizer::iterator end)
{
    float u = parseFloat(token, end);
    float v = parseOptionalFloat(token, end, 0.0);
    float w = parseOptionalFloat(token, end, 0.0);
    _textVertices.push_back(Vec3(u, v, w));
}

void OBJReader::parse_vn(tokenizer::iterator *token, tokenizer::iterator end)
{
    float i = parseFloat(token, end);
    float j = parseFloat(token, end);
    float k = parseFloat(token, end);
    _vertexNormals.push_back(Vec3(i, j, k));
}

void OBJReader::parse_face(tokenizer::iterator *token, tokenizer::iterator end)
{
    Face *face = new Face(this);
    _renderItems.push_back(face);

    (*token)++;
    for(; *token!=end; (*token)++)
    {
        tokenizer tuples(**token, boost::char_separator<char>(" \t"));

        tokenizer::iterator tuple;
        for(tuple=tuples.begin(); tuple!=tuples.end(); tuple++)
        {
            face->_element.push_back(IVec3(-1, -1, -1));
            tokenizer nums(*tuple, boost::char_separator<char>("", "/"));
            tokenizer::iterator num;
            int i=0;
            for(num=nums.begin(); num!=nums.end(); num++)
            {
                if(*num == "/")
                    i++;
                else
                    face->_element.back()._v[i] = atoi(num->c_str());
            }
        }
    }
}

void OBJReader::parse_g(tokenizer::iterator *token, tokenizer::iterator end)
{
    RW_THROW("ObjReader::parse_g --- METHOD NOT IMPLEMENTED");
    //char *token = strtok_s(NULL, "\r\n", next_token);
    //std::cout << "Group: " << token << std::endl;
}

void OBJReader::parse_usemtl(tokenizer::iterator *token, tokenizer::iterator end)
{
    (*token)++;
    std::map<std::string, Mtl*>::iterator it = _materials.find(**token);
    if(it == _materials.end())
        RW_THROW("Material '" << **token << "' not defined");
    _renderItems.push_back(new UseMaterial(it->second));
}

void OBJReader::parse_mtllib(tokenizer::iterator *token, tokenizer::iterator end)
{
    while(++(*token) != end)
        parseMtlFile(_dirName + **token);
}

void OBJReader::parse_mtl_newmtl(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
{
    (*token)++;
    *material = new Mtl();
    _materials[**token] = *material;
    (*material)->_name = **token;
}

void OBJReader::parse_mtl_illum(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
{
}

void OBJReader::parse_mtl_Kd(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
{
    (*material)->_Kd._v[0] = parseFloat(token, end);
    (*material)->_Kd._v[1] = parseFloat(token, end);
    (*material)->_Kd._v[2] = parseFloat(token, end);
}

void OBJReader::parse_mtl_Ka(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
{
    (*material)->_Ka._v[0] = parseFloat(token, end);
    (*material)->_Ka._v[1] = parseFloat(token, end);
    (*material)->_Ka._v[2] = parseFloat(token, end);
}

void OBJReader::parse_mtl_Tf(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
{
    (*material)->_Tf._v[0] = parseFloat(token, end);
    (*material)->_Tf._v[1] = parseFloat(token, end);
    (*material)->_Tf._v[2] = parseFloat(token, end);
}

void OBJReader::parse_mtl_Ni(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
{
}

void OBJReader::parse_mtl_Ks(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
{
    (*material)->_Ks._v[0] = parseFloat(token, end);
    (*material)->_Ks._v[1] = parseFloat(token, end);
    (*material)->_Ks._v[2] = parseFloat(token, end);
}

void OBJReader::parse_mtl_Ns(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
{
}

void OBJReader::parse_mtl_d(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
{
    (*material)->_d = parseFloat(token, end);
}

int OBJReader::parseInt(tokenizer::iterator *token, tokenizer::iterator end)
{
    if(++(*token) == end)
        RW_THROW("Parse error");
    return atoi((**token).c_str());
}

float OBJReader::parseFloat(tokenizer::iterator *token, tokenizer::iterator end)
{
    if(++(*token) == end)
        RW_THROW("Parse error");
    return static_cast<float>(atof((**token).c_str()));
}

float OBJReader::parseOptionalFloat(tokenizer::iterator *token, tokenizer::iterator end, float defaultVal)
{
    if(++(*token) == end)
        return defaultVal;
    else
        return static_cast<float>(atof((**token).c_str()));
}

void OBJReader::load(const std::string& fileName)
{
	int lineNum = 1;
	//char *buffer;
	std::vector<char> buffer;
	IOUtil::readFile(fileName, buffer);
	_dirName = rw::common::StringUtil::getDirectoryName(fileName);

	std::string str(buffer.begin(), buffer.end());
	tokenizer lines(str, boost::char_separator<char>("\r\n"));

	tokenizer::iterator line;
	for(line=lines.begin(); line!=lines.end(); ++line)
	{
		if((*line)[0] != '#')
		{
			tokenizer tokens(*line, boost::char_separator<char>(" \t"));
			tokenizer::iterator token = tokens.begin();

			std::map<std::string, FuncPtr>::iterator it = _objTypeMap.find(*token);
			if(it == _objTypeMap.end())
				RW_THROW("Unknown type: '" << *token << "' in line " << lineNum);
			(this->*it->second)(&token, tokens.end());
		}
		lineNum++;
	}


}

void OBJReader::parseMtlFile(const std::string& fileName)
{
	int lineNum = 1;
    std::vector<char> buffer;
	Mtl *material;

	IOUtil::readFile(fileName, buffer);

	std::string str(buffer.begin(), buffer.end());
	tokenizer lines(str, boost::char_separator<char>("\r\n"));

	tokenizer::iterator line;
	for(line=lines.begin(); line!=lines.end(); ++line)
	{
		if((*line)[0] != '#')
		{
			tokenizer tokens(*line, boost::char_separator<char>(" \t"));
			tokenizer::iterator token = tokens.begin();

			std::map<std::string, MtlFuncPtr>::iterator it = _mtlTypeMap.find(*token);
			if(it == _mtlTypeMap.end())
				RW_THROW("Unknown mtl type: '" << *token << "' in line " << lineNum);
			(this->*it->second)(&token, tokens.end(), &material);
		}
		lineNum++;
	}
}


/*void OBJReader::readFile (const std::string& fileName, char **buffer)
{
	//int fh;
	unsigned long bufSize;

	FILE* fh = fopen(fileName.c_str(), "w");

	if (fh == NULL)
	{
		RW_THROW("Can't open file '" << fileName << "'");
	}


	 fseek( fh, 0L, SEEK_END );
     unsigned long fileSize = ftell( fh );
     rewind(fh);
	// Get file size
	//unsigned long fileSize = _filelength(fh);
	bufSize = fileSize + 1;	// Allocate space for a ending NULL

	// Create buffer
	*buffer = new char[bufSize];

	// Read file content into buffer
	size_t r = fread(buffer, 1, fileSize, fh);
	if (r != fileSize)
	    RW_THROW("Reading error of file "<<fileName);
	(*buffer)[bufSize-1] = NULL;

	fclose(fh);
}*/

void OBJReader::render(float alpha) const
{
	std::vector<RenderItem*>::const_iterator it;
	for(it=_renderItems.begin(); it!=_renderItems.end(); it++)
		(*it)->render(alpha);
}

/*
void ObjReader::Test1()
{
	std::vector<Vec3>::iterator it;
	for(it=_geoVertices.begin(); it!=_geoVertices.end(); it++)
	{
		_yMap[it->_v[1]].push_back(&(*it));
	}

	std::map<double, std::vector<Vec3*> >::iterator it2;
	for(it2=_yMap.begin(); it2!=_yMap.end(); it2++)
		std::cout << ">> " << it2->first << std::endl;
}
*/

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
