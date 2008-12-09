#include "ObjReader.hpp"

#include <fstream>
#include <io.h>
#include <sys/types.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <rwlibs/os/rwgl.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rwlibs::drawable;
using namespace rw::math;

void ObjReader::Face::Render(float alpha)
{
	std::vector<IVec3>::iterator it;

	glBegin(GL_POLYGON);
	for(it=_element.begin(); it!=_element.end(); it++)
	{
		if(it->_i3 != -1)
			glNormal3fv(_objReader->_vertexNormals[it->_i3-1]._v);
		glVertex3fv(_objReader->_geoVertices[it->_i1-1]._v);
	}
	glEnd();
}

void ObjReader::Face::CalcCommonNormal()
{
	if(_element.size() >= 3)
	{
		Vector3D<float> p0(
			_objReader->_geoVertices[_element[0]._i1-1]._v[0],
			_objReader->_geoVertices[_element[0]._i1-1]._v[1],
			_objReader->_geoVertices[_element[0]._i1-1]._v[2]);
		Vector3D<float> p1(
			_objReader->_geoVertices[_element[1]._i1-1]._v[0],
			_objReader->_geoVertices[_element[1]._i1-1]._v[1],
			_objReader->_geoVertices[_element[1]._i1-1]._v[2]);
		Vector3D<float> p2(
			_objReader->_geoVertices[_element[2]._i1-1]._v[0],
			_objReader->_geoVertices[_element[2]._i1-1]._v[1],
			_objReader->_geoVertices[_element[2]._i1-1]._v[2]);
		Vector3D<float> p01 = p0 - p1;
		Vector3D<float> p21 = p2 - p1;
		_vCommonNorm = cross(p21, p01);
		_vCommonNorm = normalize(_vCommonNorm);
	}
}

void ObjReader::UseMaterial::Render(float alpha)
{
	//glColor4f(_material->_Kd._v[0], _material->_Kd._v[1], _material->_Kd._v[2], 1.0f);
	glColor4f(_material->_Kd._v[0], _material->_Kd._v[1], _material->_Kd._v[2], alpha);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, _material->_Ka._v);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, _material->_Kd._v);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, _material->_Ks._v);
	//glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, _material->_Ns._v);
}

ObjReader::ObjReader()
{
	_objTypeMap["v"]			= &ObjReader::parse_v;
	_objTypeMap["vt"]			= &ObjReader::parse_vt;
	_objTypeMap["vn"]			= &ObjReader::parse_vn;
	_objTypeMap["f"]			= &ObjReader::parse_face;
	_objTypeMap["g"]			= &ObjReader::parse_g;
	_objTypeMap["usemtl"]		= &ObjReader::parse_usemtl;
	_objTypeMap["mtllib"]		= &ObjReader::parse_mtllib;
	_objTypeMap["vp"]			= &ObjReader::parse_vp;
	_objTypeMap["cstype"]		= &ObjReader::parse_cstype;
	_objTypeMap["deg"]			= &ObjReader::parse_deg;
	_objTypeMap["bmat"]			= &ObjReader::parse_bmat;
	_objTypeMap["step"]			= &ObjReader::parse_step;
	_objTypeMap["p"]			= &ObjReader::parse_p;
	_objTypeMap["l"]			= &ObjReader::parse_l;
	_objTypeMap["curv"]			= &ObjReader::parse_curv;
	_objTypeMap["curv2"]		= &ObjReader::parse_curv2;
	_objTypeMap["surf"]			= &ObjReader::parse_surf;
	_objTypeMap["parm"]			= &ObjReader::parse_parm;
	_objTypeMap["hole"]			= &ObjReader::parse_hole;
	_objTypeMap["scrv"]			= &ObjReader::parse_scrv;
	_objTypeMap["sp"]			= &ObjReader::parse_sp;
	_objTypeMap["end"]			= &ObjReader::parse_end;
	_objTypeMap["con"]			= &ObjReader::parse_con;
	_objTypeMap["mg"]			= &ObjReader::parse_mg;
	_objTypeMap["o"]			= &ObjReader::parse_o;
	_objTypeMap["bevel"]		= &ObjReader::parse_bevel;
	_objTypeMap["c_interp"]		= &ObjReader::parse_c_interp;
	_objTypeMap["d_interp"]		= &ObjReader::parse_d_interp;
	_objTypeMap["lod"]			= &ObjReader::parse_lod;
	_objTypeMap["shadow_obj"]	= &ObjReader::parse_shadow_obj;
	_objTypeMap["trace_obj"]	= &ObjReader::parse_trace_obj;
	_objTypeMap["ctech"]		= &ObjReader::parse_ctech;
	_objTypeMap["stech"]		= &ObjReader::parse_stech;
	_objTypeMap["s"]			= &ObjReader::parse_s;

	_mtlTypeMap["newmtl"]		= &ObjReader::parse_mtl_newmtl;
	_mtlTypeMap["illum"]		= &ObjReader::parse_mtl_illum;
	_mtlTypeMap["Kd"]			= &ObjReader::parse_mtl_Kd;
	_mtlTypeMap["Ka"]			= &ObjReader::parse_mtl_Ka;
	_mtlTypeMap["Tf"]			= &ObjReader::parse_mtl_Tf;
	_mtlTypeMap["Ni"]			= &ObjReader::parse_mtl_Ni;
	_mtlTypeMap["Ks"]			= &ObjReader::parse_mtl_Ks;
	_mtlTypeMap["Ns"]			= &ObjReader::parse_mtl_Ns;
	_mtlTypeMap["d"]			= &ObjReader::parse_mtl_d;
}

ObjReader::~ObjReader()
{
	std::map<std::string, Mtl*>::iterator it;
	for(it=_materials.begin(); it!=_materials.end(); it++)
		delete it->second;
}

void ObjReader::ParseFile(const std::string& fileName)
{
	int lineNum = 1;
	char *buffer;
	char *line;
	char *next_line;

	ReadFile(fileName, &buffer);
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
				RW_THROW("Unknown type: '" << token << "' in line " << lineNum);
			(this->*it->second)(&next_token);
		}
		line = rwStrtok(NULL, "\r\n", &next_line);
		lineNum++;
	}

	delete [] buffer;
}

void ObjReader::ParseMtlFile(const std::string& fileName)
{
	int lineNum = 1;
	char *buffer;
	char *line;
	char *next_line;
	Mtl *material;

	ReadFile(fileName, &buffer);

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

	delete [] buffer;
}

void ObjReader::ReadFile (const std::string& fileName, char **buffer)
{
	int fh;
	unsigned long bufSize;

	fh = _open(fileName.c_str(), _O_RDONLY | _O_BINARY);

	if (fh == -1)
	{
		RW_THROW("Can't open file '" << fileName << "'");
	}

	// Get file size
	unsigned long fileSize = _filelength(fh);
	bufSize = fileSize + 1;	// Allocate space for a ending NULL

	// Create buffer
	*buffer = new char[bufSize];

	// Read file content into buffer
	int r = _read(fh, *buffer, fileSize);
	(*buffer)[bufSize-1] = NULL;

	_close(fh);
}

void ObjReader::Render(float alpha) const
{
	std::vector<RenderItem*>::const_iterator it;
	for(it=_renderItems.begin(); it!=_renderItems.end(); it++)
		(*it)->Render(alpha);
}

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

void ObjReader::Scale(float factor)
{
	std::vector<Vec3>::iterator it;
	for(it=_geoVertices.begin(); it!=_geoVertices.end(); it++)
	{
		it->_v[0] *= factor;
		it->_v[1] *= factor;
		it->_v[2] *= factor;
	}
}

void ObjReader::WriteFile(const std::string& filename) const
{
	std::string objFilename = filename + ".obj";
	std::string mtlFilename = filename + ".mtl";

	WriteMtlFile(mtlFilename);

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
		(*it2)->Write(out);

	out.close();
}

void ObjReader::WriteMtlFile(const std::string& filename) const
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

void ObjReader::CalcVertexNormals()
{
	std::map<int, std::vector<Face*> > geoVertexFaceMap;
	std::vector<RenderItem*>::iterator rit;
	std::vector<IVec3>::iterator eit;
	for(rit=_renderItems.begin(); rit!=_renderItems.end(); rit++)
	{
		if(typeid(**rit) == typeid(class ObjReader::Face))
		{
			Face *face = static_cast<Face*>(*rit);
			for(eit=face->_element.begin(); eit!=face->_element.end(); eit++)
			{
				geoVertexFaceMap[eit->_i1].push_back(face);
				eit->_i3 = eit->_i1;
			}
			face->CalcCommonNormal();
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
