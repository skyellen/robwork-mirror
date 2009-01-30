#pragma once

#include <string>
#include <vector>

#include <boost/tokenizer.hpp>

#include <rw/common/macros.hpp>
#include <rwlibs/os/rwgl.hpp>
#include <rw/math/Vector3D.hpp>

namespace rwlibs { namespace drawable {

    /**
     * @brief Class for loading in OBJ files
     */
	class OBJReader
	{
	private:
		typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

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
			GLfloat _v[4];
			Vec4(GLfloat v1, GLfloat v2, GLfloat v3, GLfloat v4)
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
			Vec4 _Tf;
			float _d;

			Mtl()
			: _Ka(0.2f, 0.2f, 0.2f, 1.0f),
			  _Kd(0.8f, 0.8f, 0.8f, 1.0f),
			  _Ks(1.0f, 1.0f, 1.0f, 1.0f),
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
				//out << "Ns " << _Ns << std::endl;
			}
		};

		class RenderItem
		{
		public:
			virtual void render(float alpha)=0;
			virtual void write(std::ostream &out)=0;
		};

		class Face : public RenderItem
		{
		private:
			OBJReader *_objReader;

		public:
			rw::math::Vector3D<float> _vCommonNorm;
			std::vector<IVec3> _element;

			Face(OBJReader *objReader) : _objReader(objReader) {}
			virtual void render(float alpha);
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
		private:
			Mtl *_material;

		public:
			UseMaterial(Mtl *material) : _material(material) {}
			virtual void render(float alpha);
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
		//void Test1();
		void scale(float factor);
		void writeFile(const std::string& filename) const;
		void calcVertexNormals();

	private:
		std::string _dirName;
		std::vector<Vec3> _geoVertices;
		std::vector<Vec3> _textVertices;
		std::vector<Vec3> _vertexNormals;
		std::vector<RenderItem*> _renderItems;
		std::map<std::string, Mtl*> _materials;

		std::map<double, std::vector<Vec3*> > _yMap;

		void parseMtlFile(const std::string& fileName);
		void readFile(const std::string& fileName, char **buffer);
		void writeMtlFile(const std::string& filename) const;

		typedef void (OBJReader::*FuncPtr)(tokenizer::iterator *token, tokenizer::iterator end);
		std::map<std::string, FuncPtr> _objTypeMap;

		void parse_v(tokenizer::iterator *token, tokenizer::iterator end);
		void parse_vt(tokenizer::iterator *token, tokenizer::iterator end);
		void parse_vn(tokenizer::iterator *token, tokenizer::iterator end);
		void parse_face(tokenizer::iterator *token, tokenizer::iterator end);
		void parse_g(tokenizer::iterator *token, tokenizer::iterator end);
		void parse_usemtl(tokenizer::iterator *token, tokenizer::iterator end);
		void parse_mtllib(tokenizer::iterator *token, tokenizer::iterator end);
		void parse_vp(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'vp' not implemented"); }
		void parse_cstype(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'cstype' not implemented"); }
		void parse_deg(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'deg' not implemented"); }
		void parse_bmat(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'bmat' not implemented"); }
		void parse_step(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'step' not implemented"); }
		void parse_p(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'p' not implemented"); }
		void parse_l(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'l' not implemented"); }
		void parse_curv(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'curv' not implemented"); }
		void parse_curv2(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'curv2' not implemented"); }
		void parse_surf(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'surf' not implemented"); }
		void parse_parm(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'parm' not implemented"); }
		void parse_hole(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'hole' not implemented"); }
		void parse_scrv(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'scrv' not implemented"); }
		void parse_sp(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'sp' not implemented"); }
		void parse_end(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'end' not implemented"); }
		void parse_con(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'con' not implemented"); }
		void parse_mg(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'mg' not implemented"); }
		void parse_o(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'o' not implemented"); }
		void parse_bevel(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'bevel' not implemented"); }
		void parse_c_interp(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'c_interp' not implemented"); }
		void parse_d_interp(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'd_interp' not implemented"); }
		void parse_lod(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'lod' not implemented"); }
		void parse_shadow_obj(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'shadow_obj' not implemented"); }
		void parse_trace_obj(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'trace_obj' not implemented"); }
		void parse_ctech(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'ctech' not implemented"); }
		void parse_stech(tokenizer::iterator *token, tokenizer::iterator end) { RW_THROW("obj type 'stech' not implemented"); }

		typedef void (OBJReader::*MtlFuncPtr)(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material);
		std::map<std::string, MtlFuncPtr> _mtlTypeMap;

		void parse_mtl_newmtl(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material);
		void parse_mtl_illum(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material);
		void parse_mtl_Kd(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material);
		void parse_mtl_Ka(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material);
		void parse_mtl_Tf(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material);
		void parse_mtl_Ni(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material);
		void parse_mtl_Ks(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material);
		void parse_mtl_Ns(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material);
		void parse_mtl_d(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material);

		int parseInt(tokenizer::iterator *token, tokenizer::iterator end);
		float parseFloat(tokenizer::iterator *token, tokenizer::iterator end);
		float parseOptionalFloat(tokenizer::iterator *token, tokenizer::iterator end, float defaultVal);
	};


	inline void OBJReader::parse_v(tokenizer::iterator *token, tokenizer::iterator end)
	{
		float x = parseFloat(token, end);
		float y = parseFloat(token, end);
		float z = parseFloat(token, end);
		_geoVertices.push_back(Vec3(x, y, z));
	}

	inline void OBJReader::parse_vt(tokenizer::iterator *token, tokenizer::iterator end)
	{
		float u = parseFloat(token, end);
		float v = parseOptionalFloat(token, end, 0.0);
		float w = parseOptionalFloat(token, end, 0.0);
		_textVertices.push_back(Vec3(u, v, w));
	}

	inline void OBJReader::parse_vn(tokenizer::iterator *token, tokenizer::iterator end)
	{
		float i = parseFloat(token, end);
		float j = parseFloat(token, end);
		float k = parseFloat(token, end);
		_vertexNormals.push_back(Vec3(i, j, k));
	}

	inline void OBJReader::parse_face(tokenizer::iterator *token, tokenizer::iterator end)
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

	inline void OBJReader::parse_g(tokenizer::iterator *token, tokenizer::iterator end)
	{
		RW_THROW("ObjReader::parse_g --- METHOD NOT IMPLEMENTED");
		//char *token = strtok_s(NULL, "\r\n", next_token);
		//std::cout << "Group: " << token << std::endl;
	}

	inline void OBJReader::parse_usemtl(tokenizer::iterator *token, tokenizer::iterator end)
	{
		(*token)++;
		std::map<std::string, Mtl*>::iterator it = _materials.find(**token);
		if(it == _materials.end())
			RW_THROW("Material '" << **token << "' not defined");
		_renderItems.push_back(new UseMaterial(it->second));
	}

	inline void OBJReader::parse_mtllib(tokenizer::iterator *token, tokenizer::iterator end)
	{
		while(++(*token) != end)
			parseMtlFile(_dirName + **token);
	}

	inline void OBJReader::parse_mtl_newmtl(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
	{
		(*token)++;
		*material = new Mtl();
		_materials[**token] = *material;
		(*material)->_name = **token;
	}

	inline void OBJReader::parse_mtl_illum(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
	{
	}

	inline void OBJReader::parse_mtl_Kd(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
	{
		(*material)->_Kd._v[0] = parseFloat(token, end);
		(*material)->_Kd._v[1] = parseFloat(token, end);
		(*material)->_Kd._v[2] = parseFloat(token, end);
	}

	inline void OBJReader::parse_mtl_Ka(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
	{
		(*material)->_Ka._v[0] = parseFloat(token, end);
		(*material)->_Ka._v[1] = parseFloat(token, end);
		(*material)->_Ka._v[2] = parseFloat(token, end);
	}

	inline void OBJReader::parse_mtl_Tf(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
	{
		(*material)->_Tf._v[0] = parseFloat(token, end);
		(*material)->_Tf._v[1] = parseFloat(token, end);
		(*material)->_Tf._v[2] = parseFloat(token, end);
	}

	inline void OBJReader::parse_mtl_Ni(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
	{
	}

	inline void OBJReader::parse_mtl_Ks(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
	{
		(*material)->_Ks._v[0] = parseFloat(token, end);
		(*material)->_Ks._v[1] = parseFloat(token, end);
		(*material)->_Ks._v[2] = parseFloat(token, end);
	}

	inline void OBJReader::parse_mtl_Ns(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
	{
	}

	inline void OBJReader::parse_mtl_d(tokenizer::iterator *token, tokenizer::iterator end, Mtl **material)
	{
		(*material)->_d = parseFloat(token, end);
	}

	inline int OBJReader::parseInt(tokenizer::iterator *token, tokenizer::iterator end)
	{
		if(++(*token) == end)
			RW_THROW("Parse error");
		return atoi((**token).c_str());
	}

	inline float OBJReader::parseFloat(tokenizer::iterator *token, tokenizer::iterator end)
	{
		if(++(*token) == end)
			RW_THROW("Parse error");
		return static_cast<float>(atof((**token).c_str()));
	}

	inline float OBJReader::parseOptionalFloat(tokenizer::iterator *token, tokenizer::iterator end, float defaultVal)
	{
		if(++(*token) == end)
			return defaultVal;
		else
			return static_cast<float>(atof((**token).c_str()));
	}

}}
