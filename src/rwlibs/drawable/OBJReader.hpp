#ifndef RWLIBS_DRAWABLE_OBJREADER
#define RWLIBS_DRAWABLE_OBJREADER

#include <string>
#include <vector>
#include <cstring>

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
						**_Context = NULL;
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

		int parseInt(char **next_token);
		float parseFloat(char **next_token);
		float parseOptionalFloat(char **next_token, float defaultVal);
	};



}}

#endif //end include guard
