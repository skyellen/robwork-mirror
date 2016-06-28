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

#ifndef RW_GRAPHICS_LOADERAC3D_HPP
#define RW_GRAPHICS_LOADERAC3D_HPP

//! @file LoaderAC3D.hpp

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

#include <vector>
#include <map>

#include "../Model3DLoader.hpp"

namespace rw { namespace graphics { class TextureData; } }

namespace rw { namespace loaders {

    /** @addtogroup graphics */
    /*@{*/

    /**
     * @brief This class loads AC3D geometry into a Model3D object
     */
    class LoaderAC3D: public Model3DLoader {
    public:
        /**
         * Constructs a LoaderAC3D object
         */
        LoaderAC3D();

        /**
         * @brief Destroys RenderAC3D object
         */
        virtual ~LoaderAC3D();

        //! @copydoc Model3DLoader::load
        rw::graphics::Model3D::Ptr load(const std::string& filename);

        //void save(Model3DPtr model, const std::string& filename);

    private:
        void initialize(std::istream& in, float alpha);

        enum OBJECT_TYPE {
            OBJECT_WORLD = 0,
            OBJECT_GROUP,
            OBJECT_LIGHT,
            OBJECT_NORMAL
        };

        struct Vector3f {
        	Vector3f(){
        		val[0] = 0; val[1] = 0; val[2] = 0;
        	};

        	Vector3f(float a,float b,float c){
        		val[0] = a; val[1] = b; val[2] = c;
        	};

        	Vector3f(const rw::math::Vector3D<float>& v3d){
        		val[0] = v3d(0); val[1] = v3d(1); val[2] = v3d(2);
        	};

        	virtual ~Vector3f(){};

        	rw::math::Vector3D<float> toV3D(){
        		return rw::math::Vector3D<float>(val[0],val[1],val[2]);
        	}
        	float val[3];
        };

        /*struct AC3DVertex {
            AC3DVertex() : loc(0.0, 0.0, 0.0) //, normal(0.0, 0.0, 0.0)
            {}
            // Location
            Vector3f loc;
            // Normal
            //Vector3f normal; Removed since the individual surface
            //determine the vertex normal
        };*/

        struct AC3DSurface {

            /** Array with vertex id */
            std::vector<int> vertrefs;

            /** Array with vertex normals **/
            std::vector<Vector3f> normals;

            /** Length of vertrefs array */
            int vertref_cnt;

            /** Array with uvs */
            std::vector<rw::math::Vector2D<float> > uvs;

            /** Surface normal */
            Vector3f normal;
            /** Flags */
            int flags;
            /** Material id */
            int mat;

            /** Constructs AC3DSurface */
            AC3DSurface() :
                normal(0.0, 0.0, 0.0)
            {
                vertref_cnt = 0;
                flags = 0;
                mat = 0;
            }

            /** Destroys AD3DSurface */
            ~AC3DSurface()
            {}
        };

        struct AC3DObject
        {
            // All the data from the ACObject

            /** Location */
            rw::math::Vector3D<float> loc;

            /** Rotation */
            rw::math::Rotation3D<float> rot;

            /** Object name */
            std::string name;

            /** Data array */
            std::vector<char> data;

            /** url */
            std::string url;

            /** Array with vertices and normals, each vertex has a normal */
            std::vector<Vector3f> vertices;
            std::vector<Vector3f> normals;

            /** Length of vertices array */
            int vertex_cnt;

            /** Array with surfaces */
            std::vector<AC3DSurface> surfaces;

            /** Array mapping vertices to neighboring surfaces **/
            //std::vector<std::vector<AC3DSurface*> > _vertSurfMap;

            /** Array mapping mat to surface index */
            //typedef std::map<std::pair<int,int>,std::vector<int> > MatSurfMap;
            //MatSurfMap _matToSurfArray;

            /** Length of surfaces array */
            int surf_cnt;

            /** Texture Settings */
            float texture_repeat_x;
            /** Texture Settings */
            float texture_repeat_y;
            /** Texture Settings */
            float texture_offset_x;
            /** Texture Settings */
            float texture_offset_y;

            /** Array with pointer to AC3DObjects */
            std::vector<AC3DObject*> kids;

            /** Length of kids array */
            int num_kids;

            /** Type */
            int type;
            /** Texture Id */
            int texture;

            /** Construct AC3DObject */
            AC3DObject() :
                loc(0.0, 0.0, 0.0),
                rot(rw::math::Rotation3D<float>::identity()),
                texture_repeat_x(1.0),
                texture_repeat_y(1.0),
                texture_offset_x(0.0),
                texture_offset_y(0.0)

            {
                vertex_cnt = 0;
                surf_cnt = 0;
                num_kids = 0;
                texture = -1;
            }

            /** Destroys AC3DObject */
            ~AC3DObject()
            {
                for (int i = 0;  i < num_kids; i++)
                    delete kids[i];
            }

        private:
            AC3DObject(const AC3DObject&);
            AC3DObject& operator=(const AC3DObject&);
        };

        struct AC3DMaterial {
            AC3DMaterial()
            {
                rgb[0] = 0.0; rgb[1]=0.0; rgb[2]=0.0; rgb[3]=1.0;
                ambient[0]=0.0; ambient[1]=0.0; ambient[2]=0.0; ambient[3]=1.0;
                emissive[0]=0.0;emissive[1]=0.0; emissive[2]=0.0; emissive[3]=1.0;
                specular[0]=0.0;specular[1]=0.0; specular[2]=0.0; specular[3]=1.0;
            }

            /** Red, Green, Blue color components */
            float rgb[4];
            /** Ambient color as RGB */
            float ambient[4];
            /** Emissive color as RGB */
            float emissive[4];
            /** Specular color as RGB */
            float specular[4];

            /** The shininess \f$\in [0,128] \f$ */
            float shininess;
            /** Transparency \f$ in [0, 1]\f$ */
            float transparency;
            /** Name of material */
            std::string name;
        };

        struct ModelAC3D {
        	virtual ~ModelAC3D(){
        	//	delete _object;
        	}
            std::vector<rw::graphics::TextureData> _textures;
            std::map<int, rw::graphics::TextureData*> _textureMap;
            //AC3DObject* _object;
            std::vector<AC3DMaterial> _materials;
            std::string _currentDir;
            int nrOfObjects;
        };

        int loadTexture(const std::string& filename, ModelAC3D* model);

        AC3DMaterial read_material(std::istream& in);
        OBJECT_TYPE string_to_objecttype(const std::string& s);

        void read_data(std::istream& in, std::vector<char>& out);

        void read_surface(std::istream& in, AC3DSurface& surf, AC3DObject* ob);

        AC3DObject* load_object(std::istream& in, AC3DObject* parent, ModelAC3D* model);

        void calc_vertex_normals(AC3DObject *ob);


        double _maxAngle;//in rad
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
