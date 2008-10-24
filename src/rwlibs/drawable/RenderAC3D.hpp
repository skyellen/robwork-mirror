/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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

#ifndef RWLIBS_DRAWABLE_RENDERAC3D_HPP
#define RWLIBS_DRAWABLE_RENDERAC3D_HPP

/**
 * @file RenderAC3D.hpp
 */

#include <rwlibs/os/rwgl.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

#include <list>
#include <vector>
#include <map>

#include "Render.hpp"

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief This class loads in and displays AC3D geometry
     */
    class RenderAC3D: public Render {
    public:
        /**
         * Constructs a RenderAC3D object from file
         * @param filename [in] name of file to be loaded
         */
        RenderAC3D(const std::string& filename);

        /**
         * Constructs a RenderAC3D loaded from a stream
         * @param in [in] in-stream to load data from
         */
        RenderAC3D(std::istream& in);

        /**
         * Destroys RenderAC3D object
         */
        virtual ~RenderAC3D();

        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;


    private:

    	GLuint _displayListId;

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
            std::vector<rw::math::Vector3D<float> > uvs;

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

            /** Array with vertices */
            std::vector<Vector3f> vertices;

            /** Length of vertices array */
            int vertex_cnt;

            /** Array with surfaces */
            std::vector<AC3DSurface> surfaces;

            /** Array mapping vertices to neighboring surfaces **/
            std::vector<std::vector<AC3DSurface*> > _vertSurfMap;

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
                rot(rw::math::Rotation3D<float>::identity())
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

        AC3DObject* _object;

        std::vector<AC3DMaterial> _materials;

        AC3DMaterial read_material(std::istream& in);

        OBJECT_TYPE string_to_objecttype(const std::string& s);

        void read_data(std::istream& in, std::vector<char>& out);

        void read_surface(std::istream& in, AC3DSurface& surf, AC3DObject* ob);

        AC3DObject* load_object(std::istream& in, AC3DObject* parent);

        void calc_vertex_normals(AC3DObject *ob);
        //void calc_object_vertex_normals(AC3DObject* ob);

        void render(AC3DObject* ob, float alpha) const;

        void col_set(long matno, float alpha) const;
        void col_set_simple(long matno, float alpha) const;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
