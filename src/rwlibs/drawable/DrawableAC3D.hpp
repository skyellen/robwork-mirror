/*********************************************************************
 * RobWork Version 0.2
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

#ifndef rwlibs_drawable_DrawableAC3D_HPP
#define rwlibs_drawable_DrawableAC3D_HPP

/**
 * @file DrawableAC3D.hpp
 */

#include <rwlibs/os/rwgl.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include "Drawable.hpp"

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief This class loads in and displays AC3D geometry
     */
    class DrawableAC3D: public Drawable {
    public:
        /**
         * Constructs a DrawableAC3D object from file
         * @param filename [in] name of file to be loaded
         */
        DrawableAC3D(const std::string& filename);

        /**
         * Constructs a DrawableAC3D loaded from a stream
         * @param in [in] in-stream to load data from
         */
        DrawableAC3D(std::istream& in);

        /**
         * Destroys DrawableAC3D object
         */
        ~DrawableAC3D();

    protected:
        /**
         * @copydoc Drawable::update
         */
        void update(UpdateType type);

    private:
        void initialize(std::istream& in);

        enum OBJECT_TYPE {
            OBJECT_WORLD = 0,
            OBJECT_GROUP,
            OBJECT_LIGHT,
            OBJECT_NORMAL
        };

        struct AC3DVertex {
            AC3DVertex() : loc(0.0, 0.0, 0.0), normal(0.0, 0.0, 0.0){}
            /** Location */
            rw::math::Vector3D<float> loc;
            /** Normal */
            rw::math::Vector3D<float> normal;
        };

        struct AC3DSurface {
            /** Array with vertex id */
            std::vector<int> vertrefs;

            /** Length of vertrefs array */
            int vertref_cnt;

            /** Array with uvs */
            std::vector<rw::math::Vector3D<float> > uvs;

            /** Surface normal */
            rw::math::Vector3D<float> normal;
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
            std::vector<AC3DVertex> vertices;

            /** Length of vertices array */
            int vertex_cnt;

            /** Array with surfaces */
            std::vector<AC3DSurface> surfaces;

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
                rot(rw::math::Rotation3D<float>::Identity())
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
            AC3DMaterial() :
                rgb(0.0, 0.0, 0.0),
                ambient(0.0, 0.0, 0.0),
                emissive(0.0, 0.0, 0.0),
                specular(0.0, 0.0, 0.0){}

            /** Red, Green, Blue color components */
            rw::math::Vector3D<float> rgb;
            /** Ambient color as RGB */
            rw::math::Vector3D<float> ambient;
            /** Emissive color as RGB */
            rw::math::Vector3D<float> emissive;
            /** Specular color as RGB */
            rw::math::Vector3D<float> specular;

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
        void calc_object_vertex_normals(AC3DObject* ob);

        void render(AC3DObject* ob) const;

        void col_set(long matno) const;
        void col_set_simple(long matno) const;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
