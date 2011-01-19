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

#ifndef RWLIBS_DRAWABLE_MODEL3D_HPP_
#define RWLIBS_DRAWABLE_MODEL3D_HPP_

//! @file Model3D.hpp

#include "RWGLTexture.hpp"

#include <vector>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/IndexedTriangle.hpp>
#include <rw/geometry/IndexedPolygon.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector2D.hpp>

namespace rwlibs {
namespace opengl {

	//! @addtogroup drawable
	// @{


    /**
     * @brief a 3d model that has geometry but also material, color and texture information.
     * the model can be composed of multiple objects that are connected in a hierarchical manner.
     * The model is designed for efficient drawing and as such special structures are used
     * to order the indexes such that efficient drawing is possible.
     */
    class Model3D {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<Model3D> Ptr;

        //! @brief constructor
        Model3D();

        //! @brief destructor
        virtual ~Model3D();

        /**
         * @brief describes material properties. A material can be either simple or "advanced"
         * and in both cases it can be textured.
         * A simple material is described by a 4-tuple of RGBA values. The advanced material
         * defines multiple properties: diffuse, ambient, emissive, specular, shininess and transparency
         */
        struct Material {
            //! @brief default constructor
            Material():name(""), textured(false), simplergb(true){};
            //! @brief constructor for simple material
            Material(const std::string& nam, float r, float g, float b, float a=1.0):
                name(nam), textured(false), simplergb(true)
            {
                rgb[0] = r;
                rgb[1] = g;
                rgb[2] = b;
                rgb[3] = a;
            }
            //! @brief material name, not necesarily unique
            std::string name;
            //! @brief index to a texture which is stored in Model3D, -1 if not used
            short int texId;
            //! @brief true is the material is textured
            bool textured;	// whether or not it is textured
            //! @brief true if this material is a simple material
            bool simplergb;
            //! @brief Red, Green, Blue and alpha color (simple) or diffues color(advanced)
            float rgb[4];
            //! @brief Ambient color as RGBA
            float ambient[4];
            //! @brief Emissive color as RGBA
            float emissive[4];
            //! @brief Specular color as RGB
            float specular[4];

            //! @brief The shininess \f$\in [0,128] \f$
            float shininess;
            //! @brief Transparency \f$ in [0, 1]\f$
            float transparency;
        };

        /**
         * @brief ordering triangles by material consumes more memmory but reduce switches between
         * textures. All indices \b _subFaces share material \b _matIndex.
         */
        struct MaterialFaces {
            /**
             *  @brief  Index into the vertice array of the Object3D.
             *  The _subFaces is a subset of _indices from Object3D
             */
            std::vector<rw::geometry::IndexedTriangle<> > _subFaces;
            //! @brief the material index shared by all triangles \b _subFaces
            int _matIndex;
        };

        /**
         * @brief ordering polygons by material consumes more memmory but reduce switches between
         * textures. All indices \b _subFaces share material \b _matIndex.
         */
        struct MaterialPolys {
            /**
             *  @brief  Index into the vertice array of the Object3D.
             *  The _subFaces is a subset of _indices from Object3D
             */
            std::vector<rw::geometry::IndexedPolygonN<> > _subPolys;
            //! @brief the material index shared by all polygons \b _subPolys
            int _matIndex;
        };

        /**
         * @brief
         */
        struct Object3D {
            /**
             * @brief constructor
             * @param name [in] name of object
             */
            Object3D(const std::string& name):
                _name(name),
                _parentObj(-1),
                _texture(-1),
                _texOffset(0,0),
                _texRepeat(0,0)
                {};

            //! @brief test if this object is textured
            bool hasTexture() const{ return _texture>=0;};

            //! @brief name/id of object
            std::string _name;
            //! @brief index of parent object
            int _parentObj;
            //! @brief texture id
            int _texture;

            std::vector<rw::math::Vector3D<float> > _vertices;
            std::vector<rw::math::Vector3D<float> > _normals;
            std::vector<rw::math::Vector2D<float> > _texCoords;

            /**
             * @brief list containing indexed polygons. The polygons index into the
             * \b _vertices array and the \b _normals array
             * The normal is implicitly indexed and defined as same index as the
             * vertex.
             */
            std::vector<rw::geometry::IndexedTriangle<> > _faces;

            /**
             * @brief list containing indexed polygons. The polygons index into the
             * \b _vertices array and the \b _normals array
             * The normal is implicitly indexed and defined as same index as the
             * vertex.
             */
            std::vector<rw::geometry::IndexedPolygonN<> > _polys;


            std::vector<MaterialFaces*> _matFaces;
            std::vector<MaterialPolys*> _matPolys;
            rw::math::Transform3D<float> _transform;
            std::vector<Object3D*> _kids;
            rw::math::Vector2D<float> _texOffset, _texRepeat;
        };

    public:

        int addObject(Object3D* obj);
        int addMaterial(const Material& mat);
        void removeObject(const std::string& name);

        std::vector<Material>& getMaterials(){ return _materials; };
        std::vector<Object3D*>& getObjects(){ return _objects; };

        const rw::math::Transform3D<>& getTransform(){ return _transform;};
        void setTransform(const rw::math::Transform3D<>& t3d){ _transform = t3d;};
    //private:
        rw::math::Transform3D<> _transform;
        std::vector<Material> _materials; // The array of materials
        std::vector<Object3D*> _objects; // The array of objects in the model
        std::vector<RWGLTexture*> _textures;

        int totalVerts;			// Total number of vertices in the model
        int totalFaces;			// Total number of faces in the model
    };

    typedef rw::common::Ptr<Model3D> Model3DPtr;
    //! @}
}
}

#endif /* MODEL3D_HPP_ */
