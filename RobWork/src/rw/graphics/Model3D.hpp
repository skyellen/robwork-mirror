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

#ifndef RW_GRAPHICS_MODEL3D_HPP_
#define RW_GRAPHICS_MODEL3D_HPP_

//! @file Model3D.hpp

#include "TextureData.hpp"

#include <vector>
#include <rw/geometry/GeometryData.hpp>
#include <rw/geometry/IndexedTriangle.hpp>
#include <rw/geometry/IndexedPolygon.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector2D.hpp>

namespace rw { namespace geometry { class Geometry; } }
namespace rw { namespace geometry { class TriMesh; } }

namespace rw {
namespace graphics {

	//! @addtogroup graphics
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
        Model3D(const std::string& name);

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
            Material():name(""), simplergb(true), texId(-1) {
                for (unsigned int i = 0; i < 4; i++) {
                	rgb[i] = 0;
                	ambient[i] = 0;
                	emissive[i] = 0;
                	specular[i] = 0;
                }
                shininess = 0;
                transparency = 0;
            }

            //! @brief constructor for simple material
            Material(const std::string& nam, float r, float g, float b, float a=1.0):
                name(nam), simplergb(true), texId(-1)
            {
                rgb[0] = r;
                rgb[1] = g;
                rgb[2] = b;
                rgb[3] = a;
                for (unsigned int i = 0; i < 4; i++) {
                	ambient[i] = 0;
                	emissive[i] = 0;
                	specular[i] = 0;
                }
                shininess = 0;
                transparency = 0;
            }

            /**
             * @brief Check if material has texture.
             * @return true if material has texture.
             */
            bool hasTexture() const { return texId>=0; }

            /**
             * @brief Get id of the texture for this material.
             * @return the texture id.
             */
            int getTextureID() const { return texId; }

            //! @brief material name, not necesarily unique
            std::string name;
            //! @brief true if this material is a simple material
            bool simplergb;
            //! @brief index to a texture which is stored in Model3D, -1 if not used
            short int texId;
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
        	//! @brief Smart pointer type for MaterialFaces.
            typedef rw::common::Ptr<MaterialFaces> Ptr;
            /**
             *  @brief  Index into the vertice array of the Object3D.
             *  The _subFaces is a subset of _indices from Object3D
             */
            std::vector<rw::geometry::IndexedTriangle<uint16_t> > _subFaces;
            //! @brief the material index shared by all triangles \b _subFaces
            int _matIndex;
        };

        /**
         * @brief ordering polygons by material consumes more memmory but reduce switches between
         * textures. All indices \b _subFaces share material \b _matIndex.
         */
        struct MaterialPolys {
        	//! @brief Smart pointer type for MaterialPolys.
            typedef rw::common::Ptr<MaterialPolys> Ptr;

            /**
             *  @brief  Index into the vertice array of the Object3D.
             *  The _subFaces is a subset of _indices from Object3D
             */
            std::vector<rw::geometry::IndexedPolygonN<uint16_t> > _subPolys;
            //! @brief the material index shared by all polygons \b _subPolys
            int _matIndex;
        };

        //! @brief A 3d object consisting of geometry information, material and texture.
        struct Object3D {
        	//! @brief Smart pointer type for Object3D.
            typedef rw::common::Ptr<Object3D> Ptr;

            /**
             * @brief constructor
             * @param name [in] name of object
             */
            Object3D(const std::string& name):
                _name(name),
                _parentObj(-1),
                //_texture(-1),
                _hasTexture(false),
                _mappedToFaces(false),
                _texOffset(0,0),
                _texRepeat(0,0),
                _materialMap(1,MaterialMapData(0,0,0))
                {};

            //! @brief test if the object is textured
            bool hasTexture() const{ return _hasTexture;}

            //! add triangle using currently selected material
            void addTriangle(const rw::geometry::IndexedTriangle<uint16_t>& tri){
                _faces.push_back(tri);
                _materialMap.back().size += 1;
            }

            //! add triangles using currently selected material
            void addTriangles(const std::vector<rw::geometry::IndexedTriangle<uint16_t> >& tris){
				uint16_t startIdx = (uint16_t) _faces.size();
				std::size_t newSize = _faces.size()+tris.size();
				if (newSize > 65535)
					RW_THROW("Model3D has two many faces! - max is 65535.");
                _faces.resize(newSize);
                for(size_t i=0;i<tris.size();i++){
                    _faces[startIdx+i] = tris[i];
                }
                _materialMap.back().size += (uint16_t)tris.size();
            }

            /**
             * @brief add triangles to this object using a specific material in the Model3D
             * @param material [in] index of the material to be used
             * @param tris [in] triangles to add
             */
            void addTriangles(uint16_t material, const std::vector<rw::geometry::IndexedTriangle<uint16_t> >& tris){
                setMaterial(material);
                uint16_t startIdx = (uint16_t)_faces.size();
				std::size_t newSize = _faces.size()+tris.size();
				if (newSize > 65535)
					RW_THROW("Model3D has two many faces! - max is 65535.");
                _faces.resize(newSize);
                for(size_t i=0;i<tris.size();i++){
                    _faces[startIdx+i] = tris[i];
                }
                _materialMap.back().size += (uint16_t)tris.size();
            }


            /**
             * @brief set the material used by addTriangles
             * @param material
             */
            void setMaterial(uint16_t material){
                if(_materialMap.size()==0 || _materialMap.back().matId!=material){
                    _materialMap.push_back( MaterialMapData(material, (uint16_t)_faces.size(), 0) );
                }
            }

            //! @brief name/id of object
            std::string _name;
            //! @brief index of parent object
            int _parentObj;
            //! true if any of the materials used has texture
            bool _hasTexture;

            //! @brief Vertice array
            std::vector<rw::math::Vector3D<float> > _vertices;
            //! @brief Normal array, there must be exactly one normal per vertex
            std::vector<rw::math::Vector3D<float> > _normals;

            /**
             * @brief Texture coordinate array, the texture coordinates can be mapped to
             * either vertices or faces. The reason for this is that often vertices
             * share two or more texcoordinates and if mapping directly to vertices then additional
             * vertices is necessary.
             */
            std::vector<rw::math::Vector2D<float> > _texCoords;
            /**
             * @brief if true then the tex coodinates are mapped to faces and not vertices. if false
             * then the texCoords are mapped to each vertice
             */
            bool _mappedToFaces;

            /**
             * @brief list containing indexed polygons. The polygons index into the
             * \b _vertices array and the \b _normals array
             * The normal is implicitly indexed and defined as same index as the
             * vertex.
             */
            std::vector<rw::geometry::IndexedTriangle<uint16_t> > _faces;

            //! @brief Mapping from triangles to materials.
            struct MaterialMapData {
            	/**
            	 * @brief Constructor.
            	 * @param m [in] material id.
            	 * @param sidx [in] start index of triangles.
            	 * @param s [in] number of triangles that use the material.
            	 */
                MaterialMapData(uint16_t m, uint16_t sidx, uint16_t s):
                    matId(m), startIdx(sidx), size(s)
                {};
                //! @brief material that is used for these triangles
                uint16_t matId;
                //! @brief the start index of the triangles
                uint16_t startIdx;
                //! @brief number of triangles from startIdx that use this material
                uint16_t size;
            };


            /**
             * @brief list containing indexed polygons. The polygons index into the
             * \b _vertices array and the \b _normals array
             * The normal is implicitly indexed and defined as same index as the
             * vertex.
             */
            std::vector<rw::geometry::IndexedPolygonN<uint16_t> > _polys;

            //! @brief Transform of the object.
            rw::math::Transform3D<float> _transform;
            //! @brief Child objects.
            std::vector<Object3D::Ptr> _kids;
            //! @brief Offset of texture.
            rw::math::Vector2D<float> _texOffset;
            //! @brief Repeat texture.
            rw::math::Vector2D<float> _texRepeat;

            /**
             * @brief maps material into a range of triangles.
             */
            std::vector<MaterialMapData> _materialMap;

            // these should be compiled
            //std::vector<MaterialFaces::Ptr> _matFaces;
            //! @brief Polygons ordered according to material.
            std::vector<MaterialPolys::Ptr> _matPolys;
            // parameter that define if geometry is rigid (static) or if it changes
        };

    public:
        //! @brief Method to do smoothing.
        typedef enum{
            AVERAGED_NORMALS //! vertex normal is determine as an avarage of all adjacent face normals
            ,WEIGHTED_NORMALS //! vertex normal is determined as AVARAGED_NORMALS, but with the face normals scaled by the face area
            } SmoothMethod;

        /**
         * @brief optimize vertices and vertice normals
         *
         * removes redundant vertices and recalculates all vertice normals based on the face normals
         * and the angle between face normals \b smooth_angle.
         * @param smooth_angle
         * @param method
         */
        void optimize(double smooth_angle, SmoothMethod method=WEIGHTED_NORMALS);

        /**
         * @brief add an Object to this Model3D
         * @param obj [in] the geometric object to add.
         * @return index of object in model3d
         */
        int addObject(Object3D::Ptr obj);

        /**
         * @brief add geometry to this model3d
         * @param mat [in] the material properties to use for the geometry.
         * @param geom [in] the geometry to add.
         */
        void addGeometry(const Material& mat, rw::common::Ptr<class rw::geometry::Geometry> geom);

        /**
         * @brief add a triangle mesh to this model3d
         * @param mat [in] the material properties to use for the mesh.
         * @param mesh [in] the mesh geometry.
         */
        void addTriMesh(const Material& mat, const rw::geometry::TriMesh& mesh);

        /**
         * @brief all objects in a model use the materials defined on the model
         * @param mat [in] material to add.
         * @return id of the newly added material.
         */
        int addMaterial(const Material& mat);

        /**
         * @brief get material with string id matid
         * @param matid [in] string id
         * @return pointer to Matrial data
         */
        Material* getMaterial(const std::string& matid);

        /**
         * @brief check if model has material with id matid
         * @param matid [in] string id of material
         * @return true if exists in model
         */
        bool hasMaterial(const std::string& matid);

        /**
         * @brief remove object with string id name
         * @param name [in] name of object to remove
         */
        void removeObject(const std::string& name);


        //! @brief get all materials that are available in this model
        std::vector<Material>& getMaterials(){ return _materials; }

        //! @brief get all objects that make out this model
        std::vector<Object3D::Ptr>& getObjects(){ return _objects; }

        //! get pose of this modle3d
        const rw::math::Transform3D<>& getTransform(){ return _transform;}
        //! set the pose of this modle3d
        void setTransform(const rw::math::Transform3D<>& t3d){ _transform = t3d;}

        //! get string identifier of this model3d
        const std::string& getName(){ return _name; }
        //! get filePath of this model3d
        const std::string& getFilePath(){ return _filePath; }
        //! set string identifier of this model3d
        void setName(const std::string& name){ _name = name; }
        //! set filePath this model3d
        void setFilePath(const std::string& name){ _filePath = name; }

        //! get mask of this model3d
        int getMask(){ return _mask; }
        //! set mask of this model3d
        void setMask(int mask){ _mask = mask; }

        /**
         * @brief convert this model3d to a geometry. Notice that geometry does not hold any
         * color information.
         * @return a geometry of this model3d
         */
        rw::geometry::GeometryData::Ptr toGeometryData();

        //! true if data in the model are expected to change
        bool isDynamic() const { return _isDynamic;}
        //! set to true if data in the model are expected to change
        void setDynamic(bool dynamic) { _isDynamic = dynamic;}

        // todo make these availabkle through proper interfaces
        //! @brief The array of materials.
        std::vector<Material> _materials;
        //! @brief The array of objects in the model
        std::vector<Object3D::Ptr> _objects;
        //! @brief The array of textures.
        std::vector<TextureData> _textures;

    protected:
        //! @brief The transform of the model.
        rw::math::Transform3D<> _transform;
        //! @brief Name of the model.
        std::string _name;
        //! @brief FilePath of the model, if model was constructed from file.
        std::string _filePath;
        //! @brief The DrawableNode::DrawableTypeMask
        int _mask;
        //! @brief Total number of vertices in the model
        int totalVerts;
        //! @brief Total number of faces in the model
        int totalFaces;
        //! @brief If the data can be expected to change.
        bool _isDynamic;
    };
    //! @}
}
}

#endif /* MODEL3D_HPP_ */
