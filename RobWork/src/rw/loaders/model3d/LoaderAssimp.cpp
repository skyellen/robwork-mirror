/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "LoaderAssimp.hpp"

#include <stack>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/DefaultLogger.hpp>
#include <assimp/LogStream.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::loaders;
using namespace rw::math;

namespace {
static Transform3D<float> toRWTransform(const aiMatrix4x4 &matrix) {
	Transform3D<float> T;
	T.P() = Vector3D<float>(matrix.a4, matrix.b4, matrix.c4);
	T.R() = Rotation3D<float>(matrix.a1, matrix.a2, matrix.a3,
			matrix.b1, matrix.b2, matrix.b3,
			matrix.c1, matrix.c2, matrix.c3);
	return T;
}
static Vector3D<float> toRWVector(const aiVector3D &vector) {
	Vector3D<float> v;
	v[0] = vector.x;
	v[1] = vector.y;
	v[2] = vector.z;
	return v;
}
}

LoaderAssimp::LoaderAssimp() {

Assimp::Logger::LogSeverity severity = Assimp::Logger::VERBOSE;
Assimp::DefaultLogger::create("",severity,aiDefaultLogStream_STDOUT);
Assimp::DefaultLogger::create("assimp_log.txt",severity,aiDefaultLogStream_FILE);
Assimp::DefaultLogger::get()->info("this is my info call");

}

LoaderAssimp::~LoaderAssimp() {
}

Model3D::Ptr LoaderAssimp::load(const std::string& filename) {
	Model3D::Ptr model = ownedPtr(new Model3D(filename));

	try {
		Assimp::Importer importer;
		const aiScene* scene = importer.ReadFile(filename, /*aiProcess_Triangulate | */ aiProcess_JoinIdenticalVertices);
		if (scene == NULL) {
			RW_THROW("Assimp could not load file " << filename << " : " << importer.GetErrorString());
		}

		// Make sure that model has none of the following definitions which we do not handle yet
		if (scene->HasAnimations())
			RW_THROW("LoaderAssimp could not load file " << filename << " : can not yet handle animations.");
		//UR5 uses this:
		//if (scene->HasCameras())
			//RW_THROW("LoaderAssimp could not load file " << filename << " : can not yet handle cameras.");
		//UR5 uses this:
		//if (scene->HasLights())
		//	RW_THROW("LoaderAssimp could not load file " << filename << " : can not yet handle lights.");
        //std::cout << "Loading using assimp:  " << filename << std::endl;
        //std::cout << "scene->HasTextures(): " << scene->HasTextures() << std::endl;
        //std::cout << "scene->HasMaterials(): " << scene->HasMaterials() << std::endl;

        if (scene->HasTextures())
			RW_THROW("LoaderAssimp could not load file " << filename << " : can not yet handle textures.");

		// Add materials
		if (scene->HasMaterials()) {
			std::vector<Model3D::Material>& materials = model->getMaterials();
			materials.resize( scene->mNumMaterials );
			for (std::size_t i = 0; i < scene->mNumMaterials; i++) {
				Model3D::Material &rwmaterial = materials[i];
				aiMaterial* material = scene->mMaterials[i];
                //for( int n=0;n<0xb;n++)
                //    std::cout << "   " << material->GetTextureCount((aiTextureType)n) << std::endl;

                //std::cout  << "DIFFUSE:" << material->GetTextureCount(aiTextureType_DIFFUSE) << std::endl;
				// Set name
				aiString name;
				if (material->Get(AI_MATKEY_NAME,name) == aiReturn_SUCCESS) {
					rwmaterial.name = name.data;
				}

				rwmaterial.simplergb = false;
				rwmaterial.texId = -1;

				aiColor3D color;
				if (material->Get(AI_MATKEY_COLOR_DIFFUSE,color) == aiReturn_SUCCESS) {
					rwmaterial.rgb[0] = color[0];
					rwmaterial.rgb[1] = color[1];
					rwmaterial.rgb[2] = color[2];
					rwmaterial.rgb[3] = 1;
				}
				if (material->Get(AI_MATKEY_COLOR_AMBIENT,color) == aiReturn_SUCCESS) {
					rwmaterial.ambient[0] = color[0];
					rwmaterial.ambient[1] = color[1];
					rwmaterial.ambient[2] = color[2];
					rwmaterial.ambient[3] = 1;
				}
				if (material->Get(AI_MATKEY_COLOR_EMISSIVE,color) == aiReturn_SUCCESS) {
					rwmaterial.emissive[0] = color[0];
					rwmaterial.emissive[1] = color[1];
					rwmaterial.emissive[2] = color[2];
					rwmaterial.emissive[3] = 1;
				}
				if (material->Get(AI_MATKEY_COLOR_SPECULAR,color) == aiReturn_SUCCESS) {
					rwmaterial.specular[0] = color[0];
					rwmaterial.specular[1] = color[1];
					rwmaterial.specular[2] = color[2];
					rwmaterial.specular[3] = 1;
				}

				material->Get(AI_MATKEY_SHININESS,rwmaterial.shininess);
				material->Get(AI_MATKEY_OPACITY,rwmaterial.transparency);
			}
		}

		// Find and add all nodes as objects
		std::vector<aiNode*> nodes;
		std::stack<aiNode*> stack;
		std::vector<Model3D::Object3D::Ptr> &objects = model->getObjects();
		stack.push(scene->mRootNode);
		while (stack.size() > 0) {
			aiNode* node = stack.top();
			stack.pop();
			nodes.push_back(node);

			Model3D::Object3D::Ptr rwobj = ownedPtr(new Model3D::Object3D(node->mName.C_Str()));
			aiMatrix4x4 matrix = node->mTransformation;
			rwobj->_transform = toRWTransform(matrix);
			objects.push_back(rwobj);

			for (std::size_t i = 0; i < node->mNumChildren; i++) {
				aiNode* child = node->mChildren[i];
				stack.push(child);
			}
		}

		// Set parents
		for (std::size_t i = 0; i < nodes.size(); i++) {
			aiNode* nodeA = nodes[i];
			int parent = -1;
			if (nodeA->mParent != NULL) {
				bool found = false;
				for (std::size_t j = 0; j < nodes.size() && !found; j++) {
					aiNode* nodeB = nodes[j];
					if (nodeA->mParent == nodeB) {
						parent = (int)j;
						found = true;
					}
				}
				if (!found)
					RW_THROW("LoaderAssimp: Could not construct dependency hierarchy between nodes.");
			}
			objects[i]->_parentObj = parent;
		}

		// Add meshes to each object
		for (std::size_t n = 0; n < nodes.size(); n++) {
			aiNode* node = nodes[n];
			Model3D::Object3D::Ptr rwobj = objects[n];
			for (std::size_t i = 0; i < node->mNumMeshes; i++) {
				aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];

/*				aiString name = mesh->mName;
				Model3D::Object3D::Ptr rwobj = ownedPtr(new Model3D::Object3D(name.data));
				rwobj->_parentObj = n;
				objects.push_back(rwobj);*/

				// Add all vertices and normals to RW object
				std::size_t startId = rwobj->_vertices.size();
				rwobj->_vertices.resize( startId + mesh->mNumVertices );
				rwobj->_normals.resize( startId + mesh->mNumVertices );
				for(size_t j=0; j < mesh->mNumVertices; j++) {
					aiVector3D vertex = mesh->mVertices[j];
					Vector3D<float> &rwvec = rwobj->_vertices[startId+j];
					rwvec[0] = vertex.x;
					rwvec[1] = vertex.y;
					rwvec[2] = vertex.z;
					if (mesh->HasNormals()) {
						aiVector3D normal = mesh->mNormals[j];
						Vector3D<float> &rwnorm = rwobj->_normals[startId+j];
						rwnorm[0] = normal.x;
						rwnorm[1] = normal.y;
						rwnorm[2] = normal.z;
					}
				}

				rwobj->setMaterial(mesh->mMaterialIndex);

				for(size_t j=0; j < mesh->mNumFaces; j++) {
					aiFace face = mesh->mFaces[j];
					if (face.mNumIndices == 0) {
						// This is stupid
						RW_THROW("LoaderAssimp could not load " << filename << " as a face was encountered with only " << face.mNumIndices << " vertices.");
					} else if (face.mNumIndices == 3) {
						IndexedTriangle<uint16_t> triangle(
							(uint16_t)(startId+face.mIndices[0]),
							(uint16_t)(startId+face.mIndices[1]),
							(uint16_t)(startId+face.mIndices[2]));
						if (!mesh->HasNormals()) {
							Vector3D<float> v0 = rwobj->_vertices[startId+face.mIndices[0]];
							Vector3D<float> v1 = rwobj->_vertices[startId+face.mIndices[1]];
							Vector3D<float> v2 = rwobj->_vertices[startId+face.mIndices[2]];
							Vector3D<float> normal = normalize(cross(v0-v1, v2-v1));
							rwobj->_normals[startId+j][0] = normal[0];
							rwobj->_normals[startId+j][1] = normal[1];
							rwobj->_normals[startId+j][2] = normal[2];
						}
						rwobj->addTriangle(triangle);
					} else {
						RW_THROW("LoaderAssimp encountered face with more than 3 indices: should not be possible!");
					}
				}
			}
		}

	}
	catch (const std::exception& exp) {
		RW_THROW("LoaderAssimp: Failed to load file: '"<<filename<<"': "<<exp.what());
	}
	catch (...) {
		RW_THROW("LoaderAssimp: Failed to load file: '"<<filename<<"'. Unknown Exception.");
	}

	return model;
}
