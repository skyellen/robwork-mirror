#include "RenderModel3D.hpp"

#include "DrawableUtil.hpp"
#include <boost/foreach.hpp>

using namespace rw::math;

using namespace rwlibs::opengl;
using namespace rw::graphics;

RenderModel3D::RenderModel3D(Model3D::Ptr model):
	_model(model)
{
    // create list of textures
    for(std::size_t i = 0; i < _model->_textures.size(); i++) {
        _textures.push_back(rw::common::ownedPtr(new RWGLTexture()));
    }
}

RenderModel3D::~RenderModel3D(){}


void RenderModel3D::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    switch (type) {
    case DrawableNode::SOLID:
    	glPolygonMode(GL_FRONT, GL_FILL);
    	//if(_model->_textures.size()>0)
    	    drawUsingSimple(info, type, alpha);
    	//else
        //    drawUsingArrays(info,type, alpha);
    	break;
    case DrawableNode::OUTLINE: // Draw nice frame
    	glPolygonMode(GL_FRONT, GL_FILL);
        //if(_model->_textures.size()>0)
            drawUsingSimple(info, type, alpha);
        //else
            //drawUsingArrays(info, type, alpha);
    case DrawableNode::WIRE:
    	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        //if(_model->_textures.size()>0)
            drawUsingSimple(info, type, alpha);
        //else
        //    drawUsingArrays(info, type, alpha);
    	break;
    }
    glPopAttrib();

}

void RenderModel3D::drawUsingSimple(const DrawableNode::RenderInfo& info, DrawType type, double alpha) const{
    glPushMatrix();
    //std::cout << "draw arrays" << std::endl;
    // Move the model
    DrawableUtil::multGLTransform( _model->getTransform() );
    //glScalef(scale, scale, scale);

    // Loop through the objects
    BOOST_FOREACH(Model3D::Object3D::Ptr &objPtr, _model->getObjects()){
        const Model3D::Object3D& obj = *objPtr;
        drawUsingSimple(info,obj,type, alpha);
    }

    glPopMatrix();
}

void RenderModel3D::drawUsingArrays(const DrawableNode::RenderInfo& info, DrawType type, double alpha) const {
	glPushMatrix();

	// Move the model
	DrawableUtil::multGLTransform( _model->getTransform() );
	//glScalef(scale, scale, scale);

	// Loop through the objects
	BOOST_FOREACH(Model3D::Object3D::Ptr &objPtr, _model->getObjects()){
		const Model3D::Object3D& obj = *objPtr;
		drawUsingArrays(info,obj,type, alpha);
	}

	glPopMatrix();
}

void RenderModel3D::drawUsingSimple(const DrawableNode::RenderInfo& info, const Model3D::Object3D &obj, DrawType type, double alpha) const {
    // for some reason opengl does not allow drawing to large meshes within glBegin/glEnd
    // so we split it up in smaller chunks
    glPushMatrix();

    DrawableUtil::multGLTransform( obj._transform );
    if(obj._normals.size()!=0 && obj._vertices.size()!=0){

        // Loop through the faces as sorted by material and draw them
        BOOST_FOREACH(const Model3D::Object3D::MaterialMapData& data, obj._materialMap){
            //const Model3D::MaterialFaces& faces = *facesptr;
            // Use the material's texture
            //RW_ASSERT(faces._matIndex<_model->_materials.size());
            //if( (_model->_materials[data.matId].transparency < 0.98) && !info._renderTransparent )
            //    continue;
            //if( (_model->_materials[data.matId].transparency >= 0.98) && !info._renderSolid )
            //    continue;

            useMaterial( _model->_materials[data.matId], type, alpha);

            if (obj.hasTexture()){
                glEnable(GL_TEXTURE_2D);
                
                if(obj._mappedToFaces){
                    glBegin(GL_TRIANGLES);
                    for(int i=data.startIdx; i<data.startIdx+data.size; i++){
                        // draw faces
                        const rw::geometry::IndexedTriangle<uint16_t> &tri = obj._faces[i];
                        glTexCoord2fv(&obj._texCoords[i*3](0) );
                        glNormal3fv(&obj._normals[tri[0]](0));
                        glVertex3fv(&obj._vertices[tri[0]](0));

                        glTexCoord2fv(&obj._texCoords[i*3+1](0) );
                        glNormal3fv(&obj._normals[tri[1]](0));
                        glVertex3fv(&obj._vertices[tri[1]](0));

                        glTexCoord2fv(&obj._texCoords[i*3+2](0) );
                        glNormal3fv(&obj._normals[tri[2]](0));
                        glVertex3fv(&obj._vertices[tri[2]](0));
                    }
                    glEnd();

                } else {
                    RW_ASSERT(obj._texCoords.size()==obj._normals.size());
                    glBegin(GL_TRIANGLES);
                    for(int i=data.startIdx; i<data.startIdx+data.size; i++){
                        // draw faces
                        const rw::geometry::IndexedTriangle<uint16_t> &tri = obj._faces[i];
                        glTexCoord2fv(&obj._texCoords[tri[0]](0) );
                        glNormal3fv(&obj._normals[tri[0]](0));
                        glVertex3fv(&obj._vertices[tri[0]](0));

                        glTexCoord2fv(&obj._texCoords[tri[1]](0) );
                        glNormal3fv(&obj._normals[tri[1]](0));
                        glVertex3fv(&obj._vertices[tri[1]](0));

                        glTexCoord2fv(&obj._texCoords[tri[2]](0) );
                        glNormal3fv(&obj._normals[tri[2]](0));
                        glVertex3fv(&obj._vertices[tri[2]](0));
                    }
                    glEnd();


                }
                glDisable(GL_TEXTURE_2D);
            } else {
                glBegin(GL_TRIANGLES);
                for(int i=data.startIdx; i<data.startIdx+data.size; i++){
                    // draw faces
                    const rw::geometry::IndexedTriangle<uint16_t> &tri = obj._faces[i];
                    glNormal3fv( &obj._normals[ tri[0] ](0) );
                    glVertex3fv( &obj._vertices[ tri[0] ](0) );
                    glNormal3fv( &obj._normals[ tri[1] ](0) );
                    glVertex3fv( &obj._vertices[ tri[1] ](0) );
                    glNormal3fv( &obj._normals[ tri[2] ](0) );
                    glVertex3fv( &obj._vertices[ tri[2] ](0) );
                }
                glEnd();
            }
            // this render the normals
            // TODO: create method to switch normal rendering on and off
            /*
            glBegin(GL_LINES);
            for(size_t i=data.startIdx; i<data.startIdx+data.size; i++){
                // draw faces
                const rw::geometry::IndexedTriangle<uint16_t> &tri = obj._faces[i];
                glVertex3fv(&obj._vertices[tri[0]](0));
                glVertex3fv(&(obj._normals[tri[0]]*0.01+obj._vertices[tri[0]])(0));

                glVertex3fv(&obj._vertices[tri[1]](0));
                glVertex3fv(&(obj._normals[tri[1]]*0.01+obj._vertices[tri[1]])(0));

                glVertex3fv(&obj._vertices[tri[2]](0));
                glVertex3fv(&(obj._normals[tri[2]]*0.01+obj._vertices[tri[2]])(0));
            }
            glEnd();
             */
        }
        //std::cout << glGetError() << std::endl;
    }

    // draw children
    // TODO: create method to switch normal rendering on and off
    BOOST_FOREACH(const Model3D::Object3D::Ptr& child, obj._kids){
        drawUsingSimple(info, *child, type, alpha);
    }

    //if (obj.hasTexture()){
        //glBindTexture(GL_TEXTURE_2D, 0 );
        //glDisable(GL_TEXTURE_2D);
    //}
    glPopMatrix();
}

void RenderModel3D::drawUsingArrays(const DrawableNode::RenderInfo& info, const Model3D::Object3D &obj, DrawType type, double alpha) const {
	//std::cout << "obj._normals.size()==" << obj._normals.size() << "\n";
	//std::cout << "obj._vertices.size()==" << obj._vertices.size() << "\n";
	glPushMatrix();
	DrawableUtil::multGLTransform( obj._transform );

	if(obj._normals.size()!=0 && obj._vertices.size()!=0){

		// Enable texture coordiantes, ngetormals, and vertices arrays
		if (obj.hasTexture()){
		    glEnable(GL_TEXTURE_2D);
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		}
		//if (lit)
		glEnableClientState(GL_NORMAL_ARRAY);

		glEnableClientState(GL_VERTEX_ARRAY);

		// Point them to the objects arrays
		if (obj.hasTexture()){
			glTexCoordPointer(2, GL_FLOAT, 0, &(obj._texCoords.at(0)[0]));
		}
		//if (lit) // if we use lightning... but we allways do
		glNormalPointer(GL_FLOAT, sizeof(Vector3D<float>), &(obj._normals.at(0)[0]));
		glVertexPointer(3, GL_FLOAT, sizeof(Vector3D<float>), &(obj._vertices.at(0)[0]));

		// Loop through the faces as sorted by material and draw them
		BOOST_FOREACH(const Model3D::Object3D::MaterialMapData& data, obj._materialMap){
			//const Model3D::MaterialFaces& faces = *facesptr;
			// Use the material's texture
			//RW_ASSERT(faces._matIndex<_model->_materials.size());
            //if( (_model->_materials[data.matId].transparency < 0.98) && !info._renderTransparent )
            //    continue;
            //if( (_model->_materials[data.matId].transparency >= 0.98) && !info._renderSolid )
            //    continue;

			useMaterial( _model->_materials[data.matId], type, alpha);

			// Draw the faces using an index to the vertex array
			glDrawElements(
				GL_TRIANGLES,
				data.size*3,
				GL_UNSIGNED_SHORT,
				&(obj._faces.at(data.startIdx)));

		}

		if (obj.hasTexture()){
		    glDisable(GL_TEXTURE_2D);
		}
	}
	// TODO: make sure polygons are allso drawn
	// draw children
	BOOST_FOREACH(const Model3D::Object3D::Ptr& child, obj._kids){
		drawUsingArrays(info, *child, type, alpha);
	}

	glPopMatrix();

	// Show the normals?
	/*
	if (_shownormals)
	{
		// Loop through the vertices and normals and draw the normal
		for (int k = 0; k < Objects.at(i).numVerts * 3; k += 3)
		{
			// Disable texturing
			glDisable(GL_TEXTURE_2D);
			// Disbale lighting if the model is lit
			if (lit)
				glDisable(GL_LIGHTING);
			// Draw the normals blue
			glColor3f(0.0f, 0.0f, 1.0f);

			// Draw a line between the vertex and the end of the normal
			glBegin(GL_LINES);
			glVertex3f(
				Objects.at(i).Vertexes.at(k),
				Objects.at(i).Vertexes.at(k+1),
				Objects.at(i).Vertexes.at(k+2));

			glVertex3f(
				Objects.at(i).Vertexes.at(k)+Objects.at(i).Normals.at(k),
				Objects.at(i).Vertexes.at(k+1)+Objects.at(i).Normals.at(k+1),
				Objects.at(i).Vertexes.at(k+2)+Objects.at(i).Normals.at(k+2));
			glEnd();

			// Reset the color to white
			glColor3f(1.0f, 1.0f, 1.0f);
			// If the model is lit then renable lighting
			if (lit)
				glEnable(GL_LIGHTING);
		}
	}
	*/

}


void RenderModel3D::useMaterial(const Model3D::Material& mat, DrawType type, double alpha) const {
    if (mat.hasTexture()){
        //glEnable(GL_TEXTURE_2D);
        //std::cout << "TexID: " << mat.getTextureID() << " ";
    	const RWGLTexture::Ptr tex = _textures[mat.getTextureID()];
    	const TextureData& tdata = _model->_textures[mat.getTextureID()];
    	if (tdata.hasImageData()) {
    		tex->init(*tdata.getImageData());
    	} else {
    		const Vector3D<float> rgb = tdata.getRGBData();
    		tex->init((unsigned char)(255*rgb[0]), (unsigned char)(255*rgb[1]), (unsigned char)(255*rgb[2]));
    	}
        glBindTexture(GL_TEXTURE_2D, tex->getTextureID() );
        //std::cout << " " << _textures[mat.getTextureID()]->getTextureID() << std::endl;
    }


    //std::cout << mat.name << std::endl;
	if(mat.simplergb){
		glColor4f(mat.rgb[0], mat.rgb[1], mat.rgb[2], (float)(mat.rgb[3]*alpha) );
	} else {
		glMaterialfv(GL_FRONT, GL_SPECULAR, mat.specular);
		glMaterialfv(GL_FRONT, GL_SHININESS, &mat.shininess);
		glMaterialfv(GL_FRONT, GL_EMISSION, mat.emissive);
        //
        // ambient and defused are controlled using glcolor by enabling glColorMaterial
		//glMaterialfv(GL_FRONT, GL_AMBIENT, mat.ambient);
        //glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
		glColor4f(mat.rgb[0], mat.rgb[1], mat.rgb[2], (float)(mat.rgb[3]*alpha) );
	}
}
