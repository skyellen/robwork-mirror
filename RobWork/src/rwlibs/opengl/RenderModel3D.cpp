#include "RenderModel3D.hpp"

#include "DrawableUtil.hpp"
#include <boost/foreach.hpp>
#include <rw/math/Math.hpp>

using namespace rw::math;

using namespace boost;

using namespace rwlibs::opengl;
using namespace rw::graphics;

RenderModel3D::RenderModel3D(Model3D::Ptr model):
	_model(model)
{
	//std::cout << "Render model created: \n";
	//std::cout << "nr mat: " << model->_materials.size() << std::endl;
	//std::cout << "nr obj: " << model->_objects.size() << std::endl;
}

RenderModel3D::~RenderModel3D(){}


void RenderModel3D::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const{

    switch (type) {
    case DrawableNode::SOLID:
    	glPolygonMode(GL_FRONT, GL_FILL);
    	drawUsingArrays(type, alpha);
    	break;
    case DrawableNode::OUTLINE: // Draw nice frame
    	glPolygonMode(GL_FRONT, GL_FILL);
    	drawUsingArrays(type, alpha);
    case DrawableNode::WIRE:
    	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    	drawUsingArrays(type, alpha);
    	break;
    }

}

void RenderModel3D::drawUsingArrays(DrawType type, double alpha) const {
	glPushMatrix();
	//std::cout << "draw arrays" << std::endl;
	// Move the model
	DrawableUtil::multGLTransform( _model->getTransform() );
	//glScalef(scale, scale, scale);

	// Loop through the objects
	BOOST_FOREACH(Model3D::Object3D::Ptr &objPtr, _model->getObjects()){
		const Model3D::Object3D& obj = *objPtr;
		drawUsingArrays(obj,type, alpha);
	}

	glPopMatrix();
}

void RenderModel3D::drawUsingArrays(const Model3D::Object3D &obj, DrawType type, double alpha) const {
	//std::cout << "obj._normals.size()==" << obj._normals.size() << "\n";
	//std::cout << "obj._vertices.size()==" << obj._vertices.size() << "\n";

	glPushMatrix();
	DrawableUtil::multGLTransform( obj._transform );

	if(obj._normals.size()!=0 && obj._vertices.size()!=0){

		// Enable texture coordiantes, normals, and vertices arrays
		//if (obj.hasTexture())
		//	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		//if (lit)
		glEnableClientState(GL_NORMAL_ARRAY);

		glEnableClientState(GL_VERTEX_ARRAY);

		// Point them to the objects arrays
		//if (obj.hasTexture())
		//	glTexCoordPointer(2, GL_FLOAT, 0, &(obj._texCoords.at(0)[0]));
		//if (lit) // if we use lightning... but we allways do
		glNormalPointer(GL_FLOAT, sizeof(Vector3D<float>), &(obj._normals.at(0)[0]));
		glVertexPointer(3, GL_FLOAT, sizeof(Vector3D<float>), &(obj._vertices.at(0)[0]));

		// Loop through the faces as sorted by material and draw them
		BOOST_FOREACH(const Model3D::Object3D::MaterialMapData& data, obj._materialMap){
			//const Model3D::MaterialFaces& faces = *facesptr;
			// Use the material's texture
			//RW_ASSERT(faces._matIndex<_model->_materials.size());
			useMaterial( _model->_materials[data.matId], type, alpha);

			// Draw the faces using an index to the vertex array
			glDrawElements(
				GL_TRIANGLES,
				data.size*3,
				GL_UNSIGNED_SHORT,
				&(obj._faces.at(data.startIdx)));

		}
	}
	// TODO: make sure polygons are allso drawn
	// draw children
	BOOST_FOREACH(const Model3D::Object3D::Ptr& child, obj._kids){
		drawUsingArrays(*child, type, alpha);
	}

	glPopMatrix();
/*
	// Show the normals?
	if (shownormals)
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
