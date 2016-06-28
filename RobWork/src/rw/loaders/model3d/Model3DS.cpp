//////////////////////////////////////////////////////////////////////
//
// 3D Studio Model Class
// by: Matthew Fairfax
//
// Model3DS.cpp: implementation of the Model3DS class.
// This is a simple class for loading and viewing
// 3D Studio model files (.3ds). It supports models
// with multiple objects. It also supports multiple
// textures per object. It does not support the animation
// for 3D Studio models b/c there are simply too many
// ways for an artist to animate a 3D Studio model and
// I didn't want to impose huge limitations on the artists.
// However, I have imposed a limitation on how the models are
// textured:
// 1) Every faces must be assigned a material
// 2) If you want the face to be textured assign the
//    texture to the Diffuse Color map
// 3) The texture must be supported by the GLTexture class
//    which only supports bitmap and targa right now
// 4) The texture must be located in the same directory as
//    the model
//
// Support for non-textured faces is done by reading the color
// from the material's diffuse color.
//
// Some models have problems loading even if you follow all of
// the restrictions I have stated and I don't know why. If you
// can import the 3D Studio file into Milkshape 3D
// (http://www.swissquake.ch/chumbalum-soft) and then export it
// to a new 3D Studio file. This seems to fix many of the problems
// but there is a limit on the number of faces and vertices Milkshape 3D
// can read.
//
// Usage:
// Model3DS m;
//
// m.Load("model.3ds"); // Load the model
// m.Draw();                    // Renders the model to the screen
//
// // If you want to show the model's normals
// m.shownormals = true;
//
// // If the model is not going to be lit then set the lit
// // variable to false. It defaults to true.
// m.lit = false;
//
// // You can disable the rendering of the model
// m.visible = false;
//
// // You can move and rotate the model like this:
// m.rot.x = 90.0f;
// m.rot.y = 30.0f;
// m.rot.z = 0.0f;
//
// m.pos.x = 10.0f;
// m.pos.y = 0.0f;
// m.pos.z = 0.0f;
//
// // If you want to move or rotate individual objects
// m.Objects.at(0).rot.x = 90.0f;
// m.Objects.at(0).rot.y = 30.0f;
// m.Objects.at(0).rot.z = 0.0f;
//
// m.Objects.at(0).pos.x = 10.0f;
// m.Objects.at(0).pos.y = 0.0f;
// m.Objects.at(0).pos.z = 0.0f;
//
//////////////////////////////////////////////////////////////////////

// This is used to generate a warning from the compiler
#define _QUOTE(x) # x
#define QUOTE(x) _QUOTE(x)
#define __FILE__LINE__ __FILE__ "(" QUOTE(__LINE__) ") : "
#define warn( x )  message( __FILE__LINE__ #x "\n" )

// You need to uncomment this if you are using MFC
//#pragma warn( You need to uncomment this if you are using MFC )
//#include "stdafx.h"

#include "Model3DS.hpp"

#include <rw/loaders/ImageLoader.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

#include <cstdio>

using namespace rw::common;
using namespace rw::graphics;

// The chunk's id numbers
#define MAIN3DS                         0x4D4D
#define MAIN_VERS                      0x0002
#define EDIT3DS                        0x3D3D
#define MESH_VERS                     0x3D3E
#define OBJECT                        0x4000
#define TRIG_MESH            0x4100
#define VERT_LIST           0x4110
#define FACE_DESC           0x4120
#define FACE_MAT           0x4130
#define TEX_VERTS           0x4140
#define SMOOTH_GROUP       0x4150
#define LOCAL_COORDS        0x4160
#define MATERIAL                      0xAFFF
#define MAT_NAME                     0xA000
#define MAT_AMBIENT          0xA010
#define MAT_DIFFUSE          0xA020
#define MAT_SPECULAR         0xA030
#define SHINY_PERC           0xA040
#define SHINY_STR_PERC       0xA041
#define TRANS_PERC           0xA050
#define TRANS_FOFF_PERC      0xA052
#define REF_BLUR_PERC        0xA053
#define RENDER_TYPE          0xA100
#define SELF_ILLUM           0xA084
#define MAT_SELF_ILPCT       0xA08A
#define WIRE_THICKNESS       0xA087
#define MAT_TEXMAP           0xA200
#define MAT_MAPNAME         0xA300
#define ONE_UNIT                      0x0100
#define KEYF3DS                        0xB000
#define FRAMES                        0xB008
#define MESH_INFO                     0xB002
#define HIER_POS                     0xB030
#define HIER_FATHER          0xB010
#define PIVOT_PT                     0xB013
#define TRACK00                      0xB020
#define TRACK01                      0xB021
#define TRACK02                      0xB022
#define COLOR_RGB                       0x0010
#define COLOR_TRU                       0x0011
#define COLOR_TRUG                      0x0012
#define COLOR_RGBG                      0x0013
#define PERC_INT                        0x0030
#define PERC_FLOAT                      0x0031

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Model3DS::Model3DS()
{
    // Initialization

    // Don't show the normals by default
    shownormals = false;

    // The model is lit by default
    lit = true;

    // The model is visible by default
    visible = true;

    // Set up the default position
    pos.x = 0.0f;
    pos.y = 0.0f;
    pos.z = 0.0f;
    // Set up the default rotation
    rot.x = 0.0f;
    rot.y = 0.0f;
    rot.z = 0.0f;

    // Set up the path
    path.resize(80);
    path.at(0) = '\0';

    // Zero out our counters for MFC
    numObjects = 0;
    numMaterials = 0;

    // Set the scale to one
    scale = 1.0f;
}

Model3DS::~Model3DS()
{}

void Model3DS::Load(const std::string& name)
{
    // holds the main chunk header
    ChunkHeader main;

    path = StringUtil::getDirectoryName(name);

    // Load the file
    bin3ds = fopen(name.c_str(), "rb");

    if (!bin3ds)
        RW_THROW(
            "Can't open file "
            << StringUtil::quote(name));


    // Make sure we are at the beginning
    fseek(bin3ds, 0, SEEK_SET);
    // Load the Main Chunk's header

    _retval = fread(&main.id,sizeof(main.id),1,bin3ds);
    _retval = fread(&main.len,sizeof(main.len),1,bin3ds);
    // Start Processing
    MainChunkProcessor(main.len, ftell(bin3ds));
    // Don't need the file anymore so close it
    fclose(bin3ds);
    // Calculate the vertex normals
    CalculateNormals();
    // Find the total number of faces and vertices
    totalFaces = 0;
    totalVerts = 0;

    for (int i = 0; i < numObjects; i ++)
    {
        totalFaces += Objects.at(i).numFaces/3;
        totalVerts += Objects.at(i).numVerts;
    }

    // If the object doesn't have any texcoords generate some
    for (int k = 0; k < numObjects; k++)
    {
        if (Objects.at(k).numTexCoords == 0)
        {
            // Set the number of texture coords
            Objects.at(k).numTexCoords = Objects.at(k).numVerts;

            // Allocate an array to hold the texture coordinates
            const int len = Objects.at(k).numTexCoords * 2;
            Objects.at(k).TexCoords.resize(len);

            // Make some texture coords
            for (int m = 0; m < Objects.at(k).numTexCoords; m++)
            {
                Objects.at(k).TexCoords.at(2*m) = Objects.at(k).Vertexes.at(3*m);
                Objects.at(k).TexCoords.at(2*m+1) = Objects.at(k).Vertexes.at(3*m+1);
            }
        }
    }

    std::cout<<"Number of Materials = "<<numMaterials<<std::endl;
    // Let's build simple colored textures for the materials w/o a texture
    for (int j = 0; j < numMaterials; j++)
    {
        if (Materials.at(j).textured == false)
        {
            unsigned char r = Materials.at(j).color.r;
            unsigned char g = Materials.at(j).color.g;
            unsigned char b = Materials.at(j).color.b;
            Materials.at(j).tex = TextureData("RGB", (float)(r/256.0), (float)(g/256.0), (float)(b/256.0));
            Materials.at(j).textured = true;
        }
    }
}
/*
void Model3DS::Draw()
{
    if (visible)
    {
        glPushMatrix();

        // Move the model
        glTranslatef(pos.x, pos.y, pos.z);

        // Rotate the model
        glRotatef(rot.x, 1.0f, 0.0f, 0.0f);
        glRotatef(rot.y, 0.0f, 1.0f, 0.0f);
        glRotatef(rot.z, 0.0f, 0.0f, 1.0f);

        glScalef(scale, scale, scale);

        // Loop through the objects
        for (int i = 0; i < numObjects; i++)
        {
            // Enable texture coordiantes, normals, and vertices arrays
            if (Objects.at(i).textured)
                glEnableClientState(GL_TEXTURE_COORD_ARRAY);
            if (lit)
                glEnableClientState(GL_NORMAL_ARRAY);
            glEnableClientState(GL_VERTEX_ARRAY);

            // Point them to the objects arrays
            if (Objects.at(i).textured)
                glTexCoordPointer(2, GL_FLOAT, 0, &Objects.at(i).TexCoords.at(0));
            if (lit)
                glNormalPointer(
                    GL_FLOAT,
                    0,
                    &Objects.at(i).Normals.at(0));
            glVertexPointer(
                3,
                GL_FLOAT,
                0,
                &Objects.at(i).Vertexes.at(0));

            // Loop through the faces as sorted by material and draw them
            for (int j = 0; j < Objects.at(i).numMatFaces; j ++)
            {
                // Use the material's texture
                Materials.at(Objects.at(i).MatFaces.at(j).MatIndex).tex.Use();

                glPushMatrix();

                // Move the model
                glTranslatef(Objects.at(i).pos.x, Objects.at(i).pos.y, Objects.at(i).pos.z);

                // Rotate the model
                //glRotatef(Objects.at(i).rot.x, 1.0f, 0.0f, 0.0f);
                //glRotatef(Objects.at(i).rot.y, 0.0f, 1.0f, 0.0f);
                //glRotatef(Objects.at(i).rot.z, 0.0f, 0.0f, 1.0f);

                glRotatef(Objects.at(i).rot.z, 0.0f, 0.0f, 1.0f);
                glRotatef(Objects.at(i).rot.y, 0.0f, 1.0f, 0.0f);
                glRotatef(Objects.at(i).rot.x, 1.0f, 0.0f, 0.0f);

                // Draw the faces using an index to the vertex array
                glDrawElements(
                    GL_TRIANGLES,
                    Objects.at(i).MatFaces.at(j).numSubFaces,
                    GL_UNSIGNED_SHORT,
                    &Objects.at(i).MatFaces.at(j).subFaces.at(0));

                glPopMatrix();
            }

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
        }

        glPopMatrix();
    }
}
*/

void Model3DS::CalculateNormals()
{
    // Let's build some normals
    for (int i = 0; i < numObjects; i++)
    {
        for (int g = 0; g < Objects.at(i).numVerts; g++)
        {
            // Reduce each vert's normal to unit
            float length;
            Vector unit;

            unit.x = Objects.at(i).Normals.at(g*3);
            unit.y = Objects.at(i).Normals.at(g*3+1);
            unit.z = Objects.at(i).Normals.at(g*3+2);

            length = (float)sqrt((unit.x*unit.x) + (unit.y*unit.y) + (unit.z*unit.z));

            if (length == 0.0f)
                length = 1.0f;

            unit.x /= length;
            unit.y /= length;
            unit.z /= length;

            Objects.at(i).Normals.at(g*3)   = unit.x;
            Objects.at(i).Normals.at(g*3+1) = unit.y;
            Objects.at(i).Normals.at(g*3+2) = unit.z;
        }
    }
}

void Model3DS::MainChunkProcessor(long length, long findex)
{
    ChunkHeader h;

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    while (ftell(bin3ds) < (findex + length - 6))
    {
       // std::cout<<"ftell = "<<ftell(bin3ds)<<std::endl;
        _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
        _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

        switch (h.id)
        {
            // This is the mesh information like vertices, faces, and materials
        case EDIT3DS    :
            EditChunkProcessor(h.len, ftell(bin3ds));
            break;
            // I left this in case anyone gets very ambitious
        case KEYF3DS    :
            //KeyFrameChunkProcessor(h.len, ftell(bin3ds));
            break;
        default                 :
            break;
        }

        fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place

    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::EditChunkProcessor(long length, long findex)
{
    ChunkHeader h;

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    // First count the number of Objects and Materials
    while (ftell(bin3ds) < (findex + length - 6))
    {
        _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
        _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

        switch (h.id)
        {
        case OBJECT     :
            numObjects++;
            break;
        case MATERIAL   :
            numMaterials++;
            break;
        default                 :
            break;
        }

        fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

    // Now load the materials
    if (numMaterials > 0)
    {
        Materials.resize(numMaterials);

        // Material is set to untextured until we find otherwise
        for (int d = 0; d < numMaterials; d++)
            Materials.at(d).textured = false;

        fseek(bin3ds, findex, SEEK_SET);

        int i = 0;

        while (ftell(bin3ds) < (findex + length - 6))
        {
            _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
            _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

            switch (h.id)
            {
            case MATERIAL   :
                MaterialChunkProcessor(h.len, ftell(bin3ds), i);
                i++;
                break;
            default                 :
                break;
            }

            fseek(bin3ds, (h.len - 6), SEEK_CUR);
        }
    }

    // Load the Objects (individual meshes in the whole model)
    if (numObjects > 0)
    {
        Objects.resize(numObjects);

        // Set the textured variable to false until we find a texture
        for (int k = 0; k < numObjects; k++)
            Objects.at(k).textured = false;

        // Zero the objects position and rotation
        for (int m = 0; m < numObjects; m++)
        {
            Objects.at(m).pos.x = 0.0f;
            Objects.at(m).pos.y = 0.0f;
            Objects.at(m).pos.z = 0.0f;

            Objects.at(m).rot.x = 0.0f;
            Objects.at(m).rot.y = 0.0f;
            Objects.at(m).rot.z = 0.0f;
        }

        // Zero out the number of texture coords
        for (int n = 0; n < numObjects; n++)
            Objects.at(n).numTexCoords = 0;

        fseek(bin3ds, findex, SEEK_SET);

        int j = 0;

        while (ftell(bin3ds) < (findex + length - 6))
        {
            _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
            _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

            switch (h.id)
            {
            case OBJECT     :
                ObjectChunkProcessor(h.len, ftell(bin3ds), j);
                j++;
                break;
            default                 :
                break;
            }

            fseek(bin3ds, (h.len - 6), SEEK_CUR);
        }
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::MaterialChunkProcessor(long length, long findex, int matindex)
{
    ChunkHeader h;

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    while (ftell(bin3ds) < (findex + length - 6))
    {
        _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
        _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

        switch (h.id)
        {
        case MAT_NAME   :
            // Loads the material's names
            MaterialNameChunkProcessor(h.len, ftell(bin3ds), matindex);
            break;
        case MAT_AMBIENT        :
            //ColorChunkProcessor(h.len, ftell(bin3ds));
            break;
        case MAT_DIFFUSE        :
            DiffuseColorChunkProcessor(h.len, ftell(bin3ds), matindex);
            break;
        case MAT_SPECULAR       :
            //ColorChunkProcessor(h.len, ftell(bin3ds));
        case MAT_TEXMAP :
            // Finds the names of the textures of the material and loads them
            TextureMapChunkProcessor(h.len, ftell(bin3ds), matindex);
            break;
        default                 :
            break;
        }

        fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::MaterialNameChunkProcessor(long length, long findex, int matindex)
{
    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    // Read the material's name
    for (int i = 0; i < 80; i++)
    {
        Materials.at(matindex).name[i] = fgetc(bin3ds);
        if (Materials.at(matindex).name[i] == 0)
        {
            //Materials.at(matindex).name[i] = NULL;
            break;
        }
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::DiffuseColorChunkProcessor(long length, long findex, int matindex)
{
    ChunkHeader h;

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    while (ftell(bin3ds) < (findex + length - 6))
    {
        _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
        _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

        // Determine the format of the color and load it
        switch (h.id)
        {
        case COLOR_RGB  :
            // A rgb float color chunk
            FloatColorChunkProcessor(h.len, ftell(bin3ds), matindex);
            break;
        case COLOR_TRU  :
            // A rgb int color chunk
            IntColorChunkProcessor(h.len, ftell(bin3ds), matindex);
            break;
        case COLOR_RGBG :
            // A rgb gamma corrected float color chunk
            FloatColorChunkProcessor(h.len, ftell(bin3ds), matindex);
            break;
        case COLOR_TRUG :
            // A rgb gamma corrected int color chunk
            IntColorChunkProcessor(h.len, ftell(bin3ds), matindex);
            break;
        default                 :
            break;
        }

        fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::FloatColorChunkProcessor(long length, long findex, int matindex)
{
    float r;
    float g;
    float b;

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    _retval = fread(&r,sizeof(r),1,bin3ds);
    _retval = fread(&g,sizeof(g),1,bin3ds);
    _retval = fread(&b,sizeof(b),1,bin3ds);

    Materials.at(matindex).color.r = (unsigned char)(r*255.0f);
    Materials.at(matindex).color.g = (unsigned char)(r*255.0f);
    Materials.at(matindex).color.b = (unsigned char)(r*255.0f);
    Materials.at(matindex).color.a = 255;

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::IntColorChunkProcessor(long length, long findex, int matindex)
{
    unsigned char r;
    unsigned char g;
    unsigned char b;

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    _retval = fread(&r,sizeof(r),1,bin3ds);
    _retval = fread(&g,sizeof(g),1,bin3ds);
    _retval = fread(&b,sizeof(b),1,bin3ds);

    Materials.at(matindex).color.r = r;
    Materials.at(matindex).color.g = g;
    Materials.at(matindex).color.b = b;
    Materials.at(matindex).color.a = 255;

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::TextureMapChunkProcessor(long length, long findex, int matindex)
{
    ChunkHeader h;

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    while (ftell(bin3ds) < (findex + length - 6))
    {
        _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
        _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

        switch (h.id)
        {
        case MAT_MAPNAME:
            // Read the name of texture in the Diffuse Color map
            MapNameChunkProcessor(h.len, ftell(bin3ds), matindex);
            break;
        default                 :
            break;
        }

        fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::MapNameChunkProcessor(long length, long findex, int matindex)
{
    char name[80];

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    // Read the name of the texture
    for (int i = 0; i < 80; i++)
    {
        name[i] = fgetc(bin3ds);
        if (name[i] == 0)
        {
            //name[i] = NULL;
            break;
        }
    }

    // Load the name and indicate that the material has a texture
    char fullname[80];
    sprintf(fullname, "%s%s", &path.at(0), name);
	rw::sensor::Image::Ptr img = rw::loaders::ImageLoader::Factory::load(fullname);
    Materials.at(matindex).tex = TextureData(name, img);
    Materials.at(matindex).textured = true;

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::ObjectChunkProcessor(long length, long findex, int objindex)
{
    ChunkHeader h;

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    // Load the object's name
    for (int i = 0; i < 80; i++)
    {
        Objects.at(objindex).name[i] = fgetc(bin3ds);
        if (Objects.at(objindex).name[i] == 0)
        {
            //Objects.at(objindex).name[i] = NULL;
            break;
        }
    }

    while (ftell(bin3ds) < (findex + length - 6))
    {
        _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
        _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

        switch (h.id)
        {
        case TRIG_MESH  :
            // Process the triangles of the object
            TriangularMeshChunkProcessor(h.len, ftell(bin3ds), objindex);
            break;
        default                 :
            break;
        }

        fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::TriangularMeshChunkProcessor(long length, long findex, int objindex)
{
    ChunkHeader h;

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    while (ftell(bin3ds) < (findex + length - 6))
    {
        _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
        _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

        switch (h.id)
        {
        case VERT_LIST  :
            // Load the vertices of the onject
            VertexListChunkProcessor(h.len, ftell(bin3ds), objindex);
            break;
        case LOCAL_COORDS       :
            //LocalCoordinatesChunkProcessor(h.len, ftell(bin3ds));
            break;
        case TEX_VERTS  :
            // Load the texture coordinates for the vertices
            TexCoordsChunkProcessor(h.len, ftell(bin3ds), objindex);
            Objects.at(objindex).textured = true;
            break;
        default                 :
            break;
        }

        fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

    // After we have loaded the vertices we can load the faces
    fseek(bin3ds, findex, SEEK_SET);

    while (ftell(bin3ds) < (findex + length - 6))
    {
        _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
        _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

        switch (h.id)
        {
        case FACE_DESC  :
            // Load the faces of the object

            FacesDescriptionChunkProcessor(h.len, ftell(bin3ds), objindex);

            break;
        default                 :
            break;
        }

        fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::VertexListChunkProcessor(long length, long findex, int objindex)
{
    unsigned short numVerts;

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    // Read the number of vertices of the object
    _retval = fread(&numVerts,sizeof(numVerts),1,bin3ds);

    // Allocate arrays for the vertices and normals
    Objects.at(objindex).Vertexes.resize(numVerts * 3);
    Objects.at(objindex).Normals.resize(numVerts * 3);

    // Assign the number of vertices for future use
    Objects.at(objindex).numVerts = numVerts;

    // Zero out the normals array
    for (int j = 0; j < numVerts * 3; j++)
        Objects.at(objindex).Normals.at(j) = 0.0f;

    // Read the vertices, switching the y and z coordinates and changing the
    // sign of the z coordinate
    for (int i = 0; i < numVerts * 3; i+=3)
    {
        _retval = fread(&Objects.at(objindex).Vertexes.at(i),sizeof(float),1,bin3ds);
        _retval = fread(&Objects.at(objindex).Vertexes.at(i+2),sizeof(float),1,bin3ds);
        _retval = fread(&Objects.at(objindex).Vertexes.at(i+1),sizeof(float),1,bin3ds);

        // Change the sign of the z coordinate
        Objects.at(objindex).Vertexes.at(i+2) = -Objects.at(objindex).Vertexes.at(i+2);
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::TexCoordsChunkProcessor(long length, long findex, int objindex)
{
    // The number of texture coordinates
    unsigned short numCoords;

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    // Read the number of coordinates
    _retval = fread(&numCoords, sizeof(numCoords), 1, bin3ds);

    // Allocate an array to hold the texture coordinates
    Objects.at(objindex).TexCoords.resize(numCoords * 2);

    // Set the number of texture coords
    Objects.at(objindex).numTexCoords = numCoords;

    // Read teh texture coordiantes into the array
    for (int i = 0; i < numCoords * 2; i+=2)
    {
        _retval = fread(&Objects.at(objindex).TexCoords.at(i),sizeof(float),1,bin3ds);
        _retval = fread(&Objects.at(objindex).TexCoords.at(i+1),sizeof(float),1,bin3ds);
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::FacesDescriptionChunkProcessor(long length, long findex, int objindex)
{

    ChunkHeader h;
    unsigned short numFaces;        // The number of faces in the object
    unsigned short vertA;           // The first vertex of the face
    unsigned short vertB;           // The second vertex of the face
    unsigned short vertC;           // The third vertex of the face
    unsigned short flags;           // The winding order flags
    long subs;                                      // Holds our place in the file
    int numMatFaces = 0;            // The number of different materials

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    // Read the number of faces
    _retval = fread(&numFaces,sizeof(numFaces),1,bin3ds);

    // Allocate an array to hold the faces
    Objects.at(objindex).Faces.resize(numFaces * 3);
    // Store the number of faces
    Objects.at(objindex).numFaces = numFaces * 3;


    // Read the faces into the array
    for (int i = 0; i < numFaces * 3; i+=3)
    {
        // Read the vertices of the face
        _retval = fread(&vertA,sizeof(vertA),1,bin3ds);
        _retval = fread(&vertB,sizeof(vertB),1,bin3ds);
        _retval = fread(&vertC,sizeof(vertC),1,bin3ds);
        _retval = fread(&flags,sizeof(flags),1,bin3ds);

        // Place them in the array
        Objects.at(objindex).Faces.at(i)   = vertA;
        Objects.at(objindex).Faces.at(i+1) = vertB;
        Objects.at(objindex).Faces.at(i+2) = vertC;

        // Calculate the face's normal
        Vector n;
        Vertex v1;
        Vertex v2;
        Vertex v3;

        v1.x = Objects.at(objindex).Vertexes.at(vertA*3);
        v1.y = Objects.at(objindex).Vertexes.at(vertA*3+1);
        v1.z = Objects.at(objindex).Vertexes.at(vertA*3+2);
        v2.x = Objects.at(objindex).Vertexes.at(vertB*3);
        v2.y = Objects.at(objindex).Vertexes.at(vertB*3+1);
        v2.z = Objects.at(objindex).Vertexes.at(vertB*3+2);
        v3.x = Objects.at(objindex).Vertexes.at(vertC*3);
        v3.y = Objects.at(objindex).Vertexes.at(vertC*3+1);
        v3.z = Objects.at(objindex).Vertexes.at(vertC*3+2);

        // calculate the normal
        float u[3], v[3];

        // V2 - V3;
        u[0] = v2.x - v3.x;
        u[1] = v2.y - v3.y;
        u[2] = v2.z - v3.z;

        // V2 - V1;
        v[0] = v2.x - v1.x;
        v[1] = v2.y - v1.y;
        v[2] = v2.z - v1.z;

        n.x = (u[1]*v[2] - u[2]*v[1]);
        n.y = (u[2]*v[0] - u[0]*v[2]);
        n.z = (u[0]*v[1] - u[1]*v[0]);

        // Add this normal to its verts' normals
        Objects.at(objindex).Normals.at(vertA*3)   += n.x;
        Objects.at(objindex).Normals.at(vertA*3+1) += n.y;
        Objects.at(objindex).Normals.at(vertA*3+2) += n.z;
        Objects.at(objindex).Normals.at(vertB*3)   += n.x;
        Objects.at(objindex).Normals.at(vertB*3+1) += n.y;
        Objects.at(objindex).Normals.at(vertB*3+2) += n.z;
        Objects.at(objindex).Normals.at(vertC*3)   += n.x;
        Objects.at(objindex).Normals.at(vertC*3+1) += n.y;
        Objects.at(objindex).Normals.at(vertC*3+2) += n.z;
    }


    // Store our current file position
    subs = ftell(bin3ds);

    // Check to see how many materials the faces are split into
    while (ftell(bin3ds) < (findex + length - 6))
    {
        _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
        _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

        switch (h.id)
        {
        case FACE_MAT   :
            //FacesMaterialsListChunkProcessor(h.len, ftell(bin3ds), objindex);
            numMatFaces++;
            break;
        default                 :
            break;
        }

        fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }


    // Split the faces up according to their materials
    if (numMatFaces > 0)
    {
        // Allocate an array to hold the lists of faces divided by material
        Objects.at(objindex).MatFaces.resize(numMatFaces);
        // Store the number of material faces
        Objects.at(objindex).numMatFaces = numMatFaces;

        fseek(bin3ds, subs, SEEK_SET);

        int j = 0;

        // Split the faces up
        while (ftell(bin3ds) < (findex + length - 6))
        {
            _retval = fread(&h.id,sizeof(h.id),1,bin3ds);
            _retval = fread(&h.len,sizeof(h.len),1,bin3ds);

            switch (h.id)
            {
            case FACE_MAT   :
                // Process the faces and split them up
                FacesMaterialsListChunkProcessor(h.len, ftell(bin3ds), objindex, j);
                j++;
                break;
            default                 :
                break;
            }

            fseek(bin3ds, (h.len - 6), SEEK_CUR);
        }
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::FacesMaterialsListChunkProcessor(
    long length, long findex, int objindex, int subfacesindex)
{
    char name[80];                          // The material's name
    unsigned short numEntries;      // The number of faces associated with this material
    unsigned short Face;            // Holds the faces as they are read
    int material; // An index to the Materials array for this material

    // move the file pointer to the beginning of the main
    // chunk's data findex + the size of the header
    fseek(bin3ds, findex, SEEK_SET);

    int i;
    // Read the material's name
    for (i = 0; i < 80; i++)
    {
        name[i] = fgetc(bin3ds);
        if (name[i] == 0)
        {
            //name[i] = NULL;
            break;
        }
    }

    // Faind the material's index in the Materials array
    for (material = 0; material < numMaterials; material++)
    {
        if (strcmp(name, Materials.at(material).name) == 0)
            break;
    }

    // Store this value for later so that we can find the material
    Objects.at(objindex).MatFaces.at(subfacesindex).MatIndex = material;

    // Read the number of faces associated with this material
    _retval = fread(&numEntries, sizeof(numEntries), 1, bin3ds);

    // Allocate an array to hold the list of faces associated with this material
    Objects.at(objindex).MatFaces.at(subfacesindex).subFaces.resize(
        numEntries * 3);

    // Store this number for later use
    Objects.at(objindex).MatFaces.at(subfacesindex).numSubFaces = numEntries * 3;

    // Read the faces into the array
    for (i = 0; i < numEntries * 3; i+=3)
    {
        // read the face
        _retval = fread(&Face,sizeof(Face),1,bin3ds);
        // Add the face's vertices to the list
        Objects.at(objindex).MatFaces.at(subfacesindex).subFaces.at(i) =
            Objects.at(objindex).Faces.at(Face * 3);
        Objects.at(objindex).MatFaces.at(subfacesindex).subFaces.at(i+1) =
            Objects.at(objindex).Faces.at(Face * 3 + 1);
        Objects.at(objindex).MatFaces.at(subfacesindex).subFaces.at(i+2) =
            Objects.at(objindex).Faces.at(Face * 3 + 2);
    }

    // move the file pointer back to where we got it so
    // that the ProcessChunk() which we interrupted will read
    // from the right place
    fseek(bin3ds, findex, SEEK_SET);
}
