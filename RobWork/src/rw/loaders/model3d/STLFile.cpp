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


#include <rw/loaders/model3d/STLFile.hpp>
#include <rw/geometry/TriangleUtil.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/math/Vector3D.hpp>

#include <iostream>
#include <string>
#include <fstream>

using namespace rw::loaders;
using namespace rw::geometry;
using namespace rw::common;
using namespace rw::math;

namespace
{
    // We have this utility here so that we can be sure that IO errors don't
    // take place. It is nasty for IO errors to not be discovered. Also we can
    // speed up things by reading the input stream into a buffer, if we want to,
    // without having to change the user of Reader.
    class Reader
    {
    public:
        Reader(std::ifstream* in) : _in(in)
        {
            RW_ASSERT(in);
            okOrThrow();
        }

        void read(char* out, int size)
        {
            in().read(out, size);
            okOrThrow();
        }

        void ignore(int cnt)
        {
            in().ignore(cnt);
            okOrThrow();
        }

        void get()
        {
            in().get();
            okOrThrow();
        }

    private:
        bool ok() { return !in().fail(); }
        void okOrThrow() { if (!ok()) RW_THROW("IO error."); }

        std::ifstream& in() { return *_in; }
        std::ifstream* _in;
    };

    void readVec(rw::math::Vector3D<float>& vec, Reader& input)
    {
        for (int i = 0; i < 3; i++ )
            input.read((char *)&vec[i], sizeof(float));
    }

    //
    //  Purpose:
    //
    //    STLB_READ reads a binary STL (stereolithography) file.
    //
    //  Example:
    //
    //    80 byte string = header containing nothing in particular
    //
    //    4 byte int = number of faces
    //
    //    For each face:
    //      3 4-byte floats = components of normal vector to face;
    //      3 4-byte floats = coordinates of first node;
    //      3 4-byte floats = coordinates of second node;
    //      3 4-byte floats = coordinates of third and final node;
    //        2-byte int = attribute, whose value is 0.
    //
    //  Reference:
    //    3D Systems, Inc,
    //    Stereolithography Interface Specification,
    //    October 1989.
    //
    void ReadBinarySTL(
        Reader& input,
        PlainTriMesh<TriangleN1<float> >& result)
    {
        input.ignore(80);

        //
        //  Number of faces.
        //
        int face_num;
        input.read((char*)&face_num, 4);
        result.resize(face_num);

        //
        //  For each (triangular) face,
        //    components of normal vector,
        //    coordinates of three vertices,
        //    2 byte "attribute".
        //
        for (int cnt = 0; cnt < face_num; cnt++) {
            TriangleN1<float>& face = result[cnt];
            readVec(face.getFaceNormal(), input);
            readVec(face[0], input);
            readVec(face[1], input);
            readVec(face[2], input);

            // Read 2 byte attribute - which is not used
            input.get();
            input.get();
        }
    }


    const int LINE_MAX_LENGTH = 200;
    const char TOKEN_vertex[] = "vertex";
    const char TOKEN_normal[] = "normal";


    //
    //  Purpose:
    //
    //    STLA_READ reads an ASCII STL (stereolithography) file.
    //
    //  Examples:
    //
    //    solid MYSOLID
    //      facet normal 0.4 0.4 0.2
    //        outerloop
    //          vertex  1.0 2.1 3.2
    //          vertex  2.1 3.7 4.5
    //          vertex  3.1 4.5 6.7
    //        endloop
    //      endfacet
    //      ...
    //      facet normal 0.2 0.2 0.4
    //        outerloop
    //          vertex  2.0 2.3 3.4
    //          vertex  3.1 3.2 6.5
    //          vertex  4.1 5.5 9.0
    //        endloop
    //      endfacet
    //    endsolid MYSOLID
    //
    //  Reference:
    //
    //    3D Systems, Inc,
    //    Stereolithography Interface Specification,
    //    October 1989.
    //

    char* skipWhiteSpace(char *input){
    	char *next;
    	for ( next = input; *next == '\t' && *next == ' '; next++){}
    	return next;
    }

    struct ParserState {

    	ParserState(const std::string& name, std::istream& istream):
    		lineNr(0),filename(name),input_stream(istream){}

    	std::string parseErrorString(const std::string &token){
    		std::ostringstream ostr;
    		ostr << "Error parsing "<< StringUtil::quote(token) << " in file "
				 << StringUtil::quote(filename) << " at line " << lineNr;
    		return ostr.str();
    	}

    	std::string errorUnknownString(const std::string &token){
    		std::ostringstream ostr;
    		ostr << "Error unknown token "<< StringUtil::quote(token) << " in file "
				 << StringUtil::quote(filename) << " at line " << lineNr;
    		return ostr.str();
    	}

    	void getLine(char *line){
    		input_stream.getline(line, LINE_MAX_LENGTH);
    		lineNr++;
    	}


    	int lineNr;
    	std::string filename;
    	std::istream& input_stream;
    };

    // reads a vertex from a line

    void readVertex(char *line, Vector3D<float> &vertices, ParserState& state){
    	int width;
    	char token[20];
    	//char *str = skipWhiteSpace(line);
    	/*int res = */sscanf ( line, "%s%n", token, &width );
    	if ( strcmp( token, TOKEN_vertex ) ){
    		RW_THROW( state.parseErrorString(TOKEN_vertex) );
    	}
    	sscanf(line+width,"%e %e %e",&(vertices(0)),&(vertices(1)),&(vertices(2)));
    }

    void readNormal(char *line, Vector3D<float> &vertices, ParserState& state){
    	int width;
    	char token[20];
    	//char *str = skipWhiteSpace(line);
    	/*int res = */sscanf ( line, "%s%n", token, &width );
    	if ( strcmp( token, TOKEN_normal ) ){
    		RW_THROW( state.parseErrorString(TOKEN_normal) );
    	}
    	sscanf(line+width,"%e %e %e",&(vertices(0)),&(vertices(1)),&(vertices(2)));
    }

    void ReadAsciiSTL(
        std::istream& input_stream,
        PlainTriMesh<TriangleN1<float> >& result,
        ParserState &state)
    {
        //Start by storing the current locale. This is retrieved by passing NULL to setlocale	
	    std::string locale = setlocale(LC_ALL, NULL); 
        setlocale( LC_ALL, "C" );

        bool endReached = false;

        char *next;
        float r1,r2,r3,r4;
        char  token[LINE_MAX_LENGTH];
        int   width;
        char input[LINE_MAX_LENGTH];
        state.getLine(input);

        // while characters still exists and no errors occour
        while (input_stream.fail() == 0 && input_stream.eof() == 0) {
            //  Read the next line of the file into INPUT.
            state.getLine(input);

            //  Advance to the first nonspace character in INPUT.
            // 32==SPACE character
            for ( next = input; *next != '\0' && *next == 32; next++){}
            //  Skip blank lines and comments and linebreaks.
            if ( *next == '\0' || *next == '#' || *next == '!' || *next == '$') {
                continue;
            }

            //  Extract the first word in this line.
            sscanf ( next, "%s%n", token, &width );
            //  Set NEXT to point to just after this token.
            next = next + width;  

            //  FACET
            TriangleN1<float> face;
            if ( !strcmp( token, "facet" ) ){
                //  Get the XYZ coordinates of the normal vector to the face.
                readNormal(next, face.getFaceNormal(),state);
                state.getLine(input);

                // Get the XYZ coordinates of the vertex1 vector
                state.getLine(input);
                readVertex(input,face[0],state);

                // Get the XYZ coordinates of the vertex2 vector
                state.getLine(input);
                readVertex(input,face[1],state);

                // Get the XYZ coordinates of the vertex3 vector
                state.getLine(input);
                readVertex(input,face[2],state);

                // closeloop
                state.getLine(input);
                // endfacet
                state.getLine(input);

                // save the facet in the vector
                result.add(face);
            } else if ( !strcmp( token, "color" ) ) { //  COLOR
                sscanf ( next, "%*s %f %f %f %f", &r1, &r2, &r3, &r4 );
            } else if ( !strcmp( token, "solid" ) ) { // SOLID
                // object_num = object_num + 1;
            } else if ( !strcmp( token, "endsolid" ) ){ // ENDSOLID
            	endReached = true;
            } else { //  Unexpected or unrecognized.
                setlocale( LC_ALL, locale.c_str());
                RW_THROW( state.errorUnknownString( token ) );
            }
        }
        if(!endReached){
        	RW_THROW("The 'endsolid' keyword was not found in end of file. "
        			 "The file may be damaged or is a binary STL format. "
        			 "A binary STL file must not have 'solid' keyword in header.");
        }
        setlocale( LC_ALL, locale.c_str());
        return;
    }

    void ReadSTLHelper(
        std::ifstream &streamIn,
        PlainTriMesh<TriangleN1<float> >& result,
        ParserState &state)
    {
        char solidkey[6];
        // Determine if it's a binary or ASCII STL file. ASCII start with
        // "solid" keyword
        streamIn.get(solidkey, 6);
        streamIn.seekg(0, std::ios::beg);

        if(!strcmp(solidkey,"solid")){
            ReadAsciiSTL(streamIn, result, state);
        } else {
            Reader reader(&streamIn);
            ReadBinarySTL(reader, result);
        }
    }

    /**
     * @brief outputs a face in STL format given by 3 vertices
     * and a normal that defines the face.
     */
    template <class T>
    void writeFaceSTL(const rw::math::Vector3D<T>& v1,
                             const rw::math::Vector3D<T>& v2,
                             const rw::math::Vector3D<T>& v3,
                             const rw::math::Vector3D<T>& n,
                             std::ostream& ostr){
        ostr << " facet normal "
            << n[0] << " " << n[1] << " " << n[2] << std::endl;
        ostr << "  outer loop " << std::endl;
        ostr << "   vertex "
            << v1[0] << " " << v1[1] << " " << v1[2] << std::endl;
        ostr << "   vertex "
            << v2[0] << " " << v2[1] << " " << v2[2] << std::endl;
        ostr << "   vertex "
            << v3[0] << " " << v3[1] << " " << v3[2] << std::endl;
        ostr << "  endloop " << std::endl;
        ostr << " endfacet" << std::endl;
    }

}

PlainTriMeshN1F::Ptr STLFile::load(const std::string& filename)
{

    std::ifstream streamIn(filename.c_str(), std::ios::binary);
    if (!(streamIn.is_open())){
        RW_THROW("Can't open file " << StringUtil::quote(filename));
    }

    ParserState state(filename, streamIn);

    PlainTriMesh<TriangleN1<float> >::Ptr trimesh = ownedPtr( new PlainTriMesh<TriangleN1<float> >() );
    ReadSTLHelper(streamIn, *trimesh, state);
    TriangleUtil::recalcNormals(*trimesh);
    streamIn.close();
    return trimesh;
}

void STLFile::save(const TriMesh& mesh, const std::string& filename){
    //Start by storing the current locale. This is retrieved by passing NULL to setlocale	
    std::string locale = setlocale(LC_ALL, NULL); 
    setlocale( LC_ALL, "C" );
    using namespace rw::geometry;
    std::ofstream ostr;
    ostr.open( filename.c_str() );
	if (!ostr.is_open()) {
		RW_THROW("Failed to open file \"" << filename << "\" with state "<< ostr.rdstate());
    }
    ostr << "solid ascii" << std::endl;
    for(size_t i = 0; i<mesh.getSize(); i++){
        Triangle<double> tri = mesh.getTriangle(i);
        writeFaceSTL(tri[0],tri[1],tri[2],tri.calcFaceNormal(), ostr);
    }
    ostr << "endsolid" << std::endl;
    ostr.flush();
    ostr.close();
    setlocale( LC_ALL, locale.c_str());
}

