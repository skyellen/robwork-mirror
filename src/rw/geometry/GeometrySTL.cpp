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

#include "GeometrySTL.hpp"
#include "Face.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

#include <iostream>
#include <string>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <fstream>

using namespace rw::geometry;
using namespace rw::common;

namespace
{
    // We have this utility here so that we can be sure that IO errors don't
    // take place. It is nasty for IO errors to not be discovered. Also we can
    // speed up things by reading the input stream into a buffer if we want to
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

    void readVec(float vec[3], Reader& input)
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
        std::vector< Face<float> >& result)
    {
        input.ignore(80);

        //
        //  Number of faces.
        //
        int face_num;
        input.read((char*)&face_num, 4);

        //
        //  For each (triangular) face,
        //    components of normal vector,
        //    coordinates of three vertices,
        //    2 byte "attribute".
        //

        for (int cnt = 0; cnt < face_num; cnt++) {
            Face<float> face;
            readVec(face._normal, input);
            readVec(face._vertex1, input);
            readVec(face._vertex2, input);
            readVec(face._vertex3, input);
            result.push_back(face);

            // Read 2 byte attribute - which is not used
            input.get();
            input.get();
        }
    }

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
    void ReadAsciiSTL(
        std::istream& input_stream,
        std::vector< Face<float> >& result)
    {
        const int LINE_MAX_LENGTH = 200;

        char *next;
        float r1,r2,r3,r4;
        char  token[LINE_MAX_LENGTH];
        int   width;
        char input[LINE_MAX_LENGTH];
        input_stream.getline(input, LINE_MAX_LENGTH);
        // while characters still exists and no errors occour
        while (input_stream.fail() == 0 && input_stream.eof() == 0) {
            //  Read the next line of the file into INPUT.
            input_stream.getline(input, LINE_MAX_LENGTH);
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
            Face<float> face;
            if ( !strcmp( token, "facet" ) ){
                //  Get the XYZ coordinates of the normal vector to the face.
                sscanf(
                    next,
                    "%*s %e %e %e",
                    &(face._normal[0]),
                    &(face._normal[1]),
                    &(face._normal[2]));
                input_stream.getline(input, LINE_MAX_LENGTH);

                // Get the XYZ coordinates of the vertex1 vector
                input_stream.getline(input, LINE_MAX_LENGTH);
                sscanf (
                    next,
                    "%*s %e %e %e",
                    &(face._vertex1[0]),
                    &(face._vertex1[1]),
                    &(face._vertex1[2]));

                // Get the XYZ coordinates of the vertex2 vector
                input_stream.getline(input, LINE_MAX_LENGTH);
                sscanf(
                    next,
                    "%*s %e %e %e",
                    &(face._vertex2[0]),
                    &(face._vertex2[1]),
                    &(face._vertex2[2]));

                // Get the XYZ coordinates of the vertex3 vector
                input_stream.getline(input, LINE_MAX_LENGTH);
                sscanf(
                    next,
                    "%*s %e %e %e",
                    &(face._vertex3[0]),
                    &(face._vertex3[1]),
                    &(face._vertex3[2]));

                // closeloop
                input_stream.getline(input, LINE_MAX_LENGTH);
                // endfacet
                input_stream.getline(input, LINE_MAX_LENGTH);

                // save the facet in the vector
                result.push_back(face);
            } else if ( !strcmp( token, "color" ) ) { //  COLOR
                sscanf ( next, "%*s %f %f %f %f", &r1, &r2, &r3, &r4 );
            } else if ( !strcmp( token, "solid" ) ) { // SOLID
                // object_num = object_num + 1;
            } else if ( !strcmp( token, "endsolid" ) ){ // ENDSOLID
            } else { //  Unexpected or unrecognized.
                RW_THROW("Reading ASCII STL file: "
             "First word " << StringUtil::quote(token)
             << " on line is unrecognized.");
            }
        }
        return;
    }

    void ReadSTLHelper(std::ifstream &streamIn, std::vector< Face<float> >& result)
    {
        char solidkey[6];
        // Determine if it's a binary or ASCII STL file. ASCII start with
        // "solid" keyword
        streamIn.get(solidkey, 6);
        streamIn.seekg(0, std::ios::beg);

        if(!strcmp(solidkey,"solid")){
            ReadAsciiSTL(streamIn, result);
        } else {
            Reader reader(&streamIn);
            ReadBinarySTL(reader, result);
        }
    }
}

void GeometrySTL::ReadSTL(
    const std::string &filename, std::vector< Face<float> >& result)
{
    std::ifstream streamIn(filename.c_str(), std::ios::binary);
    if (!(streamIn.is_open())){
        RW_THROW("Can't open file " << StringUtil::quote(filename));
    }
    ReadSTLHelper(streamIn, result);
    streamIn.close();
}
