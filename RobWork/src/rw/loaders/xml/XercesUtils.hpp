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

#ifndef RW_LOADERS_XERCESUTILS_HPP
#define RW_LOADERS_XERCESUTILS_HPP

#include <rw/common/macros.hpp>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/BinInputStream.hpp>
#include <xercesc/framework/StdOutFormatTarget.hpp>
#include <xercesc/framework/StdInInputSource.hpp>
#include <xercesc/util/XercesVersion.hpp>
#include <string>
#include <stdio.h>

namespace rw {
namespace loaders {


/** @addtogroup loaders */
/*@{*/


/**
 * @brief Utility class to help convert between Xerces unicode XMLCh* and ordinary C/C++ strings.
 *
 * The class makes sure to clean up pointers
 */
class XMLStr
{
public :
    /**
     * @brief Constructs from char array.
     */
    XMLStr(const char* const str)
    {
        _uniCodeForm = xercesc::XMLString::transcode(str);
    }

    /**
     * @brief Constructs from std::string
     */
    XMLStr(const std::string& str)
    {
        _uniCodeForm = xercesc::XMLString::transcode(str.c_str());
    }


    /**
     * @brief Constructs from double.
     *
     * The methods uses an ordinary sprintf to convert into ch*, which
     * are then converted to unicode.
     */
    XMLStr(double value) {
        char buffer[512];
        sprintf(buffer, "%f", value );
        _uniCodeForm = xercesc::XMLString::transcode(buffer);
    }

    /**
     * @brief Constructs from int.
     *
     * The methods uses an ordinary sprintf to convert into ch*, which
     * are then converted to unicode.
     */
    XMLStr(int value) {
        char buffer[128];
        sprintf(buffer, "%i", value );
        _uniCodeForm = xercesc::XMLString::transcode(buffer);
    }

    /**
     * @brief Constructs from bool.
     *
     * The methods convert true into the text "true" and false into "false"
     */
    XMLStr(bool value) {
        if (value)
            _uniCodeForm = xercesc::XMLString::transcode("true");
        else
            _uniCodeForm = xercesc::XMLString::transcode("false");
    }

    /**
     * @brief Constructs from unicode string
     */
    XMLStr(const XMLCh* ch) {
        _uniCodeForm = NULL;
        char* buf = xercesc::XMLString::transcode(ch);
        if(ch != NULL)
            _str = std::string(buf);
        xercesc::XMLString::release(&buf);
    }

    /**
     * @brief Destructor
     */
    ~XMLStr()
    {
        xercesc::XMLString::release(&_uniCodeForm);
    }


    /**
     * @brief Returns the unicode value
     *
     * If constructed with XMLStr(const XMLCh* ch) no conversion has appeared and the method
     * returns NULL
     */
    const XMLCh* uni() const
    {
        return _uniCodeForm;
    }

    /**
     * @brief Returns std::string from unicode
     *
     * Only defined when the object is constructed using XMLStr(const XMLCh* ch).
     */
    std::string str() const {
        return _str;
    }

    // we have to handle the unicode specifically
    XMLStr(const XMLStr& xmlstr){
        _uniCodeForm = xercesc::XMLString::replicate(xmlstr._uniCodeForm);
        _str = xmlstr._str;
    }
private:
    XMLCh*   _uniCodeForm;
    std::string _str;
};



/**
 * @brief Utility class which initializes Xerces upon creation
 */
class XercesInitializer
{
public:
    /**
     * @brief Initializes Xerces when constructed.
     *
     * The method may throw an exception if not able to initialize
     */
    XercesInitializer() {
        try
        {
        	std::cout << "init" << std::endl;
           xercesc::XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
        }
        catch(xercesc::XMLException& e) {
           RW_THROW("Unable to initialize Xerces used for parsing XML-files. Error Msg: "<<XMLStr(e.getMessage()).str());
        }
    }

    //! @brief Terminate Xerces when destructed.
    ~XercesInitializer() {
    	std::cout << "term" << std::endl;
    	xercesc::XMLPlatformUtils::Terminate();
    }
};


#if XERCES_VERSION_MAJOR == 3
typedef XMLFilePos XERCES_XMLFILEPOS;
typedef XMLSize_t XERCES_XMLSIZE_T;
#else
typedef unsigned int XERCES_XMLFILEPOS;
typedef unsigned int XERCES_XMLSIZE_T;
#endif //#else

/**
 * @brief XMLFormatTarget for writing to a std::ostream
 *
 * @note Used by XercesDocumentWriter
 */
class OutStreamFormatTarget: public xercesc::XMLFormatTarget {
public:
    OutStreamFormatTarget(std::ostream& out):
    _out(out)
    {
    }

    virtual void  writeChars (const XMLByte* const data, const XERCES_XMLSIZE_T count, xercesc::XMLFormatter* const formatter)
    {
        _out<<data;
    }

    virtual void flush() {
        _out.flush();
    }

private:
    std::ostream& _out;

};

/**
 * @brief BinInputStream for wrappuing a std::istream
 *
 * @note Used by XercesDocumentReader
 */
class XMLInputStream : public xercesc::BinInputStream
{


private:
    std::istream* _in;
    XERCES_XMLFILEPOS _pos;

public:
    XMLInputStream(std::istream* in) : _in(in), _pos(0) { }
    virtual XERCES_XMLFILEPOS curPos() const {
        return _pos;

    }

    virtual XERCES_XMLSIZE_T readBytes(XMLByte * const toFill, const XERCES_XMLSIZE_T maxToRead) {
		
//		XERCES_XMLSIZE_T s = _in->rdbuf()->sgetn(stdext::checked_array_iterator<char*>((char*)toFill, _countof((char*)toFill)), maxToRead);
			
        XERCES_XMLSIZE_T s = (XERCES_XMLSIZE_T)_in->rdbuf()->sgetn((char *)toFill, maxToRead);
        _pos += s;
        return s;
    }
    virtual const XMLCh* getContentType () const { return 0; }
};



/**
 * @brief Xerces input source for using std::istream
 *
 * @note Used by XercesDocumentReader
 */
class InputStreamSource : public xercesc::InputSource {
private:
    std::istream& _in;

public:
    InputStreamSource(std::istream& in) :
        InputSource(),
        _in(in)
    {}

    virtual XMLInputStream* makeStream() const {
        return new XMLInputStream(&_in);
    }
};



/**
 * @brief Utility class for reading in XML to a DOMDocument
 */
class XercesDocumentReader
{
public:
    /**
     * @brief Read in DOMDocument from file
     *
     * The \b parser has ownership of the DOMDocument returned.
     *
     * Throws a rw::common::Exception if failing to read document
     *
     * @param parser [in] Parse to use for reading in
     * @param filename [in] File to parse
     * @param schemaFileName [in] Optional schema which can be used when parsing
     * @return The DOMDocument created
     */
    static xercesc::DOMDocument* readDocument(xercesc::XercesDOMParser& parser, const std::string& filename, const std::string& schemaFileName);

    /**
     * @brief Read in DOMDocument from std::istream
     *
     * The \b parser has ownership of the DOMDocument returned.
     *
     * Throws a rw::common::Exception if failing to read document
     *
     * @param parser [in] Parse to use for reading in
     * @param instream [in] Input stream to read from
     * @param schemaFileName [in] Optional schema which can be used when parsing
     * @return The DOMDocument created
     */
    static xercesc::DOMDocument* readDocument(xercesc::XercesDOMParser& parser, std::istream& instream, const std::string& schemaFileName);

    /**
     * @brief Read in DOMDocument from xercesc::InputSource
     *
     * The \b parser has ownership of the DOMDocument returned.
     *
     * Throws a rw::common::Exception if failing to read document
     *
     * This method can be used to write to a custom output. See Xerces documentation
     * for information about how to create a XMLFormatTarget
     *
     * @param parser [in] Parse to use for reading in
     * @param source[in] The input source
     * @param schemaFileName [in] Optional schema which can be used when parsing
     * @return The DOMDocument created
     */
    static xercesc::DOMDocument* readDocument(xercesc::XercesDOMParser& parser, const xercesc::InputSource& source, const std::string& schemaFileName);
};

/**
 * @brief Utility class for writing a DOMDocument to file
 */
class XercesDocumentWriter
{

public:

    static xercesc::DOMDocument* createDocument(const XMLCh* rootName);

    /**
     * @brief Writes the content of \b doc to file named \b filename
     *
     * This method throws an exception in case of errors.
     *
     * @param doc [in] DOMDocument to write
     * @param filename [in] Desired filename
     */
    static void writeDocument(xercesc::DOMDocument* doc, const std::string& filename);

    /**
     * @brief Writes the content of \b doc to std::ostream \b out
     *
     * This method throws an exception in case of errors.
     *
     * @param doc [in] DOMDocument to write
     * @param out [in] The output stream to write to
     */
    static void writeDocument(xercesc::DOMDocument* doc, std::ostream& out);

    /**
     * @brief Writes the content of \b doc to \b formatTarget
     *
     * This method can be used to write to a custom output. See Xerces documentation
     * for information about how to create a XMLFormatTarget
     *
     * This method throws an exception in case of errors.
     *
     * @param doc [in] DOMDocument to write
     * @param formatTarget [in] The target to write to
     */
    static void writeDocument(xercesc::DOMDocument* doc, xercesc::XMLFormatTarget* formatTarget);



};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //End include guard
