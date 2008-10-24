/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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

#include "Tag.hpp"

#include <rw/math/Q.hpp>

#include <boost/spirit.hpp>
#include <boost/spirit/core.hpp>
#include <boost/spirit/tree/common.hpp>
#include <boost/spirit/tree/ast.hpp>
#include <boost/spirit/tree/parse_tree.hpp>

using namespace boost::spirit;

#define NS rw::loaders
using namespace NS;

using namespace rw::math;
using namespace rw::common;

namespace
{
    struct TULGrammar : public grammar<TULGrammar>
    {
        enum node_types {
            tagID = 1,
            attributeID,
            configurationID,
            numberID,
            stringID,
            attributelistID,
            keyID,
            fileID
        };

        template <typename ScannerT>
        struct definition
        {
        private:
            rule<ScannerT> tul;
            rule<ScannerT, parser_context<>, parser_tag<tagID> > tag;
            rule<ScannerT, parser_context<>, parser_tag<fileID> > file;
            rule<ScannerT, parser_context<>, parser_tag<attributeID> > attribute;
            rule<ScannerT, parser_context<>, parser_tag<configurationID> > configuration;
            rule<ScannerT, parser_context<>, parser_tag<numberID> > number;
            rule<ScannerT, parser_context<>, parser_tag<stringID> > quoted_string;
            rule<ScannerT, parser_context<>, parser_tag<attributelistID> > attributelist;
            rule<ScannerT, parser_context<>, parser_tag<keyID> > attribute_key;

            rule<ScannerT> attribute_value;
            rule<ScannerT> header, header_line;
            rule<ScannerT> optional_newlines, required_newlines;

        public:
            definition(TULGrammar const& /*self*/)
            {
                tul =
                    // Some new lines.
                    discard_node_d[optional_newlines] >>

                    // The header
                    discard_node_d[header] >>

                    // Some new lines.
                    discard_node_d[optional_newlines] >>

                    // The tags or File commands.
                    *( (file | tag) >> discard_node_d[optional_newlines] ) >>

                    // End of file.
                    discard_node_d[end_p];

                // We allow the file to have no header lines.
                header = *(header_line >> discard_node_d[required_newlines]);

                header_line =
                    (str_p("NameOfData") >> quoted_string) |
                    (str_p("DateOfData") >> quoted_string) |
                    (str_p("Scale") >> number) |
                    (str_p("TypeOfData") >> quoted_string) |
                    (str_p("NrOfFrames") >> int_p);

                // The File command.
                //
                file = root_node_d[str_p("File")] >> quoted_string;

                // A single tag.
                tag =
                    // Start token.
                    root_node_d[ch_p('{')] >>

                    // We kindly allow newlines here.
                    discard_node_d[optional_newlines] >>

                    // The name.
                    quoted_string >>

                    // New lines.
                    discard_node_d[required_newlines] >>

                    // Attributes line.
                    /* We simply omit this requirement and treat the
                       "Attributes" keyword simply as if it was an attribute
                       declaration. This is not a bug, in my opinion. We simply
                       choose to be more lenient than TUL.

                       discard_node_d[str_p("Attributes")] >>
                       discard_node_d[required_newlines] >>
                    */

                    // The attributes.
                    attributelist >>

                    // End token.
                    discard_node_d[ch_p('}')];

                attributelist = *attribute;

                // An attribute.
                attribute =
                    attribute_key >>
                    *attribute_value >>
                    discard_node_d[required_newlines];

                // The name of the attribute.
                attribute_key =
                    lexeme_d
                    [
                        // Group into a single node.
                        token_node_d
                        [
                            ( alpha_p |
                              ch_p('.') |
                              ch_p('_') |
                              ch_p('#') )
                            >> *(
                                alnum_p |
                                ch_p('.') |
                                ch_p('_')
                                )
                        ]
                    ];

                // One of the values for an attribute. So far we support only
                // numbers, vectors and strings. I can't think of other types
                // right now.
                attribute_value =
                    configuration |
                    quoted_string |
                    number;

                // A vector of values (x1, x2, ...). At least one xi is
                // required, but we should support vectors of length zero at
                // some point also.
                configuration =
                    root_node_d[ch_p('(')] >>
                    (number % discard_node_d[ch_p(',')]) >>
                    discard_node_d[ch_p(')')];

                // A string in quotes. Beware the use of lexeme_d.
                quoted_string =
                    lexeme_d
                    [
                        // Group into a single node.
                        token_node_d
                        [
                            ch_p('"') >>
                            *(anychar_p - '"') >>
                            ch_p('"')
                            ]
                        ];

                // At least one newline.
                required_newlines = +eol_p;

                // Zero or more newline.
                optional_newlines = *eol_p;

                // Tagged reals.
                number = real_p;
            }

            rule<ScannerT> const& start() const { return tul; }
        };
    };

    // We want a parser that skips spaces except newline. I am sure there is
    // something more straightforward, but this is what I have come up with:
    struct my_space_parser : public char_parser<my_space_parser>
    {
        typedef my_space_parser self_t;

        my_space_parser() {}

        template <typename CharT>
        bool test(CharT ch) const
        {
            // But not newline!
            return ch == ' ' || ch == '\t' || ch == '\r';
        }
    };

    const my_space_parser my_space_p = my_space_parser();

    typedef file_iterator<char> iterator_t;
    typedef tree_match<iterator_t>::node_t node_t;
    typedef tree_match<iterator_t>::const_tree_iterator iter_t;

    // Parse a TUL file into \a info or die with an exception.
    void parseFile(
        const std::string& filename,
        tree_parse_info<iterator_t>& info)
    {
        TULGrammar parser;
        iterator_t first(filename);
        if (!first)
            // Actually, this is triggered on empty files also...
            RW_THROW("Can't open file " << StringUtil::quote(filename));

        iterator_t last = first.make_end();
        info = ast_parse(
            first,
            last,
            parser,
            // Our parser that skips newlines and comments.
            my_space_p | "!" >> *(anychar_p - eol_p));

        if (!info.full)
            RW_THROW(
                "Parse of file " << StringUtil::quote(filename) << " failed: "
                << std::string(first, info.stop));
    }

    std::string getRawString(const iter_t& node)
    {
        return std::string(node->value.begin(), node->value.end());
    }

    std::string getString(const iter_t& node)
    {
        RW_ASSERT(node->value.id() == TULGrammar::stringID);
        const std::string& s = getRawString(node);

        RW_ASSERT(
            s.size() >= 2 &&
            s[0] == '"' &&
            s[s.size() - 1] == '"');

        return s.substr(1, s.length() - 2);
    }

    double getNumber(const iter_t& node)
    {
        RW_ASSERT(node->value.id() == TULGrammar::numberID);
        const std::string& val = getRawString(node);
        return atof(val.c_str());
    }

    Q getQ(const iter_t& node)
    {
        RW_ASSERT(node->value.id() == TULGrammar::configurationID);

        Q q(node->children.size());

        int pos = 0;
        for (iter_t p = node->children.begin(); p != node->children.end(); ++p) {
            q[pos++] = getNumber(p);
        }

        return q;
    }

    template <class T>
    boost::shared_ptr<PropertyBase> makeProperty(
        const std::string& key,
        const std::string& description,
        const T& value)
    {
        return boost::shared_ptr<PropertyBase>(
            new Property<T>(
                key,
                description,
                value));
    }

    boost::shared_ptr<PropertyBase> getProperty(
        const std::string& key, const iter_t& node)
    {
        if (node->value.id() == TULGrammar::configurationID)
            return makeProperty<Q>(key, "Q", getQ(node));
        else if (node->value.id() == TULGrammar::numberID)
            return makeProperty<double>(key, "double", getNumber(node));
        else if (node->value.id() == TULGrammar::stringID)
            return makeProperty<std::string>(key, "string", getString(node));
        else {
            RW_ASSERT(!"Impossible: Unknown node type.");
            // To avoid a compiler warning.
            return makeProperty<Q>(key, "", getQ(node));
        }
    }

    std::string getKey(const iter_t& node)
    {
        RW_ASSERT(node->value.id() == TULGrammar::keyID);
        return getRawString(node);
    }

    Tag::PropertyList getPropertyList(const std::string& key, const iter_t& node)
    {
        RW_ASSERT(node->value.id() == TULGrammar::attributeID);
        RW_ASSERT(node->children.size() >= 1);

        Tag::PropertyList result;
        iter_t p = node->children.begin();

        // Skip the 'key' element.
        for (++p; p != node->children.end(); ++p) {
            result.push_back(getProperty(key, p));
        }
        return result;
    }

    std::string replaceBackslash(const std::string& str)
    {
        std::string result = str;
        for (int i = 0; i < (int)str.length(); i++) {
            if (str.at(i) == '\\')
                result.at(i) = '/';
        }
        return result;
    }

    std::string getFileName(const std::string& dir, const iter_t& node)
    {
        const std::string& file = replaceBackslash(getString(node));

        if (StringUtil::isAbsoluteFileName(file))
            return file;
        else
            return dir + file;
    }

    void loadTagFileHelper(const std::string& file, std::vector<Tag>& result);

    void insertAttribute(const iter_t& node, Tag& tag)
    {
        // A key and no values.
        if (node->value.id() == TULGrammar::keyID) {
            const std::string& key = getKey(node);
            tag.getPropertyMap()[key];
        }

        // A list of values.
        else if (node->value.id() == TULGrammar::attributeID) {
            RW_ASSERT(node->children.size() >= 1);
            const std::string& key = getKey(node->children.begin());
            const Tag::PropertyList vals = getPropertyList(key, node);
            tag.getPropertyMap()[key] = vals;
        }

        else {
            RW_ASSERT(!"Impossible: Neither single key nor attribute.");
        }
    }

    Tag getTag(const std::string& file, const iter_t& node)
    {
        RW_ASSERT(node->value.id() == TULGrammar::tagID);
        RW_ASSERT(node->children.size() >= 1);

        iter_t node_name = node->children.begin();

        RW_ASSERT(node_name->value.id() == TULGrammar::stringID);
        const std::string& name = getString(node_name); // Names are quoted.

        // The resulting tag.
        Tag tag(name, file);

        iter_t node_attributes = node->children.begin(); ++node_attributes;

        // No attributes.
        if (node_attributes == node->children.end()) {
            return tag;
        }

        // A single attribute.
        else if (
            node_attributes->value.id() == TULGrammar::attributeID ||
            node_attributes->value.id() == TULGrammar::keyID)
        {
            insertAttribute(node_attributes, tag);
            return tag;
        }

        // Multiple attributes.
        else if (node_attributes->value.id() == TULGrammar::attributelistID) {
            for (iter_t p = node_attributes->children.begin();
                 p != node_attributes->children.end(); ++p)
            {
                insertAttribute(p, tag);
            }
            return tag;
        }

        else {
            RW_ASSERT(!"Impossible: There should be attributes in the tag.");
            // To avoid a compiler warning.
            return tag;
        }
    }

    void getTagOrTagFile(
        const std::string& file,
        const iter_t& node,
        std::vector<Tag>& result)
    {
        // Files are read relative to here:
        const std::string& dir = StringUtil::getDirectoryName(file);

        // If 'File' command:
        if (node->value.id() == TULGrammar::fileID) {
            RW_ASSERT(node->children.size() == 1);
            loadTagFileHelper(getFileName(dir, node->children.begin()), result);
        }

        // If tag:
        else if (node->value.id() == TULGrammar::tagID) {
            result.push_back(
                getTag(file, node));
        }

        else {
            RW_ASSERT(!"Impossible: Neither File command nor tag.");
        }
    }

    // Traverse the parse tree (for a current file of \a file) and build the
    // list of tags.
    void getTagList(
        const std::string& file,
        const iter_t& node,
        std::vector<Tag>& result)
    {
        // If just a single tag or single file command:
        if (node->value.id() == TULGrammar::tagID ||
            node->value.id() == TULGrammar::fileID)
        {
            getTagOrTagFile(file, node, result);
        }

        // If a number of entries or no entries at all:
        else {
            for (iter_t p = node->children.begin(); p != node->children.end(); ++p) {
                getTagOrTagFile(file, p, result);
            }
        }
    }

    // We are not using these utilities right now, but they are great to have
    // handy when debugging.

    std::string getType(const iter_t& node)
    {
        if (node->value.id() == TULGrammar::tagID) return "tagID";
        if (node->value.id() == TULGrammar::fileID) return "fileID";
        if (node->value.id() == TULGrammar::attributeID) return "attributeID";
        if (node->value.id() == TULGrammar::configurationID) return "configurationID";
        if (node->value.id() == TULGrammar::numberID) return "numberID";
        if (node->value.id() == TULGrammar::stringID) return "stringID";
        if (node->value.id() == TULGrammar::attributelistID) return "attributelistID";
        if (node->value.id() == TULGrammar::keyID) return "keyID";

        return "(unknown)";
    }

    void loadTagFileHelper(const std::string& file, std::vector<Tag>& result)
    {
        tree_parse_info<iterator_t> info;
        parseFile(file, info);
        getTagList(file, info.trees.begin(), result);
    }
}

std::vector<Tag> NS::loadTagFile(const std::string& file)
{
    std::vector<Tag> result;
    loadTagFileHelper(file, result);
    return result;
}

//----------------------------------------------------------------------
// Printing of tags to streams

namespace
{
    template <typename T>
    const T* getPropPtr(const PropertyBase& base)
    {
        const Property<T>* property = dynamic_cast<const Property<T>*>(&base);
        if (property) return &property->getValue();
        else return NULL;
    }

    void emitVal(std::ostream& out, double val)
    {
        out << val;
    }

    void emitVal(std::ostream& out, const std::string& val)
    {
        out << '"' << val << '"';
    }

    void emitVal(std::ostream& out, const Q& val)
    {
        out << "(";
        const int len = (int)val.size();
        if (len > 0) {
            int i = 0;
            out << val[i];
            for (++i; i < len; ++i) {
                out << ", " << val[i];
            }
        }
        out << ")";
    }

    template <typename T>
    bool ifHasThenEmit(
        std::ostream& out, const PropertyBase& prop)
    {
        const T* val = getPropPtr<T>(prop);
        if (val) {
            emitVal(out, *val);
            return true;
        } else
            return false;
    }

    void emitTagProperty(std::ostream& out, const PropertyBase& prop)
    {
        // Abuse the short-circuiting semantics:
        const bool ok = ifHasThenEmit<std::string>(out, prop) ||
            ifHasThenEmit<double>(out, prop) ||
            ifHasThenEmit<Q>(out, prop);

        // For all other types:
        if (!ok)
            out << "unknown:[" << prop.getDescription() << "]";
    }

    void emitPropertyList(
        std::ostream& out,
        const Tag::PropertyList& vals)
    {
        typedef Tag::PropertyList::const_iterator I;
        const I end = vals.end();
        for (I p = vals.begin(); p != vals.end(); ++p) {
            out << " ";
            emitTagProperty(out, **p);
        }
    }

    void emitTag(std::ostream& out, const Tag& tag)
    {
        out << "{ " << tag.getName() << "\n";

        typedef Tag::PropertyMap::const_iterator I;

        const I end = tag.getPropertyMap().end();
        for (I p = tag.getPropertyMap().begin(); p != end; ++p) {
            out << "    " << p->first; // The key.
            emitPropertyList(out, p->second); // The values.
            out << "\n";
        }

        out << "}\n";
    }
}

bool NS::hasAttribute(
    const Tag& tag, const std::string& key)
{
    return
        tag.getPropertyMap().find(key) !=
        tag.getPropertyMap().end();
}

int NS::getAttributeSize(const Tag& tag, const std::string& key)
{
    typedef Tag::PropertyMap::const_iterator I;
    const I p = tag.getPropertyMap().find(key);
    if (p == tag.getPropertyMap().end()) {
        RW_THROW(
            "No property named "
            << StringUtil::quote(key)
            << " in tag "
            << StringUtil::quote(tag.getName()));

        // To avoid a compiler warning.
        return -1;
    } else {
        const Tag::PropertyList& vals = p->second;
        return (int)vals.size();
    }
}

std::ostream& NS::operator<<(std::ostream& out, const Tag& tag)
{
    emitTag(out, tag);
    return out;
}
