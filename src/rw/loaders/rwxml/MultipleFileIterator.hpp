#ifndef MULTIPLEFILEITERATOR_HPP_
#define MULTIPLEFILEITERATOR_HPP_

#include <vector>

#include <rw/common/StringUtil.hpp>

#include <boost/spirit.hpp>
#include <boost/spirit/core.hpp>

namespace rw { namespace loaders {
	/** @addtogroup loaders */
	/*@{*/

	/**
	 * @brief This class is used to iterate over multiple files, keeping track of 
	 * the current file and current line number at all times.
	 */
    class MultipleFileIterator : public std::iterator< std::bidirectional_iterator_tag, char>
    {
    public:

    	/**
    	 * @brief a map from a absolute iterator position to a file description.
    	 */
    	typedef std::vector< std::pair<size_t, boost::spirit::file_position > > PosToFileMap; 
    	
    	/**
    	 * @brief Constructor
    	 * @param data [in] vector of char data
    	 * @param filedata [in] data position to file info map  
    	 */
        MultipleFileIterator(
            boost::shared_ptr< std::vector< char > > data,
            boost::shared_ptr< PosToFileMap > filedata);
        
    	/**
    	 * @brief Constructor
    	 */
        virtual ~MultipleFileIterator(){};

        /**
         * @brief Increases the position of the iterator by one
         */
        MultipleFileIterator& operator++();

        /**
         * @brief Decreases the position of the iterator by one
         */        
        MultipleFileIterator& operator--();

        /**
         * @brief return reference to char that this interator points to
         */        
        char& operator*(){
            return *_pos;
        }

        /**
         * @brief return pointer to char that this interator points to
         */        
        char* operator->(){
            return _pos.operator->();
        }

        /**
         * @brief assignment operator. Assignment is straight forward.
         */
        MultipleFileIterator& operator=(const MultipleFileIterator& other);

        /**
         * @brief Tests whether the positions of two iterators are equal
         * @param other [in] VectorIterator to compare with
         * @return true if equal
         */
        bool operator==(const MultipleFileIterator& other) const
        { return _pos == other._pos; }

        /**
         * @brief Tests whether the positions of two iterators are unequal
         * @param other [in] VectorIterator to compare with
         * @return true if unequal
         */
        bool operator!=(const MultipleFileIterator& other) const
        { return _pos != other._pos; }

        /**
         * @brief returns the directory of the file that the 
         * iterator is currently iterating through
         * @return the current directory
         */
        std::string getDirectory(){
            return rw::common::StringUtil::getDirectoryName( getFile() );
        }

        /**
         * @brief returns the position in the file that the 
         * iterator is currently at
         * @return the position of the iterator in the current file
         */
        boost::spirit::file_position get_position() const {
        	//std::cout << "Get position - index( "<< _index << " ) " << _filedata->size() << std::endl; 
            size_t filepos = _filedata->size() - 1;
            for(size_t i=0;i<_filedata->size() ;i++){
            	size_t lessT = ((*_filedata)[i]).first ;
                if( _index < (int)lessT) {
                    filepos = i-1;
                    //std::cout << " ---- Lesser than: " << lessT << std::endl;
                    break;
                }
            }
            //std::cout << " --- FILENAME: " << (*_filedata)[filepos].second.file<< std::endl;
            //std::cout << " --- FILEPOS : " << filepos << std::endl;
            // TODO: calculate line and char position
            
            return (*_filedata)[filepos].second;
        }

        /**
         * @brief returns the file that the iterator is currently iterating
         * @return filename of current file
         */
        std::string getFile(){
            return get_position().file;
        }

        /**
         * @brief returns a new begin iterator
         */
        MultipleFileIterator begin(){
            MultipleFileIterator begin = *this;
            begin._pos = _data->begin();
            begin._index = _data->size()-1;
            begin._filePos = _filedata->size()-1;
            return begin;
        }

        /**
         * @brief return a new end iterator
         */
        MultipleFileIterator end(){
            MultipleFileIterator end = *this;
            end._pos = _data->end();
            end._index = _data->size()-1;
            end._filePos = _filedata->size()-1;
            return end;
        }

    private:
        // the fileposition of each
        boost::shared_ptr< std::vector< std::pair<size_t, boost::spirit::file_position > > > _filedata;
        boost::shared_ptr< std::vector< char > > _data;
        std::vector< char >::iterator _pos;
        
        int _filePos;
        int _line;
        int _index;
    };
    /*@}*/
}}

#endif /*MULTIPLEFILEITERATOR_HPP_*/
