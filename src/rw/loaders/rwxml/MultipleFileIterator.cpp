#include "MultipleFileIterator.hpp"

using namespace rw::loaders;

MultipleFileIterator::MultipleFileIterator(
    boost::shared_ptr< std::vector< char > > data,
    boost::shared_ptr< PosToFileMap > filedata)
	    : _filedata(filedata), _data(data), _pos(_data->begin()), 
	      _filePos(0), _line(0),_index(0) 
{
    	//for(int i=0;i< _filedata->size();i++){
    	//	std::cout << "index " << i << " data " << (*_filedata)[i].first << 
    	//				 " file " << (*_filedata)[i].second.file << std::endl;
    	//}
}

MultipleFileIterator& MultipleFileIterator::operator++(){
    if( *_pos == '\n' ){
        _line++;
    }
    ++_pos;
    _index++;
    return *this;
}

MultipleFileIterator& MultipleFileIterator::operator--(){
    _index--;
    --_pos;
    if( *_pos == '\n' ){
        _line--;
    }
    return *this;
}

MultipleFileIterator& MultipleFileIterator::operator=(const MultipleFileIterator& other)
{
    _filedata = other._filedata;
    _data = other._data;
    _pos = other._pos;
    _index = other._index;
    _filePos = other._filePos;
    return(*this);
}
