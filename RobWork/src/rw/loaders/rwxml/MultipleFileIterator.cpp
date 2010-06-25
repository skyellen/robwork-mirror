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
    _line = other._line;
    return(*this);
}
