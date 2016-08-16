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

#ifndef RW_COMMON_INIARCHIVE_HPP
#define RW_COMMON_INIARCHIVE_HPP

#include <string>
#include <iostream>
#include <fstream>

#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include "InputArchive.hpp"
#include "OutputArchive.hpp"

namespace rw {
namespace common {
	/**
	 * @brief archive for loading and saving serializable classes to an ini-file format.
	 */
	class INIArchive: public InputArchive, public virtual OutputArchive {
	public:
		/**
		 * @brief constructor
		 */
		INIArchive():_ofs(NULL),_ifs(NULL),_fstr(NULL),_isopen(false){}

		INIArchive(const std::string& filename):_ofs(NULL),_ifs(NULL),_fstr(NULL),_isopen(false)
		{
		    open(filename);
		}

		INIArchive(std::ostream& ofs):_ofs(NULL),_ifs(NULL),_fstr(NULL),_isopen(false)
		{
			open(ofs);
		}

		//! destructor
		virtual ~INIArchive(){
			close();
		}

		//! @brief close this archive for streaming
		void close();

		//! @copydoc Archive::isOpen
		bool isOpen(){ return _isopen; };

		//! @copydoc Archive::flush
		void flush();

protected:

		static const int MAX_LINE_WIDTH = 1000;

		//////////////////// SCOPE
		//! @copydoc OutputArchive::writeEnterScope
		void doWriteEnterScope(const std::string& id);

		//! @copydoc OutputArchive::writeLeaveScope
		void doWriteLeaveScope(const std::string& id);

		//! @copydoc InputArchive::readEnterScope
		void doReadEnterScope(const std::string& id);

		//! @copydoc InputArchive::readLeaveScope
		void doReadLeaveScope(const std::string& id);

		void doOpenArchive(const std::string& filename);
		void doOpenArchive(std::iostream& stream);
		void doOpenOutput(std::ostream& ofs);
		void doOpenInput(std::istream& ifs);

		///////////////////////// WRITING
		virtual void doWrite(bool val, const std::string& id){
			if(val) doWrite((int)1,id);
			else doWrite((int)0,id);
		}
		void doWrite(boost::int8_t val, const std::string& id);
		void doWrite(boost::uint8_t val, const std::string& id);

		void doWrite(boost::int16_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::uint16_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::int32_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::uint32_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::int64_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::uint64_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(float val, const std::string& id){ writeValue(val,id);};
		void doWrite(double val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::string& val, const std::string& id){ writeValue(val,id);};

		void doWrite(const std::vector<bool>& val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::vector<boost::int8_t>& val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::vector<boost::uint8_t>& val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::vector<boost::int16_t>& val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::vector<boost::uint16_t>& val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::vector<boost::int32_t>& val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::vector<boost::uint32_t>& val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::vector<boost::int64_t>& val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::vector<boost::uint64_t>& val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::vector<float>& val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::vector<double>& val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::vector<std::string>& val, const std::string& id){ writeValue(val,id);};

		//template<class T>
		//void doWrite(const T& data, const std::string& id){ OutputArchive::write<T>(data,id); }

		template<class T>
		void writeValue( const std::vector<T>& val, const std::string& id ){
			(*_ofs) << id << "=";
			BOOST_FOREACH(const T& rval, val){ (*_ofs) << rval << " "; }
			(*_ofs) << "\n";
		}

		template<class T>
		void writeValue( const T&  val, const std::string& id ){
			(*_ofs) << id << "=" << val << "\n";
		}


		//template<class T>
		//void doWrite(const T& object, const std::string& id){
		//	((OutputArchive*)this)->write<T>(object, id);
		//}
		///////////////// READING

		std::pair<std::string, std::string> getNameValue(){
			std::string line(_line);
			for(size_t i=0;i<line.size();i++){
				if(line[i]== '='){
					char nname[100],nval[100];
					// split is at i
					std::string name = line.substr(0,i);
					std::string val = line.substr(i+1,line.size()-1);

					sscanf(name.c_str(), "%s",nname);
					sscanf(val.c_str(), "%s",nval);

					return std::make_pair(nname,val);
				}
			}
			RW_THROW("Not valid ini property! From line: " << line);
			return std::make_pair("","");
		}

		virtual void doRead(bool& val, const std::string& id);
		virtual void doRead(boost::int8_t& val, const std::string& id);
		virtual void doRead(boost::uint8_t& val, const std::string& id);
		virtual void doRead(boost::int16_t& val, const std::string& id){readValue<boost::int16_t>(val,id);}
		virtual void doRead(boost::uint16_t& val, const std::string& id){readValue<boost::uint16_t>(val,id);}
		virtual void doRead(boost::int32_t& val, const std::string& id){readValue<boost::int32_t>(val,id);}
		virtual void doRead(boost::uint32_t& val, const std::string& id){readValue<boost::uint32_t>(val,id);}
		virtual void doRead(boost::int64_t& val, const std::string& id){readValue<boost::int64_t>(val,id);}
		virtual void doRead(boost::uint64_t& val, const std::string& id){readValue<boost::uint64_t>(val,id);}
		virtual void doRead(float& val, const std::string& id){readValue<float>(val,id);}
		virtual void doRead(double& val, const std::string& id){readValue<double>(val,id);}
		virtual void doRead(std::string& val, const std::string& id);

		virtual void doRead(std::vector<bool>& val, const std::string& id){readValue(val,id);}
		virtual void doRead(std::vector<boost::int8_t>& val, const std::string& id){readValue(val,id);}
		virtual void doRead(std::vector<boost::uint8_t>& val, const std::string& id){readValue(val,id);}
		virtual void doRead(std::vector<boost::int16_t>& val, const std::string& id){readValue(val,id);}
		virtual void doRead(std::vector<boost::uint16_t>& val, const std::string& id){readValue(val,id);}
		virtual void doRead(std::vector<boost::int32_t>& val, const std::string& id){readValue(val,id);}
		virtual void doRead(std::vector<boost::uint32_t>& val, const std::string& id){readValue(val,id);}
		virtual void doRead(std::vector<boost::int64_t>& val, const std::string& id){readValue(val,id);}
		virtual void doRead(std::vector<boost::uint64_t>& val, const std::string& id){readValue(val,id);}
		virtual void doRead(std::vector<float>& val, const std::string& id){readValue(val,id);}
		virtual void doRead(std::vector<double>& val, const std::string& id){readValue(val,id);}
		virtual void doRead(std::vector<std::string>& val, const std::string& id) ;

        //template<class T>
        //void doRead(T& object, const std::string& id){
        //    ((InputArchive*)this)->read<T>(object, id);
        //}

		 template<class T>
		 void readValue(std::vector<T>& val, const std::string& id){
		     getLine();
			std::pair<std::string,std::string> valname = getNameValue();
			if(id!=valname.first)
				RW_WARN("mismatched ids: " << id << " ---- " << valname.first);
			// read from array
			std::vector<std::string> result;
			val.clear();
			boost::split(result, valname.second, boost::is_any_of("\t "));
			BOOST_FOREACH(std::string& rval, result){
				if(rval.empty())
					continue;
				//std::cout << rval << " : ";
				val.push_back( boost::lexical_cast<T>(rval) );
			}
		 }


		 template<class T>
		 void readValue(T& val, const std::string& id){
		     getLine();

			std::pair<std::string,std::string> valname = getNameValue();
			if(id!=valname.first)
				RW_WARN("mismatched ids: " << id << " ---- " << valname.first);
			//std::cout << "Reading: " << id << "= " << valname.second << "\n";
			val = boost::lexical_cast<T>(valname.second);
		 }

		 bool getLine();

	private:
		std::string getScope(){
			if(_scope.size()==0)
				return "";
			std::stringstream sstr;
			for(size_t i=0;i<_scope.size()-1;i++)
				sstr << _scope[i] << ".";
			sstr << _scope.back();
			return sstr.str();
		}
	private:
		std::ostream *_ofs;
		std::istream *_ifs;
		std::fstream *_fstr;
		std::iostream *_iostr;
		char _line[MAX_LINE_WIDTH];
		bool _isopen;
		std::vector<std::string> _scope;
	};

}}

#endif
