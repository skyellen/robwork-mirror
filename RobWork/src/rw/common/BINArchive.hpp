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

#ifndef RW_COMMON_BINARCHIVE_HPP
#define RW_COMMON_BINARCHIVE_HPP

#include <string>
#include <iosfwd>

#include "InputArchive.hpp"
#include "OutputArchive.hpp"

namespace rw {
namespace common {

	/**
	 * @brief archive for loading and saving serializable classes.
	 */
	class BINArchive: public InputArchive, public virtual OutputArchive {
	public:

		//! @brief constructor
		BINArchive():_ofs(NULL),_ifs(NULL),_fstr(NULL),_iostr(NULL),_isopen(false){}

		BINArchive(std::ostream& ofs):_ofs(NULL),_ifs(NULL),_fstr(NULL),_iostr(NULL),_isopen(false)
        {
            open(ofs);
        }

		//! destructor
		virtual ~BINArchive();

		//! close streaming to archive
		void close();

		//! \copydoc rw::common::Archive::flush
		void flush();

		//! \copydoc rw::common::Archive::isOpen
		bool isOpen(){ return _isopen; };



	protected:
		void doOpenArchive(const std::string& filename);

		void doOpenArchive(std::iostream& stream);

		void doOpenOutput(std::ostream& ofs);

		void doOpenInput(std::istream& ifs);

		//////////////////// SCOPE
			// utils to handle arrays
			//! \copydoc OutputArchive::writeEnterScope
			void doWriteEnterScope(const std::string& id);

			//! \copydoc OutputArchive::writeLeaveScope
			void doWriteLeaveScope(const std::string& id);

			//! \copydoc InputArchive::readEnterScope
			void doReadEnterScope(const std::string& id);

			//! \copydoc OutputArchive::readLeaveScope
			void doReadLeaveScope(const std::string& id);

		///////////////////////// WRITING

		void doWrite(bool val, const std::string& id){
			if(val) write((int)1,id);
			else write((int)0,id);
		}
		void doWrite(boost::int8_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::uint8_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::int16_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::uint16_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::int32_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::uint32_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::int64_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(boost::uint64_t val, const std::string& id){ writeValue(val,id);};
		void doWrite(float val, const std::string& id){ writeValue(val,id);};
		void doWrite(double val, const std::string& id){ writeValue(val,id);};
		void doWrite(const std::string& val, const std::string& id);

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
		void doWrite(const std::vector<std::string>& val, const std::string& id);

		//template<class T>
		//void write(const T& data, const std::string& id){ OutputArchive::write<T>(data,id); }

		template<class T>
		void writeValue( const std::vector<T>& val, const std::string& id ){
		    boost::uint32_t s = val.size();
		    _ofs->write((char*)&s, sizeof(s) );
		    for (boost::uint32_t i = 0; i < s; i++) {
				const T& rval = val[i];
			    _ofs->write((char*)&rval, sizeof(rval) );
			}
		}

		template<class T>
		void writeValue( const T&  val, const std::string& id ){
		    _ofs->write((char*)&val, sizeof(val) );
		}

		//template<class T>
		//void write(const T& object, const std::string& id){
		//	((OutputArchive*)this)->write<T>(object, id);
		//}


		///////////////// READING

		virtual void doRead(bool& val, const std::string& id);
		virtual void doRead(boost::int8_t& val, const std::string& id){readValue<boost::int8_t>(val,id);}
		virtual void doRead(boost::uint8_t& val, const std::string& id){readValue<boost::uint8_t>(val,id);}
		virtual void doRead(boost::int16_t& val, const std::string& id){readValue<boost::int16_t>(val,id);}
		virtual void doRead(boost::uint16_t& val, const std::string& id){readValue<boost::uint16_t>(val,id);}
		virtual void doRead(boost::int32_t& val, const std::string& id){readValue<boost::int32_t>(val,id);}
		virtual void doRead(boost::uint32_t& val, const std::string& id){readValue<boost::uint32_t>(val,id);}
		virtual void doRead(boost::int64_t& val, const std::string& id){readValue<boost::int64_t>(val,id);}
		virtual void doRead(boost::uint64_t& val, const std::string& id){readValue<boost::uint64_t>(val,id);}
		virtual void doRead(float& val, const std::string& id){readValue<float>(val,id);}
		virtual void doRead(double& val, const std::string& id){readValue<double>(val,id);}
		virtual void doRead(std::string& val, const std::string& id);

		virtual void doRead(std::vector<bool>& val, const std::string& id);
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
        //void read(T& object, const std::string& id){
        //    ((InputArchive*)this)->read<T>(object, id);
        //}

		 template<class T>
		 void readValue(std::vector<T>& val, const std::string& id){
            boost::uint32_t s = 0;
            _ifs->read((char*)&s, sizeof(boost::uint32_t) );
            //std::cout << "LEN:" << s << std::endl;
            val.resize(s);
            for( boost::uint32_t i=0; i<s;i++){
                T &tmp = val[i];
                _ifs->read((char*)& (tmp), sizeof(T) );
			}
		 }


		 template<class T>
		 void readValue(T& val, const std::string& id){
		     _ifs->read((char*)&val, sizeof(val) );
		     //std::cout << val << " ";
		 }


	private:
		std::string getScope();

	private:
		std::ostream *_ofs;
		std::istream *_ifs;
		std::fstream *_fstr;
		std::iostream *_iostr;


		bool _isopen;
		std::vector<std::string> _scope;
	};
}}

#endif
