#ifndef RW_COMMON_INIARCHIVE_HPP
#define RW_COMMON_INIARCHIVE_HPP

#include <cstdlib>
#include <cmath>
#include <string>

#include <boost/any.hpp>
#include <cstdio>
#include <fstream>
#include <rw/common/macros.hpp>
#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>
#include "InputArchive.hpp"
#include "OutputArchive.hpp"

/**
 * @brief archive for loading and saving serializable classes.
 */
class INIArchive: public InputArchive, public virtual OutputArchive {
public:
    INIArchive():_ofs(NULL),_ifs(NULL),_fstr(NULL),_isopen(false){}

    void close(){
        if(_fstr!=NULL)
            _fstr->close();
    }

    virtual ~INIArchive(){
        close();
    }

    virtual void open(const std::string& filename){
        _fstr = new std::fstream(filename.c_str());
        _ofs = _fstr;
        _ifs = _fstr;
        _isopen =  _fstr->is_open();
    }

    virtual void open(std::ostream& ofs){
        _ofs = &ofs;
        _isopen = true;
    }

    virtual void open(std::istream& ifs){
        _ifs = &ifs;
        _isopen = true;
    }

    virtual bool isOpen(){ return _isopen; };

    // utils to handle arrays
    virtual void writeEnterScope(const std::string& id){
        _scope.push_back(id);
        (*_ofs) << "[" << getScope() << "]\n";
    };
    virtual void writeLeaveScope(const std::string& id){
        if(id!=_scope.back()){
            RW_THROW("Scopes has been messed up!");
        }
        _scope.pop_back();
    };
    virtual void writeEnterArray(const std::string& id){};
    virtual void writeLeaveArray(const std::string& id){};


    // utils to handle arrays
    virtual void readEnterScope(const std::string& id){
        _scope.push_back(id);
        _ifs->getline(_line, 500);
        //(*_ofs) << "[" << getScope() << "]\n";
    };
    virtual void readLeaveScope(const std::string& id){
        if(id!=_scope.back()){
            RW_THROW("Scopes has been messed up!");
        }
        _scope.pop_back();
    };
    virtual void readEnterArray(const std::string& id){};
    virtual void readLeaveArray(const std::string& id){};


    // writing primitives to archive
    virtual void write(bool val, const std::string& id){
        if(val) write((int)1,id);
        else write((int)0,id);
    }

    virtual void write(int val, const std::string& id){
        (*_ofs) << id << "=" << val << "\n";
    }

    virtual void write(boost::uint64_t val, const std::string& id){
        (*_ofs) << id << "=" << val << "\n";
    }

    virtual void write(double val, const std::string& id){
        (*_ofs) << id << "=" << val << "\n";
    }
    virtual void write(const std::string&  val, const std::string& id){
        (*_ofs) << id << "=" << val << "\n";
    }


    template<class T>
    void write(const T& data, const std::string& id){ OutputArchive::write<T>(data,id); }

    std::pair<std::string,std::string> getNameValue(){
        std::string line(_line);
        for(int i=0;i<line.size();i++){
            if(line[i]== '='){
                char nname[100],nval[100];
                // split is at i
                std::string name = line.substr(0,i);
                std::string val = line.substr(i+1,line.size()-1);

                sscanf(name.c_str(), "%s",nname);
                sscanf(val.c_str(), "%s",nval);

                return std::make_pair(nname,nval);
            }
        }
        RW_THROW("Not valid ini property!");
    }

    // writing primitives to archive
    virtual void read(bool& val, const std::string& id){
        int res = readInt(id);
        if(res==0)
            val = false;
        else
            val = true;
    }

    virtual void read(int& val, const std::string& id){
        _ifs->getline(_line,500);
        std::pair<std::string,std::string> valname = getNameValue();
        if(id!=valname.first)
            RW_WARN("mismatched ids: " << id << " ---- " << valname.first);
        val = boost::lexical_cast<int>(valname.second);

    }

    virtual void read(boost::uint64_t& val, const std::string& id){
        _ifs->getline(_line,500);
        std::pair<std::string,std::string> valname = getNameValue();
        if(id!=valname.first)
            RW_WARN("mismatched ids: " << id << " ---- " << valname.first);
        val = boost::lexical_cast<boost::uint64_t>(valname.second);

    }

    virtual void read(double &val, const std::string& id){
        _ifs->getline(_line,500);
        std::pair<std::string,std::string> valname = getNameValue();
        if(id!=valname.first)
            RW_WARN("mismatched ids: " << id << " ---- " << valname.first);
        val = boost::lexical_cast<double>(valname.second);
    }

    virtual void read(std::string&  val, const std::string& id){
        _ifs->getline(_line,500);
        std::pair<std::string,std::string> valname = getNameValue();
        //std::cout << valname.first << "  " << valname.second << std::endl;
        val = valname.second;
    }


    //
    template<class T>
    T* read(const std::string& id){
        return InputArchive::read<T>(id);
    }



private:
    std::string getScope(){
        if(_scope.size()==0)
            return "";
        std::stringstream sstr;
        for(int i=0;i<_scope.size()-1;i++)
            sstr << _scope[i] << ".";
        sstr << _scope.back();
        return sstr.str();
    }
private:
    std::ostream *_ofs;
    std::istream *_ifs;
    std::fstream *_fstr;
    char _line[500];
    bool _isopen;
    std::vector<std::string> _scope;
};


#endif
