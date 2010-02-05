#ifndef MATERIALDATAMAP_HPP_
#define MATERIALDATAMAP_HPP_

#include <rw/common/macros.hpp>
#include <iostream>
#include <map>
#include <rw/math/Q.hpp>
#include <vector>

namespace dynamics {

    typedef enum {Coulomb} FrictionType;

    typedef std::pair<std::string, rw::math::Q> FrictionParam;
    struct FrictionData {
        int type;
        std::vector<FrictionParam> parameters;
    };

    /**
     *
     */
    class MaterialDataMap
    {
    public:

        /**
         * @brief constructor
         */
    	MaterialDataMap();

    	/**
    	 * @brief destructor
    	 */
    	virtual ~MaterialDataMap();

    	/**
    	 * @brief add a material name with a description.
    	 * @param name [in] name of material
    	 * @param desc [in] description of material
    	 */
    	void add(std::string name, std::string desc){
    	    if( _matToMatID.find(name)==_matToMatID.end() ){
    	        _matToMatID[name] = _matCnt;
    	        _mat.push_back("");
    	        _matCnt++;
    	    }

    	    int mat = getDataID( name );
    	    _matToDesc[mat] = desc;
    	    _mat[mat] = name;
    	}

        /**
          * @brief converts a string of material type name to an int identifier.
          * @param material [in] name of material
          * @return
          */
    	int getDataID( const std::string& material ){
    	    if(_matToMatID.find(material)==_matToMatID.end() ){
    	        RW_THROW("Material \"" << material << "\" does not exist!");
    	    }
    	    return _matToMatID[material];
    	}

    	const std::string& getMaterialName( int id ){
    	    return _mat[id];
    	}

    	const std::vector<std::string>& getMaterials(){
    	    return _mat;
    	}

    	int getMaxMatID(){return _matCnt;};

        /**
         *
         * @param materialA
         * @param materialB
         * @param dataType [in] default type is 0
         * @return
         */
        const FrictionData&
            getFrictionData(int matAID, int matBID, int dataType=0);


    	/**
    	 *
    	 * @param materialA
    	 * @param materialB
    	 * @param data
    	 */
        void addFrictionData(const std::string& materialA,
                             const std::string& materialB,
                             const FrictionData& data);

        /**
         *
         * @param type
         * @return
         */
        const FrictionData& getDefaultFriction(int type){
            return _defaultFrictionData;
        }

    private:
        std::map<std::string, int> _strToIntID;

        typedef std::pair<int,int> MatIDPair;
        typedef std::map<MatIDPair, std::vector<FrictionData> > FrictionMap;
        FrictionMap _frictionMap;
        std::vector<std::string> _mat;
        std::map<std::string, int> _matToMatID;
        std::map<int, std::string> _matToDesc;
        FrictionData _defaultFrictionData;
        int _matCnt;
    };

}

#endif /*MATERIALDATAMAP_HPP_*/
