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

#ifndef RW_LOADERS_COLLADALOADER_HPP
#define RW_LOADERS_COLLADALOADER_HPP


#include <rw/trajectory/Path.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <string>
#include <stack>

#include "Dae.hpp"

#include <rw/plugin/PluginFactory.hpp>


namespace rw {
namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

    /**
     * @brief Enables loading Collada as a WorkCell.
     *
     * @note Not all features of Collada is supported in the WorkCell format and as such these are not parsed
     */
    class ColladaLoader: public rw::loaders::WorkCellLoader
    {
    public:
        /**
         * @brief Constructor
         * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
         */
        ColladaLoader(const std::string& schemaFileName = "");

        /**
         * @brief Destructor
         */
        virtual ~ColladaLoader();

        /**
         * @brief parse \b filename
         * @param filename [in] The file to load
         */
        rw::models::WorkCell::Ptr loadWorkCell(const std::string& filename);

        /**
         * @brief parse input from \b instream
         *
         * It is possible to specify whether to use the default schema which is the default behavior. If a
         * schema is specified in the XML-file or no schema should be used set \b useDefaultSchema to false.
         *
         * Throw rw::common::Exception if reading the path fails
         *
         * @param instream [in] The input stream to read from
         */
        rw::models::WorkCell::Ptr loadWorkCell(std::istream& instream);

        /**
         * @brief load in path in \b element.
         *
         * No validation is applied hence the syntax of the element is assumed correct.
         *
         * If loading the path fails an exception is thrown
         *
         * @param element [in] DOMElement representing the path
         */
        rw::models::WorkCell::Ptr loadWorkCell(xercesc::DOMElement* element);

        /**
         * @brief get the workcell that was previously loaded
         * @return
         */
        rw::models::WorkCell::Ptr getWorkCell();

        struct ParserState {
            //Dae dae;
            Dae::Collada data;
            rw::models::WorkCell::Ptr wc;
            //rw::graphics::WorkCellScene::Ptr wcscene;

            ParserState(){
                scope.push_back(&data);
            }
            //std::vector<Dae::Data*>


            Dae::Data* getData(const std::string& url){
                if(url[0]=='#'){
                    // data is inside file
                    return data.getData(url.substr(1,url.size()-1));
                } else {
                    RW_THROW("External reference not yet implemented!");
                }
                return NULL;
            }

            template <class T>
            T* get(const std::string& url){
                Dae::Data *d = getData(url);
                T* res = dynamic_cast<T*>(d);
                if(res==NULL){
                    data.printAllData();
                    RW_THROW("Data could not be found. Type:" << typeid(T).name() << " url:"<< url);
                }
                return res;
            }

            template <class T>
            T* make(){ return new T(scope.back()); }

            template <class T>
            T* make(const std::string& id, const std::string& sid){
                return new T(id, sid, scope.back());
            }

            void push(Dae::Data *d){
                scope.back()->scope.push_back(d);
                scope.push_back(d);
            }

            Dae::Data* pop(){
                Dae::Data* d = scope.back();
                scope.pop_back();
                return d;
            }

            void add(Dae::Data *d, const std::string& id, const std::string& sid){
                d->id = id;
                d->sid = sid;
                data.addData(d);
            }

            Dae::Asset* getAsset(Dae::Data* p_tmp){
                Dae::Data *p = p_tmp;
                while(p!=NULL){
                    if(p->asset!=NULL)
                        return p->asset;
                    p = p->parent;
                }
                return NULL;
            }


            Dae::Asset* getAsset(){
                for(int i=(int)scope.size()-1;i>=0;i--){
                    if(scope[i]->asset!=NULL){
                        return scope[i]->asset;
                    }
                }
                return NULL;
            }

        private:
            std::vector<Dae::Data*> scope;
        };

    private:

        void readCollada(xercesc::DOMElement* element, ParserState& data);

        Dae::Library<Dae::Camera>* readLibraryCameras(xercesc::DOMElement* element, ParserState& data);
        Dae::Library<Dae::Geometry>* readLibraryGeometries(xercesc::DOMElement* element, ParserState& data);
        Dae::Library<Dae::VisualScene>* readLibraryVisualScenes(xercesc::DOMElement* element, ParserState& data);
        Dae::Library<Dae::ArticulatedSystem>* readLibraryArticulatedSystem(xercesc::DOMElement* element, ParserState& data);
        Dae::Library<Dae::Node>* readLibraryNodes(xercesc::DOMElement* element, ParserState& data);
        Dae::Library<Dae::Material>* readLibraryMaterials(xercesc::DOMElement* element, ParserState& state);
        Dae::Library<Dae::Effect>* readLibraryEffects(xercesc::DOMElement* element, ColladaLoader::ParserState& state);
        //Dae::Library<Dae::Geometry> readLibraryGeometries(xercesc::DOMElement* element, ParserState& data);
        //Dae::Library<Dae::Geometry> readLibraryGeometries(xercesc::DOMElement* element, ParserState& data);
        //Dae::Library<Dae::Geometry> readLibraryGeometries(xercesc::DOMElement* element, ParserState& data);
        //Dae::Library<Dae::Geometry> readLibraryGeometries(xercesc::DOMElement* element, ParserState& data);

        Dae::Geometry* readGeometry(xercesc::DOMElement* element, ParserState& data);
        Dae::VisualScene* readVisualScene(xercesc::DOMElement* element, ParserState& data);
        Dae::Camera* readCamera(xercesc::DOMElement* element, ParserState& data);
        Dae::Node* readNode(xercesc::DOMElement* element, ParserState& data);

        void readScene(xercesc::DOMElement* element, ParserState& data);
        Dae::InstanceVisualScene* readInstanceVisualScene(xercesc::DOMElement* element, ParserState& state);


        void readColladaWorkCell(xercesc::DOMElement* element);

        rw::models::WorkCell::Ptr _workcell;

        ParserState _pstate;

        std::string _schemaFileName;
    };



    class ColladaLoaderPlugin: public rw::plugin::PluginFactory<rw::loaders::WorkCellLoader> {
    public:

        ColladaLoaderPlugin():
            rw::plugin::PluginFactory<rw::loaders::WorkCellLoader>("ColladaWorkCellPlugin")
        {}

        rw::loaders::WorkCellLoader::Ptr make(){
            return rw::common::ownedPtr( new ColladaLoader() );
        }

        rw::loaders::WorkCellLoader::Ptr make(const std::string&){
            return rw::common::ownedPtr( new ColladaLoader() );
        }



    private:

    };


    /** @} */


} //end namespace loaders
} //end namespace rw

#endif //enc include guard
