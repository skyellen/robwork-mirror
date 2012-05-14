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


#include "KDTreeQ.hpp"

#ifdef IN_DEV

void KDTreeQ::save(const std::string& filename){
    std::ofstream output(filename.c_str());
    // we need to assign a id to each TreeNode
    std::map<TreeNode*, int> ptrToIdx;
    int i=0;
    BOOST_FOREACH(TreeNode *n, _nodes){
        ptrToIdx[n] = i;
        i++;
    }

    output << dim << "\t" << _nrOfNodes << "\n";
    BOOST_FOREACH(TreeNode *n, _nodes){
        // print each node
        // key
        for(int i=0;i<n->_kdnode->key.size();i++)
            output << n->_kdnode->key[i] << "\t";
        output << "\n";
        // value, hmm

        // deleted
        if(n->_deleted)
            output << "1\n";
        else
            output << "0\n";

        // left and right child
        output << ptrToIdx[_left] << "\t" << ptrToIdx[_right] << "\n";
        output << ((int)_axis) << "\n";
    }
}

void KDTreeQ::load(const std::string& filename){
    std::ifstream input(filename.c_str());
    int dim, nrOfNodes;
    input >> dim;
    input >> nrOfNodes;
    Q key(dim);
    for(int i=0;i<nrOfNodes;i++){
        // print each node
        // key
        for(int i=0;i<key.size();i++)
            intput >> key[i];
        // value, hmm

        // deleted
        int deleted;
        input >> deleted;

        // left and right child
        int left, right, axis;
        input >> left >> right >> axis;
    }
}


#endif

